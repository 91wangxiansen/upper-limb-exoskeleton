"""Wire-only diagnostic tests: never inject parameter freshness timestamps."""
import copy
import math
import unittest
from collections import Counter

from test_classic_dual_controller import FakeTransport, feedback_response, uid_response
from test_hold_gravity import Clock
from classic_dual_controller import DEFAULT_CONFIG, DualMotorController, MotionPlan
from cybergear_protocol import CMD_PARAM_READ, PARAM_RUN_MODE, PARAM_LIMIT_TORQUE, encode_slcan_extended, payload_param
from tools.verify_live_hold import stage_envelopes


class ParameterSchedulerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.link = FakeTransport()
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        for joint in cfg['joints'].values():
            joint.update(software_zero_rad=0., commissioned=True, limits_confirmed=True)
        self.controller = DualMotorController(self.link, cfg, clock=self.clock)
        self.reads = Counter()
        self.cursor = 0
        for key, joint in cfg['joints'].items():
            self.controller.feed_line(uid_response(joint['motor_can_id'], joint['expected_uid']))
            for index in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE):
                self.controller._request_parameter(key, index)
        self.reply()
        keys = ('elbow', 'shoulder')
        self.controller.motion_plan = MotionPlan(1, self.clock(), .4, keys,
            dict.fromkeys(keys, 0.), dict.fromkeys(keys, 0.), dict.fromkeys(keys, 3.5),
            dict.fromkeys(keys, 3.), dict.fromkeys(keys, 1.))
        for runtime in self.controller.runtimes.values():
            runtime.phase = 'moving'

    def response(self, motor, index, value=None, header=0):
        value = (0 if index == PARAM_RUN_MODE else 3.5) if value is None else value
        payload = payload_param(index, value, 'u8' if index == PARAM_RUN_MODE else 'float')
        return encode_slcan_extended((17 << 24) | header | (motor << 8) | 0xFD, payload).decode().strip()

    def reply(self, drop=None):
        for can_id, payload in self.link.sent[self.cursor:]:
            if (can_id >> 24) & 31 != CMD_PARAM_READ:
                continue
            motor, index = can_id & 255, int.from_bytes(payload[:2], 'little')
            self.reads[motor, index] += 1
            if index not in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE) or (drop and drop(motor, index, self.reads[motor, index])):
                continue
            self.controller.feed_line(self.response(motor, index))
        self.cursor = len(self.link.sent)

    def advance(self, seconds, drop=None):
        for _ in range(round(seconds/.01)):
            self.clock.now += .01
            for cfg in self.controller.config['joints'].values():
                self.controller.feed_line(feedback_response(cfg['motor_can_id'], mode_status=2))
            self.controller.control_tick()
            self.reply(drop)
            if self.controller.safety_latched:
                break

    def test_unsupported_optionals_do_not_starve_critical_parameters(self):
        self.advance(10.)
        self.assertFalse(self.controller.safety_latched)
        self.assertTrue(any(index not in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE) for motor, index in self.reads))
        for runtime in self.controller.runtimes.values():
            for index in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE):
                self.assertLess(self.clock()-runtime.parameter_times[index], .7)

    def test_dropped_reads_are_retried_without_parameter_writes(self):
        self.advance(10., drop=lambda motor, index, count: count % 3 != 0)
        self.assertFalse(self.controller.safety_latched)
        self.assertTrue(all((cid >> 24) & 31 in (1, 17) for cid, _ in self.link.sent))

    def test_sustained_loss_still_stops_with_specific_index(self):
        self.advance(4., drop=lambda motor, index, count: motor == 127 and index == PARAM_LIMIT_TORQUE)
        self.assertTrue(self.controller.safety_latched)
        self.assertIn('0x700B', self.controller.safety_reason)
        self.assertLessEqual(self.clock(), 103.02)
        self.assertTrue(all(r.last_feedback_at == self.clock() for r in self.controller.runtimes.values()))

    def test_missing_critical_does_not_starve_other_motor(self):
        self.advance(2.8, drop=lambda motor, index, count: motor == 127)
        for index in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE):
            self.assertLess(self.clock()-self.controller.runtimes['shoulder'].parameter_times[index], 1.)

    def test_invalid_or_unsolicited_reply_cannot_refresh_freshness(self):
        runtime = self.controller.runtimes['elbow']
        before = runtime.parameter_times[PARAM_RUN_MODE]
        self.clock.now += .1
        self.controller._request_parameter('elbow', PARAM_RUN_MODE)
        self.controller.feed_line(self.response(127, PARAM_RUN_MODE, header=0x800000))
        self.assertEqual(runtime.parameter_times[PARAM_RUN_MODE], before)
        trace = self.controller.diagnostics_snapshot()['trace'][-1]
        self.assertEqual(trace['can_id'], '11807FFD')
        self.assertEqual(trace['outcome'], 'rejected_format_status_address_or_value')
        self.controller.feed_line(self.response(127, PARAM_RUN_MODE))
        accepted = runtime.parameter_times[PARAM_RUN_MODE]
        self.clock.now += .1
        self.controller.feed_line(self.response(127, PARAM_RUN_MODE))
        self.assertEqual(runtime.parameter_times[PARAM_RUN_MODE], accepted)
        self.assertEqual(self.controller.diagnostics_snapshot()['trace'][-1]['outcome'], 'unsolicited')

    def test_real_parameter_mismatch_stops_immediately(self):
        self.controller._request_parameter('elbow', PARAM_LIMIT_TORQUE)
        self.controller.feed_line(self.response(127, PARAM_LIMIT_TORQUE, value=2.))
        self.assertTrue(self.controller.safety_latched)
        self.assertIn('不一致', self.controller.safety_reason)

    def test_trace_is_bounded(self):
        for _ in range(1100):
            self.controller._request_parameter('elbow', PARAM_RUN_MODE)
        self.assertEqual(len(self.controller.diagnostics_snapshot()['trace']), 1024)

    def test_small_live_stage_has_its_own_overshoot_boundary(self):
        bounds = stage_envelopes({'shoulder': 6., 'elbow': 29.}, {'shoulder': 11., 'elbow': 30.})
        self.assertEqual(bounds['shoulder'], (3., 14.))
        self.assertEqual(bounds['elbow'], (26., 33.))


if __name__ == '__main__':
    unittest.main()
