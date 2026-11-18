"""No serial ports. Deterministic hold, gravity, diagnostic and stop tests."""
from __future__ import annotations

import copy
import math
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from test_classic_dual_controller import FakeTransport, feedback_response, uid_response
from classic_dual_controller import DEFAULT_CONFIG, ClassicConfigStore, DualMotorController, MotionPlan
from cybergear_protocol import (
    CMD_ENABLE, CMD_MOTOR_CONTROL, CMD_PARAM_READ, CMD_PARAM_WRITE, CMD_STOP,
    HOST_ID_DEFAULT, PARAM_LIMIT_TORQUE, PARAM_RUN_MODE,
    decode_diagnostic_parameter, encode_slcan_extended, payload_param, uint_to_float,
)
from motion_policy import calibration_matches, gravity_torques, policy


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class HoldGravityTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        for cfg in self.config["joints"].values():
            cfg.update(software_zero_rad=0.0, commissioned=True, limits_confirmed=True)
        self.clock = Clock()
        self.link = FakeTransport()
        self.controller = DualMotorController(self.link, self.config, clock=self.clock)
        self.refresh()
        for key, cfg in self.config["joints"].items():
            self.controller.feed_line(uid_response(cfg["motor_can_id"], cfg["expected_uid"]))

    def refresh(self, angles=None, torque=0.0, velocity=0.0):
        angles = angles or {"elbow": 0.0, "shoulder": 0.0}
        for key, cfg in self.config["joints"].items():
            self.controller.feed_line(feedback_response(cfg["motor_can_id"], position_rad=math.radians(angles[key]),
                torque_nm=torque, velocity_rad_s=velocity, mode_status=2))
            runtime = self.controller.runtimes[key]
            runtime.parameter_values.update({PARAM_RUN_MODE: 0, PARAM_LIMIT_TORQUE: 3.5})
            runtime.parameter_times.update({PARAM_RUN_MODE: self.clock(), PARAM_LIMIT_TORQUE: self.clock()})

    def plan(self, target=0.0):
        self.controller.motion_epoch += 1
        self.controller.motion_plan = MotionPlan(
            epoch=self.controller.motion_epoch, started_at=self.clock(), duration=0.4,
            keys=("elbow", "shoulder"), start_deg={"elbow":0.0, "shoulder":0.0},
            target_deg={"elbow":target, "shoulder":target},
            torque_limit={"elbow":3.5,"shoulder":3.5}, kp={"elbow":3.0,"shoulder":3.0}, kd={"elbow":1.0,"shoulder":1.0},
            gravity_started_at=self.clock(),
        )
        self.controller.last_control_at = self.clock()
        for runtime in self.controller.runtimes.values():
            runtime.phase = "moving"
            runtime.target_deg = target

    def advance(self, duration, *, angles=None, torque=0.0, velocity=0.0):
        steps = round(duration/0.05)
        for _ in range(steps):
            self.clock.now += 0.05
            self.refresh(angles, torque, velocity)
            self.controller.control_tick()

    def calibrate_config(self):
        self.config["gravity"].update(enabled=True, calibrated=True,
            calibration_zeros_rad={k:cfg["software_zero_rad"] for k,cfg in self.config["joints"].items()},
            calibration_directions={k:cfg["direction"] for k,cfg in self.config["joints"].items()})

    def test_default_gravity_is_zero_and_enabling_requires_calibration(self):
        self.assertEqual(gravity_torques(self.config, {}), {"shoulder":0.0,"elbow":0.0})
        self.config["gravity"]["enabled"] = True
        with self.assertRaisesRegex(ValueError, "标定"):
            ClassicConfigStore.validate(self.config)

    def stationary_wait(self, sample):
        """Simulate feedback arriving during waits, without serial or wall sleeps."""
        started = self.clock()
        def wait(timeout):
            self.clock.now += timeout
            sample(round(self.clock() - started, 4))
        return patch.object(self.controller.feedback_condition, 'wait', side_effect=wait)

    def still_sample(self, elapsed, shoulder_speed=0.0, only_elbow=False):
        for key, cfg in self.config['joints'].items():
            if only_elbow and key == 'shoulder':
                continue
            self.controller.feed_line(feedback_response(cfg['motor_can_id'],
                velocity_rad_s=shoulder_speed if key == 'shoulder' else 0.0))

    def test_start_waits_for_initial_motion_to_settle_without_enabling(self):
        with self.stationary_wait(lambda t: self.still_sample(t, .20 if t < .2 else 0.0)):
            self.controller._wait_for_stationary()
        self.assertGreaterEqual(self.clock.now, 100.6)
        self.assertTrue(all(len(r.stationary_feedback) >= 4 for r in self.controller.runtimes.values()))
        self.assertTrue(self.link.sent)
        self.assertTrue(all((cid >> 24) & 31 == CMD_STOP for cid, _ in self.link.sent))

    def test_one_velocity_spike_resets_continuous_still_window(self):
        with self.stationary_wait(lambda t: self.still_sample(t, .20 if .24 < t < .26 else 0.0)):
            self.controller._wait_for_stationary()
        self.assertGreaterEqual(self.clock.now, 100.7)

    def test_persistent_motion_timeout_names_joint_speed_and_never_enables(self):
        with self.stationary_wait(lambda t: self.still_sample(t, -.0362)):
            with self.assertRaisesRegex(RuntimeError, '肩/大臂关节：速度 -0.03'):
                self.controller.start_motion({'shoulder':5.0, 'elbow':5.0})
        self.assertTrue(all((cid >> 24) & 31 == CMD_STOP for cid, _ in self.link.sent))
        self.assertIsNone(self.controller.motion_plan)

    def test_missing_joint_feedback_cannot_be_replaced_by_cached_stillness(self):
        with self.stationary_wait(lambda t: self.still_sample(t, only_elbow=True)):
            with self.assertRaisesRegex(RuntimeError, '肩/大臂关节：缺少新鲜反馈'):
                self.controller._wait_for_stationary()

    def test_single_low_speed_frame_cannot_pass_after_waiting(self):
        with self.stationary_wait(lambda t: self.still_sample(t) if t == .05 else None):
            with self.assertRaisesRegex(RuntimeError, '静止检查超时'):
                self.controller._wait_for_stationary()

    def test_feedback_gap_restarts_still_window(self):
        with self.stationary_wait(lambda t: self.still_sample(t) if t < .2 or t >= .6 else None):
            self.controller._wait_for_stationary()
        self.assertGreaterEqual(self.clock.now, 101.0)

    def test_stationary_encoder_with_isolated_speed_noise_can_pass(self):
        with self.stationary_wait(lambda t: self.still_sample(t, .087 if round(t/.05) % 4 == 0 else .01)):
            self.controller._wait_for_stationary()
        self.assertGreaterEqual(self.clock.now, 100.4)

    def test_real_position_drift_cannot_hide_behind_low_reported_velocity(self):
        def drifting(t):
            for key, cfg in self.config['joints'].items():
                self.controller.feed_line(feedback_response(cfg['motor_can_id'],
                    position_rad=.01*t, velocity_rad_s=0.0))
        with self.stationary_wait(drifting):
            with self.assertRaisesRegex(RuntimeError, '角度波动'):
                self.controller._wait_for_stationary()

    def test_back_and_forth_motion_is_not_cancelled_by_signed_speed_average(self):
        def shaking(t):
            for key, cfg in self.config['joints'].items():
                self.controller.feed_line(feedback_response(cfg['motor_can_id'],
                    position_rad=.005*math.sin(t*25), velocity_rad_s=.01))
        with self.stationary_wait(shaking):
            with self.assertRaisesRegex(RuntimeError, '静止检查超时'):
                self.controller._wait_for_stationary()

    def test_verification_retries_lost_reads_but_never_writes_or_enables(self):
        requests = []
        def read(key, index):
            requests.append(index)
            if requests.count(index) > 1:
                runtime = self.controller.runtimes[key]
                runtime.parameter_values[index] = 0 if index == PARAM_RUN_MODE else 3.5
                runtime.parameter_times[index] = self.clock()
        with self.stationary_wait(lambda t: None), patch.object(self.controller, '_request_parameter', side_effect=read):
            self.controller._verify_control_parameters('elbow', 3.5)
        self.assertGreaterEqual(requests.count(PARAM_RUN_MODE), 2)
        self.assertGreaterEqual(requests.count(PARAM_LIMIT_TORQUE), 2)
        self.assertEqual(self.link.sent, [])

    def test_horizontal_vertical_and_elbow_coupling(self):
        self.calibrate_config()
        result = gravity_torques(self.config, {"shoulder":0.0,"elbow":0.0})
        self.assertAlmostEqual(result["shoulder"], 1.45465045, places=6)
        self.assertAlmostEqual(result["elbow"], 0.05592679)
        vertical = gravity_torques(self.config, {"shoulder":90.0,"elbow":0.0})
        self.assertAlmostEqual(vertical["shoulder"], 0.0)
        folded = gravity_torques(self.config, {"shoulder":0.0,"elbow":90.0})
        self.assertAlmostEqual(folded["shoulder"], 1.39872366)
        self.assertAlmostEqual(folded["elbow"], 0.0)

    def test_zero_or_direction_change_invalidates_gravity(self):
        self.calibrate_config()
        self.config["joints"]["shoulder"]["software_zero_rad"] = 0.1
        self.assertFalse(calibration_matches(self.config))
        with self.assertRaisesRegex(RuntimeError, "失效"):
            gravity_torques(self.config, {"shoulder":0.0,"elbow":0.0})

    def test_plan_end_continues_control_and_arrival_dwell_is_display_only(self):
        self.plan()
        self.advance(0.5)
        self.assertEqual(self.controller.snapshot()["control_phase"], "controlling")
        self.assertFalse(self.controller.snapshot()["joints"]["shoulder"]["on_target"])
        self.advance(0.6)
        self.assertTrue(all(j["on_target"] for j in self.controller.snapshot()["joints"].values()))
        self.assertEqual(self.controller.snapshot()["control_phase"], "controlling")
        self.assertFalse(any((cid>>24)&31 == CMD_STOP for cid,_ in self.link.sent))
        self.assertTrue(self.controller.snapshot()["motion_active"])
        cid, payload = [f for f in self.link.sent if (f[0]>>24)&31 == CMD_MOTOR_CONTROL][-1]
        _, speed, _, _ = struct.unpack(">HHHH",payload)
        self.assertAlmostEqual(uint_to_float(speed,-30,30),0,delta=0.001)

    def test_unreached_target_continues_for_60_seconds_without_arrival_timeout(self):
        self.plan(20.0)
        self.advance(60.)
        state = self.controller.snapshot()
        self.assertFalse(state["safety_latched"])
        self.assertEqual(state["control_phase"], "controlling")
        self.assertTrue(all(not j["on_target"] for j in state["joints"].values()))
        self.assertIsNotNone(self.controller.motion_plan)
        self.assertFalse(any((cid>>24)&31 == CMD_STOP for cid,_ in self.link.sent))
        self.assertEqual({cid&255 for cid,_ in self.link.sent if (cid>>24)&31 == CMD_MOTOR_CONTROL}, {126,127})

    def test_high_speed_inside_angle_tolerance_does_not_arrive(self):
        self.plan()
        self.advance(1.2, velocity=0.05)
        self.assertEqual(self.controller.snapshot()["control_phase"], 'controlling')

    def test_stationary_encoder_with_zero_centred_speed_noise_can_arrive(self):
        self.plan()
        for step in range(140):
            self.clock.now += .01
            self.refresh(velocity=.10 if step % 2 else -.10)
            self.controller.control_tick()
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')
        self.assertTrue(all(j['arrival_stable'] for j in self.controller.snapshot()['joints'].values()))

    def test_encoder_oscillation_does_not_arrive_even_if_velocity_cancels(self):
        self.plan()
        for step in range(140):
            self.clock.now += .01
            self.refresh(angles=dict.fromkeys(('elbow', 'shoulder'), .4 if step % 2 else -.4),
                         velocity=.1 if step % 2 else -.1)
            self.controller.control_tick()
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')

    def test_cached_arrival_window_expires(self):
        self.plan()
        self.advance(1.2)
        self.assertTrue(self.controller.snapshot()['joints']['elbow']['arrival_stable'])
        self.clock.now += .31
        self.assertFalse(self.controller.snapshot()['joints']['elbow']['arrival_stable'])
        self.assertFalse(self.controller.snapshot()['joints']['elbow']['on_target'])

    def test_isolated_speed_spike_does_not_block_stable_retarget(self):
        self.plan()
        self.advance(1.2)
        self.clock.now += .01
        self.refresh(velocity=.20)
        self.assertTrue(self.controller.snapshot()['joints']['elbow']['arrival_stable'])
        self.link.sent.clear()
        self.controller.start_motion({'elbow': 5., 'shoulder': 5.})
        self.assertFalse(any((cid >> 24) & 31 in (CMD_STOP, CMD_ENABLE) for cid, _ in self.link.sent))

    def test_persistent_high_speed_cannot_hide_behind_zero_centred_average(self):
        self.plan()
        for step in range(140):
            self.clock.now += .01
            self.refresh(velocity=.20 if step % 2 else -.20)
            self.controller.control_tick()
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')

    def test_retarget_after_trajectory_does_not_require_arrival_stability(self):
        self.plan()
        self.advance(1.2)
        self.advance(.5, angles={'elbow': .5, 'shoulder': .5})
        self.assertFalse(self.controller.snapshot()['joints']['shoulder']['on_target'])
        previous = self.controller.motion_plan
        self.link.sent.clear()
        self.controller.start_motion({'elbow': 5., 'shoulder': 5.})
        self.assertIsNot(self.controller.motion_plan, previous)
        self.assertFalse(any((cid>>24)&31 in (CMD_STOP,CMD_ENABLE,CMD_PARAM_WRITE) for cid,_ in self.link.sent))

    def test_stop_cancels_hold_and_no_late_motion_frames(self):
        self.plan()
        self.advance(1.2)
        self.controller.stop_all()
        offset = len(self.link.sent)
        self.advance(0.3)
        self.assertFalse(any((cid>>24)&31 == CMD_MOTOR_CONTROL for cid,_ in self.link.sent[offset:]))
        self.assertEqual(self.controller.snapshot()["control_phase"], "stopped")

    def test_feedback_timeout_while_holding_stops_both(self):
        self.plan()
        self.advance(1.2)
        self.clock.now += .31
        self.controller.last_control_at = self.clock()  # Isolate feedback watchdog.
        self.controller.control_tick()
        self.assertIn("反馈超时",self.controller.safety_reason)
        self.assertEqual([cid&255 for cid,_ in self.link.sent if (cid>>24)&31 == CMD_STOP][-2:],[127,126])

    def test_control_loop_delay_stops_instead_of_catching_up(self):
        self.plan()
        self.clock.now += .31
        self.refresh()
        self.controller.control_tick()
        self.assertIn("控制循环超时",self.controller.safety_reason)

    def test_stall_detection_remains(self):
        self.plan(20.0)
        self.advance(2.2, torque=3.3)
        self.assertIn("堵转",self.controller.safety_reason)

    def test_large_hold_deviation_alone_no_longer_stops_in_either_direction(self):
        for angle in (0., 40.):
            with self.subTest(actual_deg=angle):
                self.setUp()
                self.plan(20.)
                self.advance(1.2, angles=dict.fromkeys(('shoulder','elbow'),20.))
                self.assertEqual(self.controller.snapshot()['control_phase'],'controlling')
                self.link.sent.clear()
                self.advance(1.5, angles=dict.fromkeys(('shoulder','elbow'),angle))
                self.assertFalse(self.controller.safety_latched)
                self.assertEqual(self.controller.snapshot()['control_phase'],'controlling')
                self.assertFalse(any((cid>>24)&31 == CMD_STOP for cid,_ in self.link.sent))
                self.assertEqual({cid&255 for cid,_ in self.link.sent if (cid>>24)&31 == CMD_MOTOR_CONTROL}, {126,127})
                self.advance(1.3, angles=dict.fromkeys(('shoulder','elbow'),20.))
                self.assertEqual(self.controller.snapshot()['control_phase'],'controlling')

    def test_removed_hold_policy_fields_are_ignored_without_rewriting_legacy_config(self):
        self.assertNotIn('hold_error_deg', policy(self.config))
        self.config['motion_policy'].update(hold_error_deg=0., hold_error_timeout_s=0., convergence_timeout_s=0.)
        original = copy.deepcopy(self.config)
        ClassicConfigStore.validate(self.config)
        self.assertNotIn('hold_error_timeout_s', policy(self.config))
        self.assertNotIn('convergence_timeout_s', policy(self.config))
        self.assertEqual(self.config, original)
        self.config['motion_policy']['unknown_guard'] = 1
        with self.assertRaisesRegex(ValueError, '字段无效'):
            ClassicConfigStore.validate(self.config)

    def test_other_faults_still_stop_both_after_large_hold_disturbance(self):
        for field,value in (('temperature_c',61),('fault_bits',1),('actual_deg',96),('mode_status',0)):
            with self.subTest(field=field):
                self.setUp()
                self.plan(20.)
                self.advance(1.2, angles=dict.fromkeys(('shoulder','elbow'),20.))
                self.advance(1.5)
                self.assertFalse(self.controller.safety_latched)
                self.link.sent.clear()
                setattr(self.controller.runtimes['shoulder'],field,value)
                self.controller.control_tick()
                self.assertTrue(self.controller.safety_latched)
                self.assertIsNone(self.controller.motion_plan)
                self.assertEqual({cid&255 for cid,_ in self.link.sent if (cid>>24)&31 == CMD_STOP}, {126,127})

    def test_gravity_ramp_and_motor_direction_conversion(self):
        self.config["joints"]["shoulder"]["direction"] = -1
        self.calibrate_config()
        self.plan()
        self.controller.control_tick()
        self.assertEqual(self.controller.runtimes["shoulder"].gravity_torque_nm,0.0)
        self.advance(.5)
        self.assertLess(self.controller.runtimes["shoulder"].gravity_torque_nm,-.70)
        self.assertGreater(self.controller.runtimes["shoulder"].gravity_torque_nm,-.75)

    def test_readback_parser_rejects_wrong_type_status_host_nan_and_unknown(self):
        cid = (CMD_PARAM_READ<<24)|(127<<8)|HOST_ID_DEFAULT
        payload = payload_param(PARAM_LIMIT_TORQUE,3.5)
        self.assertEqual(decode_diagnostic_parameter(cid,payload,HOST_ID_DEFAULT,{126,127}),(127,PARAM_LIMIT_TORQUE,3.5))
        for bad_cid,bad_payload in (
            (cid^(1<<24),payload), (cid|(1<<16),payload), (cid^1,payload),
            (cid,payload_param(PARAM_LIMIT_TORQUE,float('nan'))),
            (cid,payload[:2]+b'\x01\x00'+payload[4:]),
            (cid,payload_param(0x9999,1.0)),
        ):
            self.assertIsNone(decode_diagnostic_parameter(bad_cid,bad_payload,HOST_ID_DEFAULT,{126,127}))

    def test_unsolicited_readback_cannot_refresh_or_verify_parameter(self):
        cid = (CMD_PARAM_READ<<24)|(127<<8)|HOST_ID_DEFAULT
        self.controller.runtimes['elbow'].parameter_times.clear()
        self.controller.feed_line(encode_slcan_extended(cid,payload_param(PARAM_LIMIT_TORQUE,3.5)).decode())
        self.assertNotIn(PARAM_LIMIT_TORQUE,self.controller.runtimes['elbow'].parameter_times)

    def test_active_parameter_change_latches_and_stops(self):
        self.plan()
        self.controller._request_parameter('elbow',PARAM_LIMIT_TORQUE)
        cid = (CMD_PARAM_READ<<24)|(127<<8)|HOST_ID_DEFAULT
        self.controller.feed_line(encode_slcan_extended(cid,payload_param(PARAM_LIMIT_TORQUE,8.0)).decode())
        self.assertIn('上限读回',self.controller.safety_reason)

    def test_missing_critical_readback_stops_and_optional_values_are_unknown(self):
        self.plan()
        self.controller.runtimes['elbow'].parameter_times.clear()
        self.controller.control_tick()
        self.assertIn('诊断读回超时',self.controller.safety_reason)
        self.assertIsNone(self.controller.snapshot()['joints']['elbow']['diagnostics']['iq_a']['value'])

    def test_hold_retarget_does_not_stop_or_enable_and_keeps_other_joint(self):
        self.plan()
        self.advance(1.2)
        self.link.sent.clear()
        duration = self.controller.start_motion({'shoulder':5.0})
        self.assertGreater(duration,.4)
        self.assertEqual(self.controller.motion_plan.target_deg['elbow'],0.0)
        self.assertFalse(any((cid>>24)&31 in (CMD_STOP,CMD_ENABLE,CMD_PARAM_WRITE) for cid,_ in self.link.sent))

    def test_retarget_during_movement_is_rejected_without_stopping(self):
        self.plan(20)
        with self.assertRaisesRegex(RuntimeError,'等待当前规划轨迹结束'):
            self.controller.start_motion({'shoulder':5.0})
        self.assertIsNotNone(self.controller.motion_plan)
        self.assertEqual(self.link.sent,[])

    def test_calibration_disabled_can_be_saved_without_motion(self):
        self.controller.configure_gravity(enabled=False)
        self.assertFalse(any((cid>>24)&31 in (CMD_ENABLE,CMD_MOTOR_CONTROL) for cid,_ in self.link.sent))

    def test_invalid_policy_is_rejected(self):
        self.config['motion_policy']['arrival_dwell_s'] = float('inf')
        with self.assertRaises(ValueError):
            ClassicConfigStore.validate(self.config)

    def test_display_arrival_does_not_change_control_fields(self):
        self.plan()
        self.advance(.5)
        self.assertFalse(self.controller.snapshot()['joints']['shoulder']['on_target'])
        before = [f for f in self.link.sent if (f[0]>>24)&31 == CMD_MOTOR_CONTROL][-2:]
        self.advance(1.)
        self.assertTrue(self.controller.snapshot()['joints']['shoulder']['on_target'])
        after = [f for f in self.link.sent if (f[0]>>24)&31 == CMD_MOTOR_CONTROL][-2:]
        self.assertEqual(before, after)

    def test_retarget_keeps_previous_reference_and_loaded_joint_support(self):
        self.plan(20.)
        self.advance(12., angles={'elbow': 17., 'shoulder': 5.})
        previous = self.controller.motion_plan
        for runtime in self.controller.runtimes.values():
            runtime.load_bias_nm = .4
        self.link.sent.clear()
        self.controller.start_motion({'shoulder': 25.})
        self.assertEqual(self.controller.motion_plan.start_deg, previous.target_deg)
        self.assertEqual(self.controller.motion_plan.target_deg, {'elbow':20., 'shoulder':25.})
        self.assertTrue(all(r.load_bias_nm == .4 for r in self.controller.runtimes.values()))
        self.assertFalse(any((cid>>24)&31 in (CMD_STOP,CMD_ENABLE,CMD_PARAM_WRITE) for cid,_ in self.link.sent))
        self.controller.control_tick()
        self.assertTrue(all(r.setpoint_deg == 20. for r in self.controller.runtimes.values()))

    def test_recorded_small_error_with_oscillation_no_longer_trips(self):
        self.plan(20.)
        for step in range(600):
            self.clock.now += .05
            self.refresh(angles={'elbow':20., 'shoulder':19.52 if step % 2 else 19.36})
            self.controller.control_tick()
        state = self.controller.snapshot()
        self.assertFalse(state['safety_latched'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertFalse(state['joints']['shoulder']['on_target'])
        self.assertTrue(state['joints']['elbow']['on_target'])
        self.assertFalse(any((cid>>24)&31 == CMD_STOP for cid,_ in self.link.sent))

    def test_overtemperature_limit_and_mode_drop_still_trip_with_high_speed(self):
        for field,value in (('temperature_c',61),('actual_deg',96),('mode_status',0)):
            with self.subTest(field=field):
                self.controller.safety_latched = False
                self.plan()
                self.refresh()
                self.controller.runtimes['elbow'].velocity_rad_s = .6
                setattr(self.controller.runtimes['elbow'],field,value)
                self.controller.control_tick()
                self.assertTrue(self.controller.safety_latched)
                self.assertIsNone(self.controller.motion_plan)
                self.assertNotIn('速度', self.controller.safety_reason)

    def test_actual_speed_alone_no_longer_stops_active_motion(self):
        # Includes the recorded +0.514 rad/s spike and both directions.
        self.plan(30.)
        for velocity in (.514, -.514, .6, -.6):
            self.link.sent.clear()
            self.advance(.05, velocity=velocity)
            self.assertFalse(self.controller.safety_latched)
            self.assertIsNotNone(self.controller.motion_plan)
            self.assertFalse(any((cid >> 24) & 31 == CMD_STOP for cid, _ in self.link.sent))
            self.assertEqual({cid & 255 for cid,_ in self.link.sent if (cid >> 24) & 31 == CMD_MOTOR_CONTROL}, {126,127})

    def test_requested_speed_limit_is_not_removed(self):
        self.link.sent.clear()
        with self.assertRaisesRegex(ValueError, '速度超过配置上限'):
            self.controller.start_motion({'elbow': 5.}, speed_rad_s={'elbow': .31})
        self.assertEqual(self.link.sent, [])

    def test_feedback_over_torque_protection(self):
        self.plan()
        self.advance(.4, torque=4.5)
        self.assertIn('力矩持续超过',self.controller.safety_reason)

    def test_shoulder_elbow_signs_follow_virtual_work(self):
        self.config['gravity']['shoulder_angle_sign'] = -1
        self.config['gravity']['elbow_angle_sign'] = -1
        self.calibrate_config()
        torques = gravity_torques(self.config,{'shoulder':0.0,'elbow':0.0})
        self.assertLess(torques['shoulder'],0)
        self.assertLess(torques['elbow'],0)

    def test_small_static_residual_warns_without_falsely_arriving_or_stopping(self):
        self.plan(20.)
        self.advance(10., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        state = self.controller.snapshot()
        self.assertFalse(state['safety_latched'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertTrue(all(not j['on_target'] and j['load_bias_nm'] == 0 for j in state['joints'].values()))
        self.assertFalse(any((cid >> 24) & 31 == CMD_STOP for cid, _ in self.link.sent))
        self.advance(1.2, angles=dict.fromkeys(('elbow','shoulder'), 20.))
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')
        self.assertTrue(all(not j['notice'] for j in self.controller.snapshot()['joints'].values()))

    def test_small_residual_can_retarget_without_stop_enable_or_parameter_write(self):
        self.plan(20.)
        self.advance(9., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        self.link.sent.clear()
        self.controller.start_motion({'elbow': 17.8, 'shoulder': 17.8})
        self.assertEqual(self.controller.snapshot()['control_phase'], 'moving')
        self.assertFalse(any((cid >> 24) & 31 in (CMD_STOP, CMD_ENABLE, CMD_PARAM_WRITE) for cid, _ in self.link.sent))

    def test_held_joint_reports_recovery_and_rearrives_after_small_push(self):
        self.plan(20.)
        self.advance(1.2, angles=dict.fromkeys(('elbow','shoulder'), 20.))
        self.advance(.2, angles=dict.fromkeys(('elbow','shoulder'), 17.))
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')
        self.assertFalse(self.controller.safety_latched)
        self.advance(1.3, angles=dict.fromkeys(('elbow','shoulder'), 20.))
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')
        self.assertFalse(any((cid >> 24) & 31 == CMD_STOP for cid, _ in self.link.sent))

    def test_large_disturbance_does_not_start_a_recovery_timeout(self):
        self.plan(20.)
        self.advance(9., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        self.advance(20., angles=dict.fromkeys(('elbow','shoulder'), 5.))
        state = self.controller.snapshot()
        self.assertFalse(state['safety_latched'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertTrue(all(not j['on_target'] for j in state['joints'].values()))
        self.assertFalse(any((cid>>24)&31 == CMD_STOP for cid,_ in self.link.sent))

    def test_new_disturbance_in_warning_has_its_own_recovery_window(self):
        self.plan(20.)
        self.advance(9., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        self.advance(.2, angles=dict.fromkeys(('elbow','shoulder'), 16.))
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')
        self.assertFalse(self.controller.safety_latched)
        self.advance(1.3, angles=dict.fromkeys(('elbow','shoulder'), 20.))
        self.assertEqual(self.controller.snapshot()['control_phase'], 'controlling')

    def test_persistent_small_encoder_oscillation_only_changes_display(self):
        self.plan(20.)
        for step in range(180):
            self.clock.now += .05
            self.refresh(angles=dict.fromkeys(('elbow','shoulder'), 19.6 if step % 2 else 20.4))
            self.controller.control_tick()
        self.assertFalse(self.controller.safety_latched)
        self.assertTrue(all(not j['on_target'] for j in self.controller.snapshot()['joints'].values()))
        self.assertFalse(any((cid>>24)&31 == CMD_STOP for cid,_ in self.link.sent))

    def test_enabling_residual_compensation_requires_confirmation_and_no_motion(self):
        with self.assertRaisesRegex(ValueError, '确认'):
            self.controller.configure_load_compensation(enabled=True)
        self.controller.configure_load_compensation(enabled=True, confirmed=True)
        self.assertEqual(self.link.sent, [])
        self.assertTrue(self.controller.snapshot()['load_compensation']['enabled'])
        self.plan()
        with self.assertRaisesRegex(RuntimeError, '停止'):
            self.controller.configure_load_compensation(enabled=False)
        self.assertTrue(self.controller.snapshot()['load_compensation']['enabled'])

    def test_residual_compensation_clears_on_stop_and_watchdog_trip(self):
        self.controller.configure_load_compensation(enabled=True, confirmed=True)
        self.plan(20.)
        self.advance(2., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        self.assertTrue(all(r.load_bias_nm > 0 for r in self.controller.runtimes.values()))
        self.controller.stop_all()
        self.assertTrue(all(r.load_bias_nm == 0 for r in self.controller.runtimes.values()))
        self.plan(20.)
        self.advance(2., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        self.clock.now += .31
        self.controller.last_control_at = self.clock()
        self.controller.control_tick()
        self.assertIn('反馈超时', self.controller.safety_reason)
        self.assertTrue(all(r.load_bias_nm == 0 for r in self.controller.runtimes.values()))

    def test_residual_bias_freezes_during_encoder_motion_and_is_bumpless_on_retarget(self):
        self.controller.configure_load_compensation(enabled=True, confirmed=True)
        self.plan(20.)
        self.advance(2., angles=dict.fromkeys(('elbow','shoulder'), 17.8))
        bias = self.controller.runtimes['elbow'].load_bias_nm
        for step in range(10):
            self.clock.now += .05
            self.refresh(angles=dict.fromkeys(('elbow','shoulder'), 17.8 + .25*(step+1)))
            self.controller.control_tick()
            self.assertEqual(self.controller.runtimes['elbow'].load_bias_nm, bias)
        self.advance(1.2, angles=dict.fromkeys(('elbow','shoulder'), 20.))
        bias = self.controller.runtimes['elbow'].load_bias_nm
        self.controller.start_motion({'elbow': 25., 'shoulder': 25.})
        self.assertEqual(self.controller.runtimes['elbow'].load_bias_nm, bias)
        self.advance(.05, angles=dict.fromkeys(('elbow','shoulder'), 20.))
        after = self.controller.runtimes['elbow'].load_bias_nm
        self.assertLessEqual(abs(after-bias), .005001)

    def test_quasistatic_synthetic_load_converges_with_opt_in_bias(self):
        # A deliberately simple equilibrium model, NOT wearable validation:
        # PD alone has 0.12/3 rad = 2.29 deg residual under constant torque.
        self.controller.configure_load_compensation(enabled=True, confirmed=True)
        self.plan(20.)
        for _ in range(800):
            angles = {k:20.-math.degrees((.12-r.load_bias_nm)/3.) for k,r in self.controller.runtimes.items()}
            self.clock.now += .05
            self.refresh(angles=angles, torque=.12)
            self.controller.control_tick()
            self.assertFalse(self.controller.safety_latched)
        state = self.controller.snapshot()
        self.assertEqual(state['control_phase'], 'controlling')
        for joint in state['joints'].values():
            self.assertLess(abs(joint['error_deg']), .5)
            self.assertGreater(joint['load_bias_nm'], .09)
            self.assertLessEqual(joint['load_bias_nm'], .5)

    def test_save_failure_does_not_enable_compensation(self):
        self.controller.save_config_callback = lambda _config: (_ for _ in ()).throw(OSError('disk full'))
        with self.assertRaises(OSError):
            self.controller.configure_load_compensation(enabled=True, confirmed=True)
        self.assertFalse(self.controller.snapshot()['load_compensation']['enabled'])
        self.assertEqual(self.link.sent, [])


if __name__ == '__main__':
    unittest.main()
