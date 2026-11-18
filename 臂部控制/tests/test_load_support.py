"""Software-only load plant, wire commands and support-profile regression.

The synthetic inertia/load are test fixtures, not an identified real arm.
No serial ports, real server or live motor are accessed.
"""
import copy
import math
import struct
import unittest
from unittest.mock import patch

import test_hold_gravity as fixtures
from classic_dual_controller import DEFAULT_CONFIG, ClassicConfigStore, MotionPlan
from cybergear_protocol import CMD_MOTOR_CONTROL, CMD_ENABLE, CMD_PARAM_WRITE, CMD_STOP, uint_to_float
from motion_policy import SUPPORT_PRESET, motion_gains, residual_bias


class SupportPolicyTests(unittest.TestCase):
    def setUp(self):
        self.settings = {**SUPPORT_PRESET, 'enabled': True}

    def update(self, bias=0., error=5., base=0., limit=3.5, rate=0., complete=False):
        return residual_bias(self.settings, bias, math.radians(error), base, limit, .05,
                             stationary=False, trajectory_complete=complete,
                             tracking_error_rate_rad_s=rate)

    def test_learns_during_tracking_not_only_after_stopping(self):
        self.assertGreater(self.update(), 0.)
        self.assertLess(self.update(error=-5.), 0.)
        self.assertGreater(self.update(error=12.), 0.)
        self.assertEqual(self.update(bias=.2, error=16.), .2)

    def test_missing_noisy_or_fast_motion_freezes_learning(self):
        for rate in (None, float('nan'), .13, -.13):
            self.assertEqual(self.update(bias=.2, rate=rate), .2)

    def test_retarget_keeps_support_and_zero_error_does_not_drop_it(self):
        self.assertEqual(self.update(bias=1.4, error=0.), 1.4)
        self.assertLess(self.update(bias=1.4, error=-5.), 1.4)
        self.assertLessEqual(abs(self.update(bias=1.4, error=15.)-1.4), .020001)

    def test_saturation_stops_accumulation_but_allows_unloading(self):
        self.assertEqual(self.update(bias=1., base=2.5), 1.)
        self.assertEqual(self.update(bias=-1., base=-2.5, error=-5.), -1.)
        self.assertLess(self.update(bias=1., base=3., error=-5.), 1.)
        self.assertGreater(self.update(bias=-1., base=-3., error=5.), -1.)
        self.assertLessEqual(self.update(bias=2.5, error=0., limit=.3), .3)

    def test_bias_bounds_both_directions(self):
        for sign in (-1, 1):
            bias = 0.
            for _ in range(2000):
                bias = self.update(bias=bias, error=sign*15.)
            self.assertAlmostEqual(bias, sign*2.5)

    def test_profile_gains_only_when_explicitly_enabled(self):
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        cfg['load_compensation'] = copy.deepcopy(SUPPORT_PRESET)
        self.assertEqual(motion_gains(cfg, 'shoulder'), (3., 1.))
        cfg['load_compensation']['enabled'] = True
        ClassicConfigStore.validate(cfg)
        self.assertEqual(motion_gains(cfg, 'shoulder'), (12., 1.5))


class SupportControllerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.HoldGravityTests()
        self.fixture.setUp()
        self.c = self.fixture.controller

    def test_profile_selection_is_local_confirmed_atomic_and_keeps_zero_limits(self):
        before = copy.deepcopy(self.c.config['joints'])
        with self.assertRaisesRegex(ValueError, '确认'):
            self.c.configure_load_compensation(enabled=True, profile='tracking_support')
        self.c.configure_load_compensation(enabled=True, confirmed=True, profile='tracking_support')
        self.assertEqual(self.fixture.link.sent, [])
        self.assertIsNone(self.c.motion_plan)
        self.assertEqual(self.c.config['joints'], before)
        self.assertEqual(self.c.snapshot()['joints']['shoulder']['next_motion_gains']['kp'], 12.)
        self.c.configure_load_compensation(enabled=False)
        self.assertEqual(self.c.snapshot()['joints']['shoulder']['next_motion_gains']['kp'], 3.)
        with self.assertRaises(ValueError):
            self.c.configure_load_compensation(enabled=True, confirmed=True, profile='unknown')

    def test_save_failure_does_not_partially_select_profile(self):
        before = copy.deepcopy(self.c.config)
        self.c.save_config_callback = lambda _: (_ for _ in ()).throw(OSError('disk full'))
        with self.assertRaises(OSError):
            self.c.configure_load_compensation(enabled=True, confirmed=True, profile='tracking_support')
        self.assertEqual(self.c.config, before)
        self.assertEqual(self.fixture.link.sent, [])

    def test_custom_parameters_preserve_other_settings_and_do_not_send(self):
        before = copy.deepcopy(self.c.config)
        saved = []
        self.c.save_config_callback = lambda cfg: saved.append(copy.deepcopy(cfg))
        parameters = {**SUPPORT_PRESET, 'position_kp': 10.25, 'velocity_kd': 1.25,
                      'ki_nm_per_rad_s': 1.8, 'maximum_bias_nm': 2.1}
        parameters.pop('enabled')
        self.c.configure_load_compensation(enabled=True, confirmed=True, parameters=parameters)
        self.assertEqual(self.c.config['load_compensation']['position_kp'], 10.25)
        self.assertEqual(motion_gains(self.c.config, 'shoulder'), (10.25, 1.25))
        self.assertEqual(self.c.config['gravity'], before['gravity'])
        self.assertEqual(self.c.config['joints'], before['joints'])
        self.assertEqual(self.c.config['motion_policy'], before['motion_policy'])
        self.assertEqual(saved[-1], self.c.config)
        self.assertEqual(self.fixture.link.sent, [])
        self.assertIsNone(self.c.motion_plan)
        # Switching off and back on must not silently restore the preset.
        self.c.configure_load_compensation(enabled=False)
        self.assertEqual(self.c.config['load_compensation']['maximum_bias_nm'], 2.1)
        self.c.configure_load_compensation(enabled=True, confirmed=True)
        self.assertEqual(motion_gains(self.c.config, 'shoulder'), (10.25, 1.25))

    def test_parameter_patch_validation_and_atomicity(self):
        before = copy.deepcopy(self.c.config)
        invalid = [[], {'enabled': True}, {'unknown': 1}, {'position_kp': 21},
                   {'velocity_kd': float('nan')}, {'maximum_bias_nm': 3},
                   {'ki_nm_per_rad_s': True}, {'strategy': 'automatic'},
                   {'maximum_rate_nm_s': 0}, {'tracking_error_rate_rad_s': .16}]
        for parameters in invalid:
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                self.c.configure_load_compensation(enabled=True, confirmed=True, parameters=parameters)
            self.assertEqual(self.c.config, before)
        with self.assertRaises(ValueError):
            self.c.configure_load_compensation(enabled=True, confirmed=True,
                profile='tracking_support', parameters={'position_kp': 8})
        self.c.save_config_callback = lambda _: (_ for _ in ()).throw(OSError('disk full'))
        with self.assertRaises(OSError):
            self.c.configure_load_compensation(enabled=False, parameters={'position_kp': 9})
        self.assertEqual(self.c.config, before)
        self.assertEqual(self.fixture.link.sent, [])

    def test_custom_parameters_rejected_during_motion_or_without_confirmation(self):
        with self.assertRaisesRegex(ValueError, '确认'):
            self.c.configure_load_compensation(enabled=True, parameters={'position_kp': 8})
        self.fixture.plan(5.)
        before = copy.deepcopy(self.c.config)
        with self.assertRaisesRegex(RuntimeError, '停止'):
            self.c.configure_load_compensation(enabled=False, parameters={'position_kp': 8})
        self.assertEqual(self.c.config, before)
        self.assertEqual(self.fixture.link.sent, [])

    def test_profile_gains_used_in_initial_enable_and_retarget(self):
        self.c.configure_load_compensation(enabled=True, confirmed=True, profile='tracking_support')
        with patch.object(self.c, '_wait_for_stationary'), patch.object(self.c, '_verify_control_parameters'), patch.object(self.c, '_wait_for_feedback_after'):
            self.c.start_motion({'elbow': 5., 'shoulder': 5.})
        plan = self.c.motion_plan
        self.assertEqual(plan.kp, {'elbow':12., 'shoulder':12.})
        self.assertEqual(plan.torque_limit, {'elbow':3.5, 'shoulder':3.5})
        for cid, payload in self.fixture.link.sent:
            if (cid >> 24) & 31 == CMD_MOTOR_CONTROL:
                _, _, kp, kd = struct.unpack('>HHHH', payload)
                self.assertAlmostEqual(uint_to_float(kp, 0, 500), 12., delta=.008)
                self.assertAlmostEqual(uint_to_float(kd, 0, 5), 1.5, delta=.0001)
        self.fixture.advance(plan.duration+1., angles={'elbow':5., 'shoulder':5.})
        for r in self.c.runtimes.values():
            r.load_bias_nm = .5
        before = len(self.fixture.link.sent)
        self.c.start_motion({'shoulder': 10.})
        self.assertEqual(self.c.motion_plan.kp, plan.kp)
        self.assertTrue(all(r.load_bias_nm == .5 for r in self.c.runtimes.values()))
        self.assertFalse(any((cid >> 24) & 31 in (CMD_ENABLE, CMD_PARAM_WRITE, CMD_STOP)
                             for cid, _ in self.fixture.link.sent[before:]))

    def test_sent_snapshot_is_quantized_and_not_double_counted_pd(self):
        self.fixture.plan(20.)
        self.fixture.advance(1., angles={'elbow':18., 'shoulder':18.})
        sent = self.c.snapshot()['joints']['shoulder']['sent_control']
        self.assertIsNotNone(sent)
        self.assertGreater(sent['pd_estimate_nm'], .1)
        # With compensation disabled Type1 torque is zero, NOT host PD.
        self.assertAlmostEqual(sent['feedforward_nm'], 0., delta=.0004)
        self.assertAlmostEqual(sent['kp'], 3., delta=.008)
        self.c.stop_all()
        self.assertIsNone(self.c.snapshot()['joints']['shoulder']['sent_control'])

    def test_encoder_slope_gate_rejects_gaps_and_push(self):
        r = self.c.runtimes['shoulder']
        now = self.fixture.clock()
        r.stationary_feedback.clear()
        self.assertIsNone(self.c._tracking_error_rate(r, 0., now))
        r.stationary_feedback.extend([(now-.1, 0., 0.), (now-.05, .01, 0.), (now, .02, 0.)])
        self.assertAlmostEqual(self.c._tracking_error_rate(r, .2, now), 0.)
        self.assertAlmostEqual(self.c._tracking_error_rate(r, 0., now), -.2)
        self.assertIsNone(self.c._tracking_error_rate(r, 0., now+.09))

    def test_zero_mean_oscillating_encoder_does_not_learn(self):
        r = self.c.runtimes['shoulder']
        now = self.fixture.clock()
        r.stationary_feedback.clear()
        r.stationary_feedback.extend([(now-.1, 0., 0.), (now-.05, .01, 0.), (now, 0., 0.)])
        self.assertIsNone(self.c._tracking_error_rate(r, 0., now))

    def run_plant(self, support, *, load_amplitude=2.8, push=False, pd_kp=None, retarget=False):
        # Synthetic 2-link decoupled plant: inertia + viscous loss + sin load.
        # Motor response uses decoded wire fields, including quantization.
        f, c = self.fixture, self.c
        if support:
            c.configure_load_compensation(enabled=True, confirmed=True, profile='tracking_support')
        gains = {k: motion_gains(c.config, k) for k in ('elbow', 'shoulder')}
        if pd_kp is not None:
            gains = {k: (pd_kp, 1.5) for k in gains}
        c.motion_plan = MotionPlan(1, f.clock(), 6., ('elbow','shoulder'),
            dict.fromkeys(gains, 0.), dict.fromkeys(gains, 30.), dict.fromkeys(gains, 3.5),
            {k:v[0] for k,v in gains.items()}, {k:v[1] for k,v in gains.items()})
        c.last_control_at = f.clock()
        for r in c.runtimes.values():
            r.target_deg = 30.
            r.phase = 'moving'
        q = dict.fromkeys(gains, 0.)
        velocity = dict.fromkeys(gains, 0.)
        torque = dict.fromkeys(gains, 0.)
        max_bias = max_torque = 0.
        start = f.clock()
        for step in range(6500 if retarget else 3500):
            f.clock.now += .01
            for key, cfg in c.config['joints'].items():
                c.feed_line(fixtures.feedback_response(cfg['motor_can_id'], position_rad=q[key],
                    velocity_rad_s=velocity[key], torque_nm=torque[key], mode_status=2))
                r = c.runtimes[key]
                r.parameter_values.update({0x7005:0, 0x700B:3.5})
                r.parameter_times.update({0x7005:f.clock(), 0x700B:f.clock()})
            if retarget and step == 3500:
                old_bias = c.runtimes['shoulder'].load_bias_nm
                c.start_motion({'shoulder': 5.})
                self.assertEqual(c.runtimes['shoulder'].load_bias_nm, old_bias)
            c.control_tick()
            max_bias = max(max_bias, *(abs(r.load_bias_nm) for r in c.runtimes.values()))
            if c.safety_latched:
                break
            for key in gains:
                sent = c.runtimes[key].last_sent_control
                requested = (sent['feedforward_nm']+sent['kp']*(sent['position_rad']-q[key])
                             +sent['kd']*(sent['velocity_rad_s']-velocity[key]))
                torque[key] = max(-3.5, min(3.5, requested))
                max_torque = max(max_torque, abs(torque[key]))
                load = (load_amplitude if key == 'shoulder' else .5)*math.sin(q[key])
                disturbance = -.3 if push and key == 'shoulder' and 22 < f.clock()-start < 23 else 0.
                acceleration = (torque[key]-load-.2*velocity[key]+disturbance)/.08
                velocity[key] += acceleration*.01
                q[key] += velocity[key]*.01
        return c.snapshot(), max_bias, max_torque

    def test_dynamic_load_arrives_and_holds_instead_of_pd_static_offset(self):
        state, bias, torque = self.run_plant(True)
        self.assertFalse(state['safety_latched'], state['safety_reason'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertLess(abs(state['joints']['shoulder']['error_deg']), .6)
        self.assertGreater(state['joints']['shoulder']['load_bias_nm'], 1.)
        self.assertLessEqual(bias, 2.5)
        self.assertLessEqual(torque, 3.5)

    def test_dynamic_load_without_support_still_exposes_original_problem(self):
        state, _, _ = self.run_plant(False)
        self.assertFalse(state['safety_latched'], state['safety_reason'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertFalse(state['joints']['shoulder']['on_target'])
        self.assertGreater(abs(state['joints']['shoulder']['error_deg']), 1.5)

    def test_higher_kp_alone_does_not_remove_static_load_error(self):
        state, _, _ = self.run_plant(False, pd_kp=12.)
        self.assertFalse(state['safety_latched'], state['safety_reason'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertFalse(state['joints']['shoulder']['on_target'])
        self.assertGreater(abs(state['joints']['shoulder']['error_deg']), 1.5)

    def test_small_external_load_is_rejected_then_original_target_recovers(self):
        state, _, _ = self.run_plant(True, push=True)
        self.assertFalse(state['safety_latched'], state['safety_reason'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertLess(abs(state['joints']['shoulder']['error_deg']), .6)

    def test_unreachable_load_still_stops_and_clears_bias(self):
        state, bias, torque = self.run_plant(True, load_amplitude=30.)
        self.assertTrue(state['safety_latched'])
        self.assertEqual(state['control_phase'], 'stopped')
        self.assertTrue(all(j['load_bias_nm'] == 0. for j in state['joints'].values()))
        self.assertLessEqual(bias, 2.5)
        self.assertLessEqual(torque, 3.5)

    def test_loaded_descent_keeps_support_then_unloads_for_new_target(self):
        state, _, torque = self.run_plant(True, retarget=True)
        self.assertFalse(state['safety_latched'], state['safety_reason'])
        self.assertEqual(state['control_phase'], 'controlling')
        self.assertEqual(state['joints']['shoulder']['target_deg'], 5.)
        self.assertEqual(state['joints']['elbow']['target_deg'], 30.)
        self.assertLess(abs(state['joints']['shoulder']['error_deg']), .6)
        self.assertLess(state['joints']['shoulder']['load_bias_nm'], .4)
        self.assertLessEqual(torque, 3.5)


if __name__ == '__main__':
    unittest.main()
