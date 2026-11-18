"""Offline arithmetic checks; no serial port or motor is opened."""
import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from classic_dual_controller import DEFAULT_CONFIG, ClassicConfigStore
from motion_policy import DEFAULT_LOAD_COMPENSATION, residual_bias


class ResidualCompensationTests(unittest.TestCase):
    def setUp(self):
        self.settings = {**DEFAULT_LOAD_COMPENSATION, 'enabled': True}

    def update(self, bias=0., error_deg=2., base=0., limit=3.5, dt=.05, stationary=True, complete=True):
        return residual_bias(self.settings, bias, math.radians(error_deg), base, limit, dt,
                             stationary=stationary, trajectory_complete=complete)

    def test_disabled_is_exactly_zero(self):
        self.settings['enabled'] = False
        self.assertEqual(self.update(bias=.4), 0.)

    def test_sign_deadband_and_stationary_gate(self):
        self.assertGreater(self.update(), 0.)
        self.assertLess(self.update(error_deg=-2.), 0.)
        self.assertEqual(self.update(bias=.1, error_deg=.1), .1)
        self.assertEqual(self.update(bias=.1, stationary=False), .1)
        self.assertEqual(self.update(bias=.1, error_deg=6.), .1)

    def test_rate_limit_and_dt_cap(self):
        self.settings['ki_nm_per_rad_s'] = 2.
        self.assertAlmostEqual(self.update(error_deg=5., dt=10.), .005)
        self.assertEqual(self.update(dt=0.), 0.)
        self.assertEqual(self.update(dt=-1.), 0.)

    def test_bias_cap_in_both_directions(self):
        bias = 0.
        for _ in range(10000):
            bias = self.update(bias=bias, error_deg=5.)
        self.assertAlmostEqual(bias, .5)
        for _ in range(20000):
            bias = self.update(bias=bias, error_deg=-5.)
        self.assertAlmostEqual(bias, -.5)

    def test_no_windup_into_either_torque_limit_and_can_unload(self):
        self.assertEqual(self.update(bias=.1, base=3.4), .1)
        self.assertEqual(self.update(bias=-.1, base=-3.4, error_deg=-2.), -.1)
        self.assertLess(self.update(bias=.1, base=3.5, error_deg=-2.), .1)
        self.assertGreater(self.update(bias=-.1, base=-3.5, error_deg=2.), -.1)

    def test_new_trajectory_decays_without_zero_crossing(self):
        self.assertAlmostEqual(self.update(bias=.1, complete=False), .095)
        self.assertAlmostEqual(self.update(bias=-.1, complete=False), -.095)
        self.assertEqual(self.update(bias=.001, complete=False), 0.)

    def test_invalid_switch_unknown_fields_and_unbounded_parameters_rejected(self):
        for field, value in (('enabled', 'yes'), ('maximum_bias_nm', 5.),
                             ('maximum_rate_nm_s', 1.), ('ki_nm_per_rad_s', float('nan')),
                             ('maximum_error_deg', 20.), ('unexpected', 1)):
            with self.subTest(field=field):
                cfg = copy.deepcopy(DEFAULT_CONFIG)
                cfg['load_compensation'][field] = value
                with self.assertRaises(ValueError):
                    ClassicConfigStore.validate(cfg)

    def test_old_configuration_without_load_section_loads_disabled(self):
        import json
        import tempfile
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        del cfg['load_compensation']
        # Existing stricter tolerance stays valid while compensation is off.
        cfg['motion_policy']['arrival_tolerance_deg'] = .1
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'legacy.json'
            path.write_text(json.dumps(cfg), encoding='utf-8')
            loaded = ClassicConfigStore(path).load()
        self.assertFalse(loaded['load_compensation']['enabled'])
        self.assertEqual(loaded['joints'], cfg['joints'])

    def test_v22_enabled_configuration_keeps_legacy_strategy_and_gains(self):
        import json
        import tempfile
        from motion_policy import motion_gains
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        cfg['load_compensation'] = {'enabled':True, 'ki_nm_per_rad_s':.6,
            'maximum_bias_nm':.5, 'maximum_rate_nm_s':.1, 'deadband_deg':.2, 'maximum_error_deg':5.}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'v22.json'
            path.write_text(json.dumps(cfg), encoding='utf-8')
            loaded = ClassicConfigStore(path).load()
        self.assertEqual(loaded['load_compensation']['strategy'], 'stationary_residual')
        self.assertEqual(motion_gains(loaded, 'shoulder'), (3., 1.))
        self.assertEqual(loaded['load_compensation']['maximum_bias_nm'], .5)


if __name__ == '__main__':
    unittest.main()
