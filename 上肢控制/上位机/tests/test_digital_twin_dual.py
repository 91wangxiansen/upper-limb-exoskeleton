from __future__ import annotations

import copy
import math
import queue
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch


APP_ROOT = Path(__file__).resolve().parents[1]
if str(APP_ROOT) not in sys.path:
    sys.path.insert(0, str(APP_ROOT))

from classic_dual_controller import ClassicConfigStore, DEFAULT_CONFIG  # noqa: E402
from cybergear_protocol import (  # noqa: E402
    CMD_ENABLE,
    CMD_GET_DEVICE_ID,
    CMD_MOTOR_CONTROL,
    CMD_STOP,
    HOST_ID_DEFAULT,
    encode_slcan_extended,
    float_to_uint,
    make_set_can_id,
    CMD_PARAM_READ, CMD_PARAM_WRITE, payload_param,
)
from digital_twin_server import CyberArmTwin, SlcanLink, main  # noqa: E402


class SlcanConfigurationTests(unittest.TestCase):
    def test_classic_transport_uses_same_retry_without_motion(self):
        from upper_computer import SlcanTransport
        with patch('upper_computer.serial.Serial') as serial_class, patch('upper_computer.threading.Thread'), patch('upper_computer.time.sleep'):
            link = SlcanTransport('OFFLINE')
            link.connect()
            writes = [call.args[0] for call in serial_class.return_value.write.call_args_list]
            self.assertEqual(writes, [b'C\r', b'S8\r', b'M0\r', b'A1\r', b'O\r'])
            link.close()

    def test_connection_enables_can_retry_before_opening_without_motion(self):
        with patch('digital_twin_server.serial.Serial') as serial_class, patch('digital_twin_server.threading.Thread'):
            link = SlcanLink('OFFLINE')
            link.connect()
            writes = [call.args[0] for call in serial_class.return_value.write.call_args_list]
            self.assertEqual(writes, [b'C\r', b'S8\r', b'M0\r', b'A1\r', b'O\r'])
            link.close()

    def test_ui_snapshot_does_not_hold_log_lock_while_entering_controller(self):
        with tempfile.TemporaryDirectory() as folder:
            twin = CyberArmTwin(Path(folder) / 'config.json')
            def check_lock():
                self.assertFalse(twin.lock._is_owned())
                return None
            with patch.object(twin, 'controller') as controller:
                controller.snapshot.side_effect = check_lock
                twin.snapshot()


class FakeSlcanLink:
    instances: list[FakeSlcanLink] = []

    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.frames: list[tuple[int, bytes]] = []
        self.positions = {126: 0.0, 127: 0.0}
        self.uids = {126: "3A4B3B8A2093B015", 127: "034C3B8A2093B015"}
        self.mode = {126: 0, 127: 0}
        self.parameters = {126: {0x7005: 0, 0x700B: 3.5, 0x7018: 6.5, 0x701A: 0.0, 0x701C: 24.0},
                           127: {0x7005: 0, 0x700B: 3.5, 0x7018: 6.5, 0x701A: 0.0, 0x701C: 24.0}}
        self.instances.append(self)

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def feedback(self, motor_id: int) -> None:
        status = (self.mode[motor_id] << 14) | motor_id
        can_id = (2 << 24) | (status << 8) | HOST_ID_DEFAULT
        payload = struct.pack(
            ">HHHH",
            float_to_uint(self.positions[motor_id], -4 * math.pi, 4 * math.pi),
            float_to_uint(0.0, -30.0, 30.0),
            float_to_uint(0.0, -12.0, 12.0),
            250,
        )
        self.rx_queue.put(encode_slcan_extended(can_id, payload).decode().strip())

    def send(self, can_id: int, payload: bytes) -> None:
        if not self.connected:
            raise RuntimeError("fake link disconnected")
        self.frames.append((can_id, payload))
        command, motor_id = (can_id >> 24) & 0x1F, can_id & 0xFF
        if command == CMD_GET_DEVICE_ID:
            response_id = (CMD_GET_DEVICE_ID << 24) | (motor_id << 8) | HOST_ID_DEFAULT
            self.rx_queue.put(
                encode_slcan_extended(response_id, bytes.fromhex(self.uids[motor_id])).decode().strip()
            )
        elif command == CMD_STOP:
            self.mode[motor_id] = 0
            self.feedback(motor_id)
        elif command == CMD_ENABLE:
            self.mode[motor_id] = 2
            self.feedback(motor_id)
        elif command == CMD_MOTOR_CONTROL:
            self.feedback(motor_id)
        elif command == CMD_PARAM_WRITE:
            index = struct.unpack("<H", payload[:2])[0]
            self.parameters[motor_id][index] = payload[4] if index == 0x7005 else struct.unpack("<f", payload[4:])[0]
        elif command == CMD_PARAM_READ:
            index = struct.unpack("<H", payload[:2])[0]
            response = (CMD_PARAM_READ << 24) | (motor_id << 8) | HOST_ID_DEFAULT
            value = self.parameters[motor_id].get(index)
            if value is not None:
                self.rx_queue.put(encode_slcan_extended(response, payload_param(index, value, "u8" if index == 0x7005 else "float")).decode().strip())


class DigitalTwinDualTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config_path = Path(self.temp.name) / "classic_dual_config.json"
        ClassicConfigStore(self.config_path).save(copy.deepcopy(DEFAULT_CONFIG))
        FakeSlcanLink.instances.clear()
        self.twin = CyberArmTwin(self.config_path, FakeSlcanLink)
        self.addCleanup(self.twin.shutdown)

    def test_offline_snapshot_has_two_fixed_roles_and_no_motion(self) -> None:
        state = self.twin.snapshot()
        self.assertFalse(state["connected"])
        self.assertEqual(set(state["joints"]), {"elbow", "shoulder"})
        self.assertEqual(state["joints"]["elbow"]["motor_can_id"], 127)
        self.assertEqual(state["joints"]["shoulder"]["motor_can_id"], 126)
        with self.assertRaisesRegex(RuntimeError, "CANable"):
            self.twin.move({"targets_deg": {"elbow": 20.0}})

    def test_load_compensation_schema_persistence_and_no_automatic_motion(self) -> None:
        state = self.twin.snapshot()
        self.assertEqual(state['version'], '2.3.3')
        self.assertFalse(state['load_compensation']['enabled'])
        self.assertTrue(all(j['load_bias_nm'] == 0. and j['notice'] == '' for j in state['joints'].values()))
        with self.assertRaisesRegex(RuntimeError, 'CANable'):
            self.twin.configure_load_compensation({'enabled':True, 'confirmed':True})
        self.twin.connect('FAKE')
        link = FakeSlcanLink.instances[-1]
        link.frames.clear()
        for body in ({'enabled':True}, {'enabled':'yes', 'confirmed':True}, {'enabled':True, 'ki':99}):
            with self.assertRaises(ValueError):
                self.twin.configure_load_compensation(body)
        self.twin.configure_load_compensation({'enabled':True, 'confirmed':True})
        self.assertTrue(ClassicConfigStore(self.config_path).load()['load_compensation']['enabled'])
        self.assertFalse(self.twin.snapshot()['motion_active'])
        self.assertFalse(any((cid >> 24) & 31 in (CMD_ENABLE, CMD_MOTOR_CONTROL, CMD_PARAM_WRITE) for cid, _ in link.frames))
        self.twin.configure_load_compensation({'enabled':False})
        self.assertFalse(ClassicConfigStore(self.config_path).load()['load_compensation']['enabled'])

    def test_connect_verifies_both_uids_and_stop_addresses_both(self) -> None:
        result = self.twin.connect("FAKE")
        self.assertEqual(result, {"elbow": True, "shoulder": True})
        self.assertTrue(all(j["verified"] for j in self.twin.snapshot()["joints"].values()))
        link = FakeSlcanLink.instances[-1]
        link.frames.clear()
        self.assertEqual(self.twin.emergency_stop(), {"elbow": "sent", "shoulder": "sent"})
        stop_ids = [can_id & 0xFF for can_id, _ in link.frames if (can_id >> 24) & 0x1F == CMD_STOP]
        self.assertEqual(stop_ids, [127, 126])

    def test_custom_motion_parameters_api_persists_without_resetting_gravity(self) -> None:
        self.twin.connect('FAKE')
        link = FakeSlcanLink.instances[-1]
        before = ClassicConfigStore(self.config_path).load()
        for parameters in (None, [], {'enabled': True}, {'position_kp': 500}):
            with self.assertRaises(ValueError):
                self.twin.configure_load_compensation({'enabled': True, 'confirmed': True,
                                                      'parameters': parameters})
        self.twin.configure_load_compensation({'enabled': True, 'confirmed': True,
            'parameters': {'strategy': 'tracking_support', 'position_kp': 9.5,
                           'velocity_kd': 1.3, 'ki_nm_per_rad_s': 1.7}})
        saved = ClassicConfigStore(self.config_path).load()
        self.assertEqual(saved['load_compensation']['position_kp'], 9.5)
        self.assertEqual(saved['load_compensation']['ki_nm_per_rad_s'], 1.7)
        self.assertEqual(saved['load_compensation']['maximum_bias_nm'], before['load_compensation']['maximum_bias_nm'])
        self.assertEqual(saved['gravity'], before['gravity'])
        self.assertEqual(saved['joints'], before['joints'])
        self.assertFalse(self.twin.snapshot()['motion_active'])
        self.assertFalse(any((cid >> 24) & 31 in (CMD_ENABLE, CMD_MOTOR_CONTROL, CMD_PARAM_WRITE)
                             for cid, _ in link.frames))

    def test_zero_commission_and_coordinated_motion(self) -> None:
        self.twin.connect("FAKE")
        for key in ("elbow", "shoulder"):
            self.twin.capture_zero(key)
            cfg = self.twin.config["joints"][key]
            self.twin.configure_joint(key, {
                "direction": cfg["direction"],
                "minimum_angle_deg": cfg["minimum_angle_deg"],
                "maximum_angle_deg": cfg["maximum_angle_deg"],
                "maximum_speed_rad_s": cfg["maximum_speed_rad_s"],
                "maximum_torque_nm": cfg["maximum_torque_nm"],
                "motion_kp": cfg["motion_kp"],
                "motion_kd": cfg["motion_kd"],
                "limits_confirmed": True,
            })
            self.twin.set_commissioned(key, True)
        link = FakeSlcanLink.instances[-1]
        link.frames.clear()
        duration = self.twin.move({
            "targets_deg": {"elbow": 10.0, "shoulder": 5.0},
            "speeds_rad_s": {"elbow": 0.05, "shoulder": 0.05},
            "torque_limits_nm": {"elbow": 0.3, "shoulder": 0.3},
        })
        self.assertGreater(duration, 0.4)
        self.assertTrue(self.twin.snapshot()["motion_active"])
        enable_ids = [can_id & 0xFF for can_id, _ in link.frames if (can_id >> 24) & 0x1F == CMD_ENABLE]
        self.assertEqual(enable_ids, [127, 126])
        self.assertEqual(set(self.twin.history_snapshot()), {"elbow", "shoulder"})
        self.twin.emergency_stop()
        self.assertFalse(self.twin.snapshot()["motion_active"])

    def test_uncommissioned_motor_blocks_motion(self) -> None:
        self.twin.connect("FAKE")
        with self.assertRaisesRegex(RuntimeError, "\u5c1a\u672a\u5b8c\u6210"):
            self.twin.move({"targets_deg": {"elbow": 20.0}})

    def test_single_confirmation_prepares_both_and_idle_feedback_can_restart(self) -> None:
        self.twin.connect("FAKE")
        for key in ("elbow", "shoulder"):
            self.twin.capture_zero(key)
        configs = {
            key: {name: self.twin.config["joints"][key][name] for name in (
                "direction", "minimum_angle_deg", "maximum_angle_deg",
                "maximum_speed_rad_s", "maximum_torque_nm", "motion_kp", "motion_kd",
            )}
            for key in ("elbow", "shoulder")
        }
        with self.assertRaisesRegex(ValueError, "核对"):
            self.twin.prepare_joints({"configs": configs})
        self.assertFalse(self.twin.config["joints"]["elbow"]["commissioned"])
        self.twin.prepare_joints({"configs": configs, "confirmed": True})
        self.assertTrue(all(j["commissioned"] and j["limits_confirmed"] for j in self.twin.snapshot()["joints"].values()))
        for runtime in self.twin.controller.runtimes.values():
            runtime.last_feedback_at = time.monotonic() - 10
        self.assertGreater(self.twin.move({"targets_deg": {"shoulder": 2.0}}), 0)
        self.assertTrue(self.twin.snapshot()["motion_active"])

    def test_confirmation_without_zero_does_not_enable_motor(self) -> None:
        self.twin.connect("FAKE")
        cfg = self.twin.config["joints"]["elbow"]
        settings = {name: cfg[name] for name in (
            "direction", "minimum_angle_deg", "maximum_angle_deg",
            "maximum_speed_rad_s", "maximum_torque_nm", "motion_kp", "motion_kd",
        )}
        link = FakeSlcanLink.instances[-1]
        link.frames.clear()
        with self.assertRaisesRegex(RuntimeError, "零点"):
            self.twin.prepare_joints({"configs": {"elbow": settings}, "confirmed": True})
        self.assertFalse(any((can_id >> 24) & 0x1F == CMD_ENABLE for can_id, _ in link.frames))

    def test_elbow_zero_target_is_within_updated_soft_limit(self) -> None:
        self.twin.connect("FAKE")
        for key in ("elbow", "shoulder"):
            self.twin.capture_zero(key)
        cfg = self.twin.config["joints"]["elbow"]
        settings = {name: cfg[name] for name in (
            "direction", "minimum_angle_deg", "maximum_angle_deg",
            "maximum_speed_rad_s", "maximum_torque_nm", "motion_kp", "motion_kd",
        )}
        self.twin.prepare_joints({"configs": {"elbow": settings}, "confirmed": True})
        self.assertEqual(self.twin.snapshot()["joints"]["elbow"]["limits"]["minimum_angle_deg"], 0.0)
        self.assertGreaterEqual(self.twin.move({"targets_deg": {"elbow": 0.0}}), 0.4)

    def test_type_seven_can_id_encoder_keeps_manual_layout(self) -> None:
        can_id, payload = make_set_can_id(0, 126, HOST_ID_DEFAULT)
        self.assertEqual(can_id, 0x077EFD00)
        self.assertEqual(payload, bytes(8))

    def prepare_test_joints(self):
        self.twin.connect('FAKE')
        for key in ('elbow','shoulder'):
            self.twin.capture_zero(key)
        settings = {key: {name: self.twin.config['joints'][key][name] for name in (
            'direction','minimum_angle_deg','maximum_angle_deg','maximum_speed_rad_s',
            'maximum_torque_nm','motion_kp','motion_kd')} for key in ('elbow','shoulder')}
        self.twin.prepare_joints({'configs':settings,'confirmed':True})
        return FakeSlcanLink.instances[-1]

    def test_support_profile_api_is_persisted_without_enabling_motors(self):
        link = self.prepare_test_joints()
        before = {k:dict(v) for k,v in self.twin.config['joints'].items()}
        link.frames.clear()
        self.twin.configure_load_compensation({'enabled':True, 'confirmed':True, 'profile':'tracking_support'})
        state = self.twin.snapshot()
        saved = ClassicConfigStore(self.config_path).load()
        self.assertEqual(saved['load_compensation']['strategy'], 'tracking_support')
        self.assertEqual(saved['load_compensation']['position_kp'], 12.)
        self.assertEqual(saved['joints'], before)
        self.assertEqual(state['joints']['shoulder']['next_motion_gains']['kp'], 12.)
        self.assertFalse(state['motion_active'])
        self.assertFalse(any((cid >> 24) & 31 in (CMD_ENABLE, CMD_MOTOR_CONTROL, CMD_PARAM_WRITE) for cid,_ in link.frames))

    def test_second_motor_readback_mismatch_prevents_either_enable(self):
        link = self.prepare_test_joints()
        original = link.send
        def bad_read(can_id, payload):
            if (can_id>>24)&31 == CMD_PARAM_READ and can_id&255 == 126 and struct.unpack('<H',payload[:2])[0] == 0x700B:
                link.parameters[126][0x700B] = 8.0
            original(can_id,payload)
        with patch.object(link,'send',side_effect=bad_read):
            link.frames.clear()
            with self.assertRaisesRegex(RuntimeError,'读回不一致'):
                self.twin.move({'targets_deg':{'elbow':5.0,'shoulder':5.0}})
        self.assertFalse(any((cid>>24)&31 == CMD_ENABLE for cid,_ in link.frames))

    def test_no_parameter_reply_prevents_enable(self):
        link = self.prepare_test_joints()
        original = link.send
        def no_reply(can_id,payload):
            if (can_id>>24)&31 == CMD_PARAM_READ:
                return
            original(can_id,payload)
        with patch.object(link,'send',side_effect=no_reply):
            link.frames.clear()
            with self.assertRaisesRegex(RuntimeError,'读回超时'):
                self.twin.move({'targets_deg':{'elbow':5.0}})
        self.assertFalse(any((cid>>24)&31 == CMD_ENABLE for cid,_ in link.frames))

    def test_calibration_is_local_only_and_rezero_invalidates_it(self):
        link = self.prepare_test_joints()
        link.frames.clear()
        self.twin.configure_gravity({'enabled':True,'confirmed':True,'shoulder_reference_deg':-90.0,
            'forearm_reference_deg':-90.0,'shoulder_angle_sign':1,'elbow_angle_sign':1})
        snapshot = self.twin.snapshot()
        self.assertTrue(snapshot['gravity']['enabled'])
        self.assertTrue(snapshot['gravity']['calibration_valid'])
        self.assertFalse(snapshot['motion_active'])
        self.assertFalse(any((cid>>24)&31 in (CMD_ENABLE,CMD_MOTOR_CONTROL) for cid,_ in link.frames))
        with self.assertRaisesRegex(RuntimeError,'双关节运动'):
            self.twin.move({'targets_deg':{'elbow':5.0}})
        self.twin.capture_zero('shoulder')
        self.assertFalse(self.twin.snapshot()['gravity']['enabled'])

    def test_changed_direction_disables_calibration_when_preparing(self):
        self.prepare_test_joints()
        self.twin.configure_gravity({'enabled':True,'confirmed':True,'shoulder_reference_deg':-90.0,'forearm_reference_deg':-90.0})
        cfg = self.twin.config['joints']['shoulder']
        settings = {name:cfg[name] for name in ('direction','minimum_angle_deg','maximum_angle_deg',
            'maximum_speed_rad_s','maximum_torque_nm','motion_kp','motion_kd')}
        settings['direction'] = -1
        self.twin.prepare_joints({'confirmed':True,'configs':{'shoulder':settings}})
        self.assertFalse(self.twin.snapshot()['gravity']['enabled'])
        self.assertFalse(self.twin.snapshot()['gravity']['calibration_valid'])

    def test_calibration_requires_explicit_physical_confirmation(self):
        self.twin.connect('FAKE')
        with self.assertRaisesRegex(ValueError,'确认'):
            self.twin.configure_gravity({'enabled':True})

    def test_confirmed_vertical_zero_has_exact_offsets_without_rezero_or_enable(self):
        link = self.prepare_test_joints()
        before = {k: self.twin.config['joints'][k]['software_zero_rad'] for k in ('elbow','shoulder')}
        link.frames.clear()
        self.twin.configure_gravity({'enabled': True, 'confirmed': True, 'vertical_zero_confirmed': True})
        gravity = self.twin.snapshot()['gravity']
        self.assertEqual(gravity['shoulder_offset_deg'], -90.)
        self.assertEqual(gravity['elbow_offset_deg'], 0.)
        self.assertTrue(gravity['calibration_valid'])
        self.assertEqual(before, {k: self.twin.config['joints'][k]['software_zero_rad'] for k in before})
        self.assertTrue(all((cid >> 24) & 31 in (CMD_STOP, CMD_PARAM_READ) for cid, _ in link.frames))
        saved = ClassicConfigStore(self.config_path).load()['gravity']
        self.assertTrue(saved['enabled'])
        self.assertEqual(saved['shoulder_offset_deg'], -90.)

    def test_vertical_zero_does_not_imply_physical_confirmation(self):
        self.twin.connect('FAKE')
        with self.assertRaisesRegex(ValueError, '确认'):
            self.twin.configure_gravity({'enabled': True, 'vertical_zero_confirmed': True})

    def test_conflicting_vertical_zero_and_current_angles_are_rejected(self):
        self.prepare_test_joints()
        with self.assertRaisesRegex(ValueError, '不可混用'):
            self.twin.configure_gravity({'enabled': True, 'confirmed': True, 'vertical_zero_confirmed': True,
                                        'shoulder_reference_deg': -90., 'forearm_reference_deg': -90.})


if __name__ == "__main__":
    unittest.main()
