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


APP_ROOT = Path(__file__).resolve().parents[1]
if str(APP_ROOT) not in sys.path:
    sys.path.insert(0, str(APP_ROOT))

from classic_dual_controller import (  # noqa: E402
    DEFAULT_CONFIG,
    ClassicConfigStore,
    DualMotorController,
    coordinated_duration,
)
from cybergear_protocol import (  # noqa: E402
    CMD_FEEDBACK,
    CMD_GET_DEVICE_ID,
    CMD_STOP,
    HOST_ID_DEFAULT,
    encode_slcan_extended,
    float_to_uint,
)


class FakeTransport:
    def __init__(self) -> None:
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.attempted_ids: list[int] = []
        self.sent: list[tuple[int, bytes]] = []
        self.fail_motor_ids: set[int] = set()

    def send(self, can_id: int, payload: bytes) -> None:
        motor_id = can_id & 0xFF
        self.attempted_ids.append(motor_id)
        if motor_id in self.fail_motor_ids:
            raise RuntimeError(f"injected send failure for ID {motor_id}")
        self.sent.append((can_id, payload))


def uid_response(motor_id: int, uid_hex: str) -> str:
    can_id = (CMD_GET_DEVICE_ID << 24) | (motor_id << 8) | HOST_ID_DEFAULT
    return encode_slcan_extended(can_id, bytes.fromhex(uid_hex)).decode("ascii").strip()


def feedback_response(
    motor_id: int,
    *,
    position_rad: float = 0.0,
    velocity_rad_s: float = 0.0,
    torque_nm: float = 0.0,
    temperature_c: float = 25.0,
    mode_status: int = 0,
    fault_bits: int = 0,
) -> str:
    status = ((mode_status & 0x03) << 14) | ((fault_bits & 0x3F) << 8) | motor_id
    can_id = (CMD_FEEDBACK << 24) | (status << 8) | HOST_ID_DEFAULT
    payload = struct.pack(
        ">HHHH",
        float_to_uint(position_rad, -4.0 * math.pi, 4.0 * math.pi),
        float_to_uint(velocity_rad_s, -30.0, 30.0),
        float_to_uint(torque_nm, -12.0, 12.0),
        round(temperature_c * 10.0),
    )
    return encode_slcan_extended(can_id, payload).decode("ascii").strip()


class ClassicDualControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.transport = FakeTransport()
        self.controller = DualMotorController(self.transport, self.config)

    def verify_both_identities(self) -> None:
        for key in ("elbow", "shoulder"):
            cfg = self.config["joints"][key]
            self.controller.feed_line(
                uid_response(int(cfg["motor_can_id"]), str(cfg["expected_uid"]))
            )

    def test_default_identity_map_is_unique_and_uncommissioned(self) -> None:
        joints = self.config["joints"]
        self.assertEqual(joints["elbow"]["motor_can_id"], 127)
        self.assertEqual(joints["elbow"]["minimum_angle_deg"], 0.0)
        self.assertEqual(joints["shoulder"]["motor_can_id"], 126)
        self.assertEqual(len({joints[key]["expected_uid"] for key in joints}), 2)
        for item in joints.values():
            self.assertIsNone(item["software_zero_rad"])
            self.assertFalse(item["commissioned"])
            self.assertFalse(item["limits_confirmed"])

    def test_uid_responses_are_routed_to_fixed_joint_roles(self) -> None:
        self.verify_both_identities()
        snapshot = self.controller.snapshot()
        self.assertTrue(snapshot["joints"]["elbow"]["verified"])
        self.assertTrue(snapshot["joints"]["shoulder"]["verified"])
        self.assertEqual(snapshot["joints"]["elbow"]["observed_uid"], "034C3B8A2093B015")
        self.assertEqual(snapshot["joints"]["shoulder"]["observed_uid"], "3A4B3B8A2093B015")

    def test_wrong_uid_latches_safety_and_stops_both(self) -> None:
        self.controller.feed_line(uid_response(127, "0000000000000000"))
        snapshot = self.controller.snapshot()
        self.assertTrue(snapshot["safety_latched"])
        self.assertIn("UID", snapshot["safety_reason"])
        stop_ids = [
            can_id & 0xFF
            for can_id, _payload in self.transport.sent
            if ((can_id >> 24) & 0x1F) == CMD_STOP
        ]
        self.assertEqual(stop_ids, [127, 126])

    def test_stop_all_attempts_second_motor_after_first_send_failure(self) -> None:
        self.transport.fail_motor_ids.add(127)
        result = self.controller.stop_all("test")
        self.assertEqual(self.transport.attempted_ids, [127, 126])
        self.assertTrue(result["elbow"].startswith("error:"))
        self.assertEqual(result["shoulder"], "sent")

    def test_uncommissioned_joint_cannot_start_motion(self) -> None:
        self.verify_both_identities()
        with self.assertRaisesRegex(RuntimeError, "尚未完成"):
            self.controller.start_motion({"elbow": 20.0})
        self.assertEqual(self.transport.sent, [])

    def test_motion_refuses_to_enable_if_either_stop_frame_fails(self) -> None:
        self.verify_both_identities()
        for key in ("elbow", "shoulder"):
            self.config["joints"][key]["software_zero_rad"] = 0.0
            self.config["joints"][key]["limits_confirmed"] = True
            self.config["joints"][key]["commissioned"] = True
            self.controller.runtimes[key].last_feedback_at = time.monotonic()
        self.transport.fail_motor_ids.add(127)
        with self.assertRaisesRegex(RuntimeError, "停止未全部发送"):
            self.controller.start_motion({"elbow": 20.0})
        self.assertFalse(any((can_id >> 24) & 0x1F == 3 for can_id, _ in self.transport.sent))

    def test_feedback_uses_independent_zero_and_direction(self) -> None:
        elbow = self.config["joints"]["elbow"]
        elbow["software_zero_rad"] = 0.25
        elbow["direction"] = -1
        self.controller.feed_line(feedback_response(127, position_rad=0.15))
        actual = self.controller.snapshot()["joints"]["elbow"]["actual_deg"]
        # Type-2 position is quantized to a 16-bit field over an 8π range.
        self.assertAlmostEqual(actual, math.degrees(0.10), delta=0.03)

    def test_coordinated_duration_obeys_each_speed_and_acceleration_limit(self) -> None:
        starts = {"elbow": 0.0, "shoulder": 10.0}
        targets = {"elbow": 30.0, "shoulder": -20.0}
        speeds = {"elbow": 0.30, "shoulder": 0.15}
        accelerations = {"elbow": 0.20, "shoulder": 0.10}
        duration = coordinated_duration(starts, targets, speeds, accelerations)
        for key in targets:
            distance = abs(math.radians(targets[key] - starts[key]))
            self.assertGreaterEqual(duration, 1.875 * distance / speeds[key])
            self.assertGreaterEqual(
                duration,
                math.sqrt(5.7735 * distance / accelerations[key]),
            )

    def test_config_store_round_trip_and_rejects_duplicate_id(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "classic_dual_config.json"
            store = ClassicConfigStore(path)
            store.save(self.config)
            self.assertEqual(store.load(), self.config)

        bad = copy.deepcopy(self.config)
        bad["joints"]["shoulder"]["motor_can_id"] = 127
        with self.assertRaisesRegex(ValueError, "相同 CAN ID"):
            ClassicConfigStore.validate(bad)

    def test_torque_ceiling_and_requested_defaults(self) -> None:
        self.assertEqual(self.config["joints"]["elbow"]["maximum_torque_nm"], 5.0)
        self.assertEqual(self.config["joints"]["shoulder"]["maximum_torque_nm"], 5.0)
        for item in self.config["joints"].values():
            self.assertEqual(item["default_torque_limit_nm"], 3.50)
            self.assertEqual(item["default_speed_rad_s"], 0.30)
        ClassicConfigStore.validate(self.config)
        too_high = copy.deepcopy(self.config)
        too_high["joints"]["elbow"]["maximum_torque_nm"] = 5.01
        with self.assertRaisesRegex(ValueError, "转矩配置"):
            ClassicConfigStore.validate(too_high)


if __name__ == "__main__":
    unittest.main()
