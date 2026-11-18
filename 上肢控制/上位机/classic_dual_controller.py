"""Safety-focused dual-CyberGear controller for the classic Tk dashboard.

This module contains no GUI or serial-port discovery.  A single transport is
injected by the caller, which makes the controller fully testable with an
offline fake link.  The controller owns both configured motor roles and never
accepts a raw CAN ID from a motion request.
"""

from __future__ import annotations

import copy
import json
import math
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Protocol

from cybergear_protocol import (
    CMD_FAULT_FEEDBACK,
    CMD_GET_DEVICE_ID,
    CMD_PARAM_READ,
    HOST_ID_DEFAULT,
    PARAM_LIMIT_TORQUE,
    PARAM_RUN_MODE,
    PARAMETER_DIAGNOSTICS,
    RUN_MODE_CONTROL,
    decode_feedback,
    decode_slcan_extended,
    decode_diagnostic_parameter,
    make_enable,
    make_get_device_id,
    make_motor_control,
    make_param_write,
    make_param_read,
    make_stop,
    uint_to_float,
)
from motion_policy import (
    DEFAULT_GRAVITY, DEFAULT_MOTION_POLICY, DEFAULT_LOAD_COMPENSATION, calibration_matches,
    gravity_config, gravity_torques, load_compensation_config, residual_bias, policy, validate_motion_settings,
    SUPPORT_PRESET, motion_gains,
)


JOINT_KEYS = ("elbow", "shoulder")
CONTROL_PERIOD_S = 0.01
START_FEEDBACK_MAX_AGE_S = 0.60
START_STILL_SPEED_RAD_S = 0.03
START_STILL_DWELL_S = 0.40
START_STILL_MIN_SAMPLES = 4
START_STILL_TIMEOUT_S = 3.0
START_STILL_POLL_S = 0.10
START_STILL_MAX_GAP_S = 0.25
START_STILL_WINDOW_S = 0.55
START_STILL_MAX_EXCURSION_RAD = math.radians(0.10)
START_STILL_GROSS_SPEED_RAD_S = 0.15
ACTIVE_FEEDBACK_TIMEOUT_S = 0.30
MAX_SAFE_TEMPERATURE_C = 60.0


DEFAULT_CONFIG: dict = {
    "schema_version": 2,
    "motion_policy": copy.deepcopy(DEFAULT_MOTION_POLICY),
    "gravity": copy.deepcopy(DEFAULT_GRAVITY),
    "load_compensation": copy.deepcopy(DEFAULT_LOAD_COMPENSATION),
    "bus": {
        "port": "COM11",
        "serial_baud": 921600,
        "can_bitrate": 1_000_000,
        "host_can_id": HOST_ID_DEFAULT,
    },
    "joints": {
        "elbow": {
            "display_name": "肘关节",
            "motor_can_id": 127,
            "expected_uid": "034C3B8A2093B015",
            "commissioned": False,
            "limits_confirmed": False,
            "direction": 1,
            "software_zero_rad": None,
            "minimum_angle_deg": 0.0,
            "maximum_angle_deg": 90.0,
            "maximum_speed_rad_s": 0.30,
            "default_speed_rad_s": 0.30,
            "maximum_torque_nm": 5.0,
            "default_torque_limit_nm": 3.50,
            "maximum_acceleration_rad_s2": 0.20,
            "hard_limit_margin_deg": 5.0,
            "motion_kp": 3.0,
            "motion_kd": 1.0,
        },
        "shoulder": {
            "display_name": "肩/大臂关节",
            "motor_can_id": 126,
            "expected_uid": "3A4B3B8A2093B015",
            "commissioned": False,
            "limits_confirmed": False,
            "direction": 1,
            "software_zero_rad": None,
            "minimum_angle_deg": -45.0,
            "maximum_angle_deg": 90.0,
            "maximum_speed_rad_s": 0.30,
            "default_speed_rad_s": 0.30,
            "maximum_torque_nm": 5.0,
            "default_torque_limit_nm": 3.50,
            "maximum_acceleration_rad_s2": 0.20,
            "hard_limit_margin_deg": 5.0,
            "motion_kp": 3.0,
            "motion_kd": 1.0,
        },
    },
    "legacy_zero_candidates_rad": {
        "motor_127_from_original_copy": -0.42089240098448855,
        "motor_126_from_migrated_copy": -0.4761165520020434,
    },
}


class Transport(Protocol):
    rx_queue: queue.Queue[str]

    def send(self, can_id: int, payload: bytes) -> None: ...


class ClassicConfigStore:
    def __init__(self, path: Path) -> None:
        self.path = path

    def load(self) -> dict:
        config = copy.deepcopy(DEFAULT_CONFIG)
        try:
            loaded = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            loaded = {}
        if not isinstance(loaded, dict):
            loaded = {}
        if loaded.get("schema_version") == 2:
            for section in ("motion_policy", "gravity", "load_compensation"):
                config[section].update(loaded.get(section, {}))
            config["bus"].update(loaded.get("bus", {}))
            for key in JOINT_KEYS:
                config["joints"][key].update(loaded.get("joints", {}).get(key, {}))
            if isinstance(loaded.get("legacy_zero_candidates_rad"), dict):
                config["legacy_zero_candidates_rad"].update(loaded["legacy_zero_candidates_rad"])
        self.validate(config)
        return config

    def save(self, config: dict) -> None:
        self.validate(config)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.path)

    @staticmethod
    def validate(config: dict) -> None:
        if config.get("schema_version") != 2:
            raise ValueError("经典双电机配置 schema_version 必须为 2")
        joints = config.get("joints", {})
        if set(joints) != set(JOINT_KEYS):
            raise ValueError("配置必须且只能包含 elbow 与 shoulder")
        ids: list[int] = []
        uids: list[str] = []
        for key in JOINT_KEYS:
            item = joints[key]
            motor_id = int(item["motor_can_id"])
            uid = str(item["expected_uid"]).upper()
            if not 0 <= motor_id <= 127:
                raise ValueError(f"{key} CAN ID 必须在 0～127")
            if len(uid) != 16 or any(ch not in "0123456789ABCDEF" for ch in uid):
                raise ValueError(f"{key} expected_uid 必须是 16 位十六进制")
            minimum = float(item["minimum_angle_deg"])
            maximum = float(item["maximum_angle_deg"])
            if minimum >= maximum:
                raise ValueError(f"{key} 最小角度必须小于最大角度")
            if int(item["direction"]) not in (-1, 1):
                raise ValueError(f"{key} direction 只能为 -1 或 +1")
            numeric_fields = (
                "minimum_angle_deg", "maximum_angle_deg", "maximum_speed_rad_s",
                "default_speed_rad_s", "maximum_torque_nm", "default_torque_limit_nm",
                "maximum_acceleration_rad_s2", "hard_limit_margin_deg", "motion_kp", "motion_kd",
            )
            if any(not math.isfinite(float(item[field])) for field in numeric_fields):
                raise ValueError(f"{key} 配置中存在非有限数值")
            zero = item.get("software_zero_rad")
            if zero is not None and not math.isfinite(float(zero)):
                raise ValueError(f"{key} 软件零点无效")
            if not 0.02 <= float(item["default_speed_rad_s"]) <= float(item["maximum_speed_rad_s"]) <= 0.30:
                raise ValueError(f"{key} 速度配置超出经典版上限")
            if not 0.10 <= float(item["default_torque_limit_nm"]) <= float(item["maximum_torque_nm"]) <= 5.0:
                raise ValueError(f"{key} 转矩配置超出经典版上限")
            if float(item["maximum_acceleration_rad_s2"]) <= 0 or float(item["hard_limit_margin_deg"]) < 0:
                raise ValueError(f"{key} 加速度或保护边界无效")
            if not 0 <= float(item["motion_kp"]) <= 20 or not 0 <= float(item["motion_kd"]) <= 5:
                raise ValueError(f"{key} PD 参数超出上限")
            ids.append(motor_id)
            uids.append(uid)
        if len(set(ids)) != len(ids):
            raise ValueError("两个关节不能使用相同 CAN ID")
        if len(set(uids)) != len(uids):
            raise ValueError("两个关节不能绑定相同 MCU UID")
        validate_motion_settings(config)


@dataclass
class JointRuntime:
    key: str
    observed_uid: str | None = None
    verified: bool = False
    actual_raw_rad: float | None = None
    actual_deg: float | None = None
    target_deg: float | None = None
    setpoint_deg: float | None = None
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_c: float = 0.0
    fault_bits: int = 0
    mode_status: int = 0
    last_feedback_at: float | None = None
    stationary_feedback: deque = field(default_factory=lambda: deque(maxlen=128))
    phase: str = "stopped"
    on_target: bool = False
    arrival_since: float | None = None
    stall_since: float | None = None
    overtorque_since: float | None = None
    gravity_torque_nm: float = 0.0
    estimated_command_torque_nm: float = 0.0
    load_bias_nm: float = 0.0
    last_sent_control: dict | None = None
    notice: str = ""
    parameter_values: dict = field(default_factory=dict)
    parameter_times: dict = field(default_factory=dict)
    history: dict[str, deque] = field(
        default_factory=lambda: {
            "angle": deque(maxlen=1200),
            "velocity": deque(maxlen=1200),
            "torque": deque(maxlen=1200),
            "temperature": deque(maxlen=1200),
            "target": deque(maxlen=1200),
            "error": deque(maxlen=1200),
            "gravity": deque(maxlen=1200),
            "estimated_torque": deque(maxlen=1200),
            "iq": deque(maxlen=1200),
            "vbus": deque(maxlen=1200),
        }
    )


@dataclass(frozen=True)
class MotionPlan:
    epoch: int
    started_at: float
    duration: float
    keys: tuple[str, ...]
    start_deg: dict[str, float]
    target_deg: dict[str, float]
    torque_limit: dict[str, float]
    kp: dict[str, float]
    kd: dict[str, float]
    gravity_started_at: float | None = None


def coordinated_duration(
    starts_deg: dict[str, float],
    targets_deg: dict[str, float],
    speeds_rad_s: dict[str, float],
    accelerations_rad_s2: dict[str, float],
) -> float:
    duration = 0.40
    for key, target in targets_deg.items():
        distance = abs(math.radians(target - starts_deg[key]))
        if distance <= math.radians(0.05):
            continue
        duration = max(
            duration,
            1.875 * distance / speeds_rad_s[key],
            math.sqrt(5.7735 * distance / accelerations_rad_s2[key]),
        )
    return duration


class DualMotorController:
    def __init__(
        self,
        transport: Transport,
        config: dict,
        *,
        save_config: Callable[[dict], None] | None = None,
        log: Callable[[str], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        telemetry: Callable[[str, dict], object] | None = None,
    ) -> None:
        ClassicConfigStore.validate(config)
        self.transport = transport
        self.config = config
        self.save_config_callback = save_config
        self.log_callback = log or (lambda _message: None)
        self.clock = clock
        self.telemetry_callback = telemetry
        self.telemetry_error = ""
        self.host_id = int(config["bus"].get("host_can_id", HOST_ID_DEFAULT))
        self.runtimes = {key: JointRuntime(key) for key in JOINT_KEYS}
        self.id_to_key = {
            int(config["joints"][key]["motor_can_id"]): key for key in JOINT_KEYS
        }
        self.state_lock = threading.RLock()
        self.feedback_condition = threading.Condition(self.state_lock)
        self.command_lock = threading.RLock()
        self.stop_event = threading.Event()
        self.rx_thread: threading.Thread | None = None
        self.supervisor_thread: threading.Thread | None = None
        self.motion_plan: MotionPlan | None = None
        self.motion_epoch = 0
        self.safety_latched = False
        self.safety_reason = ""
        self.pending_parameters: dict[tuple[int, int], float] = {}
        self.parameter_trace: deque = deque(maxlen=1024)
        self.parameter_last_requests: dict[tuple[int, int], float] = {}
        self.last_parameter_poll = 0.0
        self.parameter_poll_cursor = 0
        self.last_control_at: float | None = None

    def start(self) -> None:
        self.stop_event.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, name="classic-dual-rx", daemon=True)
        self.supervisor_thread = threading.Thread(
            target=self._supervisor_loop,
            name="classic-dual-supervisor",
            daemon=True,
        )
        self.rx_thread.start()
        self.supervisor_thread.start()

    def shutdown(self) -> None:
        try:
            self.stop_all("关闭经典双电机上位机")
        finally:
            self.stop_event.set()
            if self.rx_thread:
                self.rx_thread.join(timeout=0.5)
            if self.supervisor_thread:
                self.supervisor_thread.join(timeout=0.5)

    def _joint_config(self, key: str) -> dict:
        if key not in JOINT_KEYS:
            raise KeyError(f"未知关节：{key}")
        return self.config["joints"][key]

    def _emit_telemetry(self, kind: str, payload: dict) -> None:
        if self.telemetry_callback is not None:
            try:
                self.telemetry_callback(kind, payload)
            except Exception as exc:
                # A failed recorder must never stop or interrupt motor control.
                self.telemetry_error = f"数据记录回调异常：{exc}"

    def _send(self, frame: tuple[int, bytes], note: str, *, quiet: bool = False) -> None:
        can_id, payload = frame
        self.transport.send(can_id, payload)
        if not quiet:
            self.log_callback(f"TX {can_id:08X} [{payload.hex(' ').upper()}] {note}")

    def discover(self, timeout_s: float = 1.2) -> dict[str, bool]:
        with self.command_lock:
            self.stop_all("重新识别电机前停止双机")
            with self.state_lock:
                for runtime in self.runtimes.values():
                    runtime.observed_uid = None
                    runtime.verified = False
            for _ in range(2):
                for key in JOINT_KEYS:
                    cfg = self._joint_config(key)
                    self._send(
                        make_get_device_id(int(cfg["motor_can_id"]), self.host_id),
                        f"读取 {cfg['display_name']} ID/UID",
                    )
                    time.sleep(0.04)
            deadline = self.clock() + timeout_s
            with self.feedback_condition:
                while self.clock() < deadline and not all(
                    self.runtimes[key].verified for key in JOINT_KEYS
                ):
                    self.feedback_condition.wait(timeout=min(0.05, deadline - self.clock()))
            return {key: self.runtimes[key].verified for key in JOINT_KEYS}

    def _rx_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = self.transport.rx_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self.feed_line(line)
            except Exception as exc:
                self.log_callback(f"RX 处理失败：{exc}")
                self._trip(f"接收处理异常：{exc}")

    def feed_line(self, line: str) -> None:
        timestamp_ns, monotonic_ns = time.time_ns(), time.monotonic_ns()
        telemetry = None
        decoded = decode_slcan_extended(line)
        if not decoded:
            self.log_callback(f"RX 未解析：{line}")
            return
        can_id, payload = decoded
        command = (can_id >> 24) & 0x1F
        trip_reason: str | None = None
        if command == CMD_GET_DEVICE_ID and len(payload) == 8:
            motor_id = (can_id >> 8) & 0xFF
            key = self.id_to_key.get(motor_id)
            uid = payload.hex().upper()
            if key is None:
                self.log_callback(f"RX 未配置设备：ID {motor_id} UID {uid}")
                return
            cfg = self._joint_config(key)
            with self.feedback_condition:
                runtime = self.runtimes[key]
                runtime.observed_uid = uid
                runtime.verified = uid == str(cfg["expected_uid"]).upper()
                self.feedback_condition.notify_all()
            self.log_callback(
                f"RX {cfg['display_name']} ID {motor_id} UID {uid} "
                + ("验证通过" if runtime.verified else "UID 不匹配")
            )
            if not runtime.verified:
                trip_reason = f"{cfg['display_name']} UID 不匹配"
        elif command == CMD_PARAM_READ:
            decoded_parameter = decode_diagnostic_parameter(can_id, payload, self.host_id, set(self.id_to_key))
            now = self.clock()
            outcome = "rejected_format_status_address_or_value"
            if decoded_parameter:
                motor_id, index, value = decoded_parameter
                with self.feedback_condition:
                    sent = self.pending_parameters.pop((motor_id, index), None)
                    outcome = "unsolicited" if sent is None else "expired"
                    if sent is not None and 0 <= now - sent <= 0.8:
                        outcome = "accepted"
                        runtime = self.runtimes[self.id_to_key[motor_id]]
                        runtime.parameter_values[index] = value
                        runtime.parameter_times[index] = now
                        telemetry = ("parameters", {
                            "timestamp_unix_ns": timestamp_ns, "monotonic_ns": monotonic_ns,
                            "joint": runtime.key, "motor_can_id": motor_id,
                            "parameter_index_hex": f"0x{index:04X}",
                            "parameter_name": PARAMETER_DIAGNOSTICS[index][0], "value": value,
                            "unit": {0x700B:"N*m", 0x7018:"A", 0x701A:"A", 0x701C:"V"}.get(index, ""),
                            "can_id_hex": f"{can_id:08X}", "payload_hex": payload.hex().upper(),
                        })
                        plan = self.motion_plan
                        if plan and runtime.key in plan.keys:
                            if index == PARAM_RUN_MODE and value != RUN_MODE_CONTROL:
                                trip_reason = "电机运行模式读回发生变化"
                            if index == PARAM_LIMIT_TORQUE and not math.isclose(value, plan.torque_limit[runtime.key], abs_tol=0.02):
                                trip_reason = "电机力矩上限读回与命令不一致"
                        self.feedback_condition.notify_all()
            with self.state_lock:
                self.parameter_trace.append({"time": now, "direction": "RX", "can_id": f"{can_id:08X}",
                    "payload": payload.hex().upper(), "index": int.from_bytes(payload[:2], "little"),
                    "outcome": outcome})
        elif command == CMD_FAULT_FEEDBACK:
            trip_reason = f"收到 Type 21 故障/警告帧 0x{can_id:08X}"
        else:
            feedback = decode_feedback(can_id, payload)
            if feedback and feedback.host_id == self.host_id:
                key = self.id_to_key.get(feedback.motor_id)
                if key is None:
                    return
                now = self.clock()
                cfg = self._joint_config(key)
                with self.feedback_condition:
                    runtime = self.runtimes[key]
                    runtime.actual_raw_rad = feedback.position_rad
                    runtime.velocity_rad_s = feedback.velocity_rad_s
                    runtime.torque_nm = feedback.torque_nm
                    runtime.temperature_c = feedback.temperature_c
                    runtime.fault_bits = feedback.fault_bits
                    runtime.mode_status = feedback.mode_status
                    runtime.last_feedback_at = now
                    runtime.stationary_feedback.append((now, feedback.position_rad, feedback.velocity_rad_s))
                    zero = cfg.get("software_zero_rad")
                    if zero is None:
                        runtime.actual_deg = None
                    else:
                        runtime.actual_deg = int(cfg["direction"]) * math.degrees(
                            feedback.position_rad - float(zero)
                        )
                    telemetry = ("feedback", {
                        "timestamp_unix_ns": timestamp_ns, "monotonic_ns": monotonic_ns,
                        "joint": key, "motor_can_id": feedback.motor_id,
                        "observed_uid": runtime.observed_uid, "identity_verified": runtime.verified,
                        "software_zero_rad": zero, "direction": int(cfg["direction"]),
                        "position_motor_rad": feedback.position_rad,
                        "position_joint_rad": math.radians(runtime.actual_deg) if runtime.actual_deg is not None else None,
                        "position_joint_deg": runtime.actual_deg,
                        "velocity_motor_rad_s": feedback.velocity_rad_s,
                        "velocity_joint_rad_s": int(cfg["direction"])*feedback.velocity_rad_s,
                        "torque_feedback_motor_nm": feedback.torque_nm, "temperature_c": feedback.temperature_c,
                        "mode_status": feedback.mode_status, "fault_bits": feedback.fault_bits,
                        "target_joint_rad": math.radians(runtime.target_deg) if runtime.target_deg is not None else None,
                        "setpoint_joint_rad": math.radians(runtime.setpoint_deg) if runtime.setpoint_deg is not None else None,
                        "target_error_rad": math.radians(runtime.target_deg-runtime.actual_deg)
                            if runtime.target_deg is not None and runtime.actual_deg is not None else None,
                        "control_phase": runtime.phase, "safety_latched": self.safety_latched,
                        "can_id_hex": f"{can_id:08X}", "payload_hex": payload.hex().upper(),
                    })
                    sample_time = time.time()
                    runtime.history["angle"].append((sample_time, runtime.actual_deg))
                    runtime.history["velocity"].append((sample_time, feedback.velocity_rad_s))
                    runtime.history["torque"].append((sample_time, feedback.torque_nm))
                    runtime.history["temperature"].append((sample_time, feedback.temperature_c))
                    runtime.history["target"].append((sample_time, runtime.target_deg))
                    runtime.history["error"].append((sample_time, None if runtime.actual_deg is None or runtime.target_deg is None else runtime.target_deg-runtime.actual_deg))
                    runtime.history["gravity"].append((sample_time, runtime.gravity_torque_nm))
                    runtime.history["estimated_torque"].append((sample_time, runtime.estimated_command_torque_nm))
                    for metric, index in (("iq", 0x701A), ("vbus", 0x701C)):
                        value = runtime.parameter_values.get(index) if now-runtime.parameter_times.get(index, -math.inf) <= 3 else None
                        runtime.history[metric].append((sample_time, value))
                    active = self.motion_plan is not None
                    if active:
                        hard_minimum = min(0.0, float(cfg["minimum_angle_deg"])) - float(
                            cfg["hard_limit_margin_deg"]
                        )
                        hard_maximum = float(cfg["maximum_angle_deg"]) + float(
                            cfg["hard_limit_margin_deg"]
                        )
                        if feedback.fault_bits:
                            trip_reason = f"{cfg['display_name']} 故障 0x{feedback.fault_bits:02X}"
                        elif feedback.temperature_c > MAX_SAFE_TEMPERATURE_C:
                            trip_reason = f"{cfg['display_name']} 温度过高 {feedback.temperature_c:.1f}°C"
                        elif runtime.actual_deg is not None and not hard_minimum <= runtime.actual_deg <= hard_maximum:
                            trip_reason = f"{cfg['display_name']} 越过硬保护边界 {runtime.actual_deg:.1f}°"
                    self.feedback_condition.notify_all()
        if telemetry is not None:
            self._emit_telemetry(*telemetry)
        if trip_reason:
            self._trip(trip_reason)

    def _wait_for_feedback_after(self, keys: tuple[str, ...], after: float, timeout_s: float,
                                 mode_status: int | None = None) -> None:
        deadline = self.clock() + timeout_s
        with self.feedback_condition:
            while self.clock() < deadline:
                if all(
                    self.runtimes[key].last_feedback_at is not None
                    and self.runtimes[key].last_feedback_at >= after
                    and (mode_status is None or self.runtimes[key].mode_status == mode_status)
                    for key in keys
                ):
                    return
                self.feedback_condition.wait(timeout=min(0.05, deadline - self.clock()))
        missing = [
            self._joint_config(key)["display_name"]
            for key in keys
            if self.runtimes[key].last_feedback_at is None
            or self.runtimes[key].last_feedback_at < after
            or (mode_status is not None and self.runtimes[key].mode_status != mode_status)
        ]
        description = "没有收到使能后的 Motor 模式确认：" if mode_status == 2 else "没有收到停止后的新反馈："
        raise RuntimeError(description + "、".join(missing))

    def _wait_for_stationary(self) -> None:
        """Stopped motors need repeated stop replies; never enable to get samples."""
        started = self.clock()
        deadline = started + START_STILL_TIMEOUT_S
        next_poll = started
        with self.feedback_condition:
            for runtime in self.runtimes.values():
                runtime.stationary_feedback.clear()
        self.log_callback("启动静止检查：最多等待 3 秒，连续多帧静止 0.4 秒后才允许使能")
        while self.clock() < deadline:
            now = self.clock()
            if now >= next_poll:
                self._require_stops_sent(self._send_stop_frames(
                    "启动静止检查（保持停止、刷新反馈）", quiet=now > started))
                next_poll = now + START_STILL_POLL_S
            with self.feedback_condition:
                if self.safety_latched:
                    raise RuntimeError(f"安全锁存尚未复位：{self.safety_reason}")
                if self.stop_event.is_set():
                    raise RuntimeError("控制器已关闭，取消启动")
                ready = True
                reasons = []
                now = self.clock()
                for key in JOINT_KEYS:
                    runtime = self.runtimes[key]
                    name = self._joint_config(key)["display_name"]
                    if runtime.fault_bits:
                        raise RuntimeError(f"{name} 静止检查发现故障 0x{runtime.fault_bits:02X}")
                    if runtime.temperature_c > MAX_SAFE_TEMPERATURE_C:
                        raise RuntimeError(f"{name} 温度过高")
                    fresh = (runtime.last_feedback_at is not None
                             and runtime.last_feedback_at >= started
                             and now - runtime.last_feedback_at <= START_STILL_MAX_GAP_S)
                    window = [s for s in runtime.stationary_feedback
                              if s[0] >= max(started, now - START_STILL_WINDOW_S)]
                    enough = (len(window) >= START_STILL_MIN_SAMPLES
                              and window[-1][0] - window[0][0] >= START_STILL_DWELL_S)
                    excursion = max(s[1] for s in window) - min(s[1] for s in window) if window else math.inf
                    mean_speed = abs(sum(s[2] for s in window)/len(window)) if window else math.inf
                    stable = (fresh and enough
                              and all(b[0]-a[0] <= START_STILL_MAX_GAP_S for a,b in zip(window, window[1:]))
                              and max(abs(s[2]) for s in window) <= START_STILL_GROSS_SPEED_RAD_S
                              and mean_speed <= START_STILL_SPEED_RAD_S
                              and excursion <= START_STILL_MAX_EXCURSION_RAD)
                    if not stable:
                        ready = False
                        detail = (f"速度 {runtime.velocity_rad_s:+.4f} rad/s，多帧平均速度 {mean_speed:.4f}（需≤{START_STILL_SPEED_RAD_S:.2f}），"
                                  f"角度波动 {math.degrees(excursion):.3f}°（需≤0.100°）"
                                  if fresh else "缺少新鲜反馈")
                        reasons.append(f"{name}：{detail}，未连续静止 {START_STILL_DWELL_S:.1f} 秒")
                if ready:
                    self.log_callback("两关节连续静止检查通过")
                    return
                self.feedback_condition.wait(timeout=max(0.001, min(0.05, deadline - now)))
        raise RuntimeError("机构静止检查超时（3秒），未使能任何电机；"
                           + "；".join(reasons) + "。请支撑机构，待稳定后重试")

    def _require_all_verified(self) -> None:
        missing = [
            self._joint_config(key)["display_name"]
            for key in JOINT_KEYS
            if not self.runtimes[key].verified
        ]
        if missing:
            raise RuntimeError("以下关节身份尚未通过 UID 验证：" + "、".join(missing))

    def capture_zero(self, key: str) -> float:
        cfg = self._joint_config(key)
        with self.command_lock:
            self._require_all_verified()
            self._cancel_motion_locked()
            since = self.clock()
            self._require_stops_sent(self._send_stop_frames("采集软件零点前停止两台电机"))
            self._wait_for_feedback_after((key,), since, 1.0)
            with self.state_lock:
                position = self.runtimes[key].actual_raw_rad
                if position is None:
                    raise RuntimeError("没有可用的原始位置反馈")
                cfg["software_zero_rad"] = position
                cfg["commissioned"] = False
                self.runtimes[key].actual_deg = 0.0
            self._invalidate_gravity()
            self._save_config()
            self.log_callback(f"{cfg['display_name']} 软件零点已保存：{position:.6f} rad；需重新验收")
            return position

    def update_joint_config(
        self,
        key: str,
        *,
        direction: int,
        minimum_angle_deg: float,
        maximum_angle_deg: float,
        maximum_speed_rad_s: float,
        maximum_torque_nm: float,
        motion_kp: float,
        motion_kd: float,
        limits_confirmed: bool,
    ) -> None:
        cfg = self._joint_config(key)
        if direction not in (-1, 1):
            raise ValueError("方向只能为 -1 或 +1")
        if minimum_angle_deg >= maximum_angle_deg:
            raise ValueError("最小角度必须小于最大角度")
        if not 0.02 <= maximum_speed_rad_s <= 0.30:
            raise ValueError("经典版最大速度必须在 0.02～0.30 rad/s")
        if not 0.10 <= maximum_torque_nm <= 5.0:
            raise ValueError("经典版最大转矩必须在 0.10～5.0 N·m")
        if not 0.0 <= motion_kp <= 20.0 or not 0.0 <= motion_kd <= 5.0:
            raise ValueError("Kp 必须为 0～20，Kd 必须为 0～5")
        with self.command_lock:
            self._require_stops_sent(self.stop_all("修改关节安全配置"))
            if direction != cfg["direction"]:
                self._invalidate_gravity()
            cfg.update(
                {
                    "direction": direction,
                    "minimum_angle_deg": minimum_angle_deg,
                    "maximum_angle_deg": maximum_angle_deg,
                    "maximum_speed_rad_s": maximum_speed_rad_s,
                    "default_speed_rad_s": min(float(cfg["default_speed_rad_s"]), maximum_speed_rad_s),
                    "maximum_torque_nm": maximum_torque_nm,
                    "default_torque_limit_nm": min(float(cfg["default_torque_limit_nm"]), maximum_torque_nm),
                    "motion_kp": motion_kp,
                    "motion_kd": motion_kd,
                    "limits_confirmed": bool(limits_confirmed),
                    "commissioned": False,
                }
            )
            self._save_config()

    def set_commissioned(self, key: str, value: bool) -> None:
        cfg = self._joint_config(key)
        if value:
            self._require_all_verified()
            if cfg.get("software_zero_rad") is None:
                raise RuntimeError("必须先采集该关节的软件零点")
            if not cfg.get("limits_confirmed"):
                raise RuntimeError("必须先确认方向、软限位和机械空间")
        with self.command_lock:
            self._require_stops_sent(self.stop_all("修改关节验收状态"))
            cfg["commissioned"] = bool(value)
            self._save_config()

    def prepare_joints(self, settings: dict[str, dict]) -> None:
        """Confirm one or both joint setups after a fresh stopped feedback cycle."""
        if not settings or set(settings) - set(JOINT_KEYS):
            raise ValueError("请选择要准备的关节")
        required = {
            "direction", "minimum_angle_deg", "maximum_angle_deg", "maximum_speed_rad_s",
            "maximum_torque_nm", "motion_kp", "motion_kd",
        }
        with self.command_lock:
            if self.safety_latched:
                raise RuntimeError(f"安全锁存尚未复位：{self.safety_reason}")
            self._require_all_verified()
            candidate = copy.deepcopy(self.config)
            for key, proposed in settings.items():
                if not isinstance(proposed, dict) or set(proposed) != required:
                    raise ValueError(f"{key} 安全配置字段不完整")
                item = candidate["joints"][key]
                if item.get("software_zero_rad") is None:
                    raise RuntimeError(f"{item['display_name']} 请先采集软件零点")
                if proposed["direction"] not in (-1, 1):
                    raise ValueError(f"{item['display_name']} 方向只能是 -1 或 +1")
                values = {
                    "direction": int(proposed["direction"]),
                    "minimum_angle_deg": float(proposed["minimum_angle_deg"]),
                    "maximum_angle_deg": float(proposed["maximum_angle_deg"]),
                    "maximum_speed_rad_s": float(proposed["maximum_speed_rad_s"]),
                    "maximum_torque_nm": float(proposed["maximum_torque_nm"]),
                    "motion_kp": float(proposed["motion_kp"]),
                    "motion_kd": float(proposed["motion_kd"]),
                }
                item.update(values)
                if item["direction"] != self.config["joints"][key]["direction"]:
                    candidate["gravity"] = gravity_config(candidate)
                    candidate["gravity"].update(enabled=False, calibrated=False)
                item["default_speed_rad_s"] = min(float(item["default_speed_rad_s"]), values["maximum_speed_rad_s"])
                item["default_torque_limit_nm"] = min(float(item["default_torque_limit_nm"]), values["maximum_torque_nm"])
                item["limits_confirmed"] = True
                item["commissioned"] = True
            ClassicConfigStore.validate(candidate)

            self._cancel_motion_locked()
            since = self.clock()
            self._require_stops_sent(self._send_stop_frames("关节准备前停止双机"))
            self._wait_for_feedback_after(JOINT_KEYS, since, 1.0)
            with self.state_lock:
                if self.safety_latched:
                    raise RuntimeError(f"安全锁存尚未复位：{self.safety_reason}")
                for key in JOINT_KEYS:
                    runtime = self.runtimes[key]
                    if runtime.fault_bits:
                        raise RuntimeError(f"{self._joint_config(key)['display_name']} 仍存在故障")
                    if runtime.temperature_c > MAX_SAFE_TEMPERATURE_C:
                        raise RuntimeError(f"{self._joint_config(key)['display_name']} 温度过高")
                old_items = {key: copy.deepcopy(self.config["joints"][key]) for key in settings}
                old_angles = {key: self.runtimes[key].actual_deg for key in settings}
                old_gravity = copy.deepcopy(self.config.get("gravity", DEFAULT_GRAVITY))
                for key in settings:
                    item = candidate["joints"][key]
                    raw = self.runtimes[key].actual_raw_rad
                    if raw is None:
                        raise RuntimeError(f"{item['display_name']} 缺少原始位置反馈")
                    angle = int(item["direction"]) * math.degrees(raw - float(item["software_zero_rad"]))
                    minimum = min(0.0, float(item["minimum_angle_deg"])) - float(item["hard_limit_margin_deg"])
                    maximum = float(item["maximum_angle_deg"]) + float(item["hard_limit_margin_deg"])
                    if not minimum <= angle <= maximum:
                        raise RuntimeError(f"{item['display_name']} 当前角度 {angle:.1f}° 超出安全边界")
                for key in settings:
                    if self.config["joints"][key]["direction"] != candidate["joints"][key]["direction"]:
                        self._invalidate_gravity()
                        candidate["gravity"] = copy.deepcopy(self.config["gravity"])
                    self.config["joints"][key].update(candidate["joints"][key])
                    item = self.config["joints"][key]
                    self.runtimes[key].actual_deg = int(item["direction"]) * math.degrees(
                        float(self.runtimes[key].actual_raw_rad) - float(item["software_zero_rad"])
                    )
                try:
                    self._save_config()
                except Exception:
                    for key in settings:
                        self.config["joints"][key] = old_items[key]
                        self.runtimes[key].actual_deg = old_angles[key]
                    self.config["gravity"] = old_gravity
                    raise
            self.log_callback("关节准备完成：" + "、".join(self._joint_config(key)["display_name"] for key in settings))

    def _save_config(self) -> None:
        ClassicConfigStore.validate(self.config)
        if self.save_config_callback:
            self.save_config_callback(self.config)

    def _invalidate_gravity(self) -> None:
        gravity = gravity_config(self.config)
        gravity.update(enabled=False, calibrated=False)
        self.config["gravity"] = gravity

    def configure_gravity(self, *, enabled: bool, confirmed: bool = False,
                          shoulder_reference_deg: float | None = None,
                          forearm_reference_deg: float | None = None,
                          shoulder_angle_sign: int = 1, elbow_angle_sign: int = 1,
                          vertical_zero_confirmed: bool = False) -> None:
        """Local calibration only. Stops first; never enables or moves a motor."""
        if not isinstance(enabled, bool):
            raise ValueError("补偿开关必须为布尔值")
        if not isinstance(vertical_zero_confirmed, bool):
            raise ValueError("零位确认必须为布尔值")
        with self.command_lock:
            if self.motion_plan is not None:
                raise RuntimeError("请先支撑机构并停止双机，再修改重力补偿")
            candidate = copy.deepcopy(self.config)
            candidate["gravity"] = gravity_config(candidate)
            gravity = candidate["gravity"]
            if enabled:
                if self.safety_latched:
                    raise RuntimeError("请先排除故障并复位安全锁存，再标定补偿")
                if confirmed is not True:
                    raise ValueError("请确认水平转轴、物理角度和正方向后再标定")
                if shoulder_angle_sign not in (-1, 1) or elbow_angle_sign not in (-1, 1):
                    raise ValueError("物理角度正方向只能为 +1 或 -1")
                references = (shoulder_reference_deg, forearm_reference_deg)
                if vertical_zero_confirmed and (shoulder_angle_sign != 1 or elbow_angle_sign != 1 or any(v is not None for v in references)):
                    raise ValueError("竖直零位标定要求肩0°下垂、肘0°伸直、正向前抬；不可混用当前参考角")
                if not vertical_zero_confirmed and any(v is None or not math.isfinite(float(v)) or abs(float(v)) > 180 for v in references):
                    raise ValueError("请输入当前大臂、小臂相对水平的绝对角度（-180～180°）")
                self._require_all_verified()
                self._wait_for_stationary()
                with self.state_lock:
                    for key in JOINT_KEYS:
                        runtime = self.runtimes[key]
                        if runtime.actual_deg is None or runtime.fault_bits or runtime.temperature_c > MAX_SAFE_TEMPERATURE_C:
                            raise RuntimeError("两关节必须停止、无故障且软件零点有效")
                    gravity.update(
                        enabled=True, calibrated=True,
                        shoulder_angle_sign=shoulder_angle_sign, elbow_angle_sign=elbow_angle_sign,
                        shoulder_offset_deg=-90.0 if vertical_zero_confirmed else float(shoulder_reference_deg) - shoulder_angle_sign*self.runtimes["shoulder"].actual_deg,
                        elbow_offset_deg=0.0 if vertical_zero_confirmed else float(forearm_reference_deg)-float(shoulder_reference_deg)-elbow_angle_sign*self.runtimes["elbow"].actual_deg,
                        calibration_zeros_rad={k: self._joint_config(k)["software_zero_rad"] for k in JOINT_KEYS},
                        calibration_directions={k: self._joint_config(k)["direction"] for k in JOINT_KEYS},
                    )
            else:
                gravity["enabled"] = False
            ClassicConfigStore.validate(candidate)
            if self.save_config_callback:
                self.save_config_callback(candidate)
            with self.state_lock:
                self.config["gravity"] = gravity
            self.log_callback("空载重力补偿已标定并开启（下次手动运动时生效）" if enabled else "重力补偿已关闭")

    def configure_load_compensation(self, *, enabled: bool, confirmed: bool = False,
                                    profile: str | None = None, parameters: dict | None = None) -> None:
        """Save an explicitly selected profile/switch, never enable or retain learned torque."""
        if not isinstance(enabled, bool) or not isinstance(confirmed, bool):
            raise ValueError("补偿开关和确认必须是布尔值")
        if profile is not None and (profile != "tracking_support" or not enabled):
            raise ValueError("仅开启时可选择负载承托档")
        if parameters is not None:
            allowed = set(DEFAULT_LOAD_COMPENSATION) - {"enabled"}
            if not isinstance(parameters, dict) or set(parameters) - allowed or profile is not None:
                raise ValueError("运控参数字段无效；不能同时选择预设档")
        with self.command_lock, self.state_lock:
            if self.motion_plan is not None:
                raise RuntimeError("请先支撑机构并停止双机，再修改负载误差补偿")
            if enabled:
                if not confirmed:
                    raise ValueError("负载误差补偿为试验功能，需明确确认")
                if self.safety_latched:
                    raise RuntimeError("请先排除故障并复位安全锁存")
                self._require_all_verified()
            candidate = copy.deepcopy(self.config)
            candidate["load_compensation"] = {
                **(SUPPORT_PRESET if profile else load_compensation_config(candidate)),
                **(parameters or {}), "enabled": enabled}
            ClassicConfigStore.validate(candidate)
            if self.save_config_callback:
                self.save_config_callback(candidate)
            self.config["load_compensation"] = candidate["load_compensation"]
            for runtime in self.runtimes.values():
                runtime.load_bias_nm = 0.0
        self.log_callback("负载误差补偿已" + ("开启（试验）；下次手动启动时生效" if enabled else "关闭"))

    def _request_parameter(self, key: str, index: int) -> None:
        motor_id = int(self._joint_config(key)["motor_can_id"])
        frame = make_param_read(motor_id, index, self.host_id)
        with self.state_lock:
            self.pending_parameters[(motor_id, index)] = self.clock()
            self.parameter_last_requests[(motor_id, index)] = self.clock()
            self.parameter_trace.append({"time": self.clock(), "direction": "TX", "can_id": f"{frame[0]:08X}",
                "payload": frame[1].hex().upper(), "joint": key, "index": index, "outcome": "requested"})
        self._send(frame, "参数诊断读取", quiet=True)

    def diagnostics_snapshot(self) -> dict:
        with self.state_lock:
            return {"trace": list(self.parameter_trace), "pending": [
                {"motor_id": motor_id, "index": index, "age_s": self.clock()-sent}
                for (motor_id, index), sent in self.pending_parameters.items()]}

    def _verify_control_parameters(self, key: str, torque: float) -> None:
        for index, expected in ((PARAM_RUN_MODE, RUN_MODE_CONTROL), (PARAM_LIMIT_TORQUE, torque)):
            since = self.clock()
            with self.state_lock:
                self.runtimes[key].parameter_times.pop(index, None)
            self._request_parameter(key, index)
            deadline = self.clock() + 0.8
            next_read = self.clock() + 0.2
            with self.feedback_condition:
                runtime = self.runtimes[key]
                while runtime.parameter_times.get(index, -math.inf) < since:
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        raise RuntimeError(f"{self._joint_config(key)['display_name']} 参数 0x{index:04X} 读回超时，禁止使能")
                    if self.clock() >= next_read:
                        # A dropped read is retried, never the write or enable.
                        # Deadline and independent value comparison stay unchanged.
                        self._request_parameter(key, index)
                        next_read = self.clock() + 0.2
                    self.feedback_condition.wait(timeout=min(0.05, remaining))
                if not math.isclose(float(runtime.parameter_values[index]), expected, abs_tol=0.02):
                    raise RuntimeError(f"{self._joint_config(key)['display_name']} 参数读回不一致，禁止使能")

    def _poll_diagnostics(self, now: float) -> None:
        if now - self.last_parameter_poll < 0.1:
            return
        # Essential readbacks must not compete equally with unsupported optional
        # diagnostics. Retry missing reads within the unchanged 3-second guard.
        with self.state_lock:
            for token, sent in list(self.pending_parameters.items()):
                if now-sent > 0.8:
                    self.pending_parameters.pop(token)
                    self.parameter_trace.append({"time": now, "direction": "timeout", "motor_id": token[0],
                        "index": token[1], "outcome": "no_valid_reply"})
            critical = [(key, index) for key in JOINT_KEYS for index in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE)]
            eligible = [(key, index) for key, index in critical
                if now-self.pending_parameters.get((int(self._joint_config(key)["motor_can_id"]), index), -math.inf) >= 0.2]
            due = [(key, index) for key, index in eligible
                   if now-self.runtimes[key].parameter_times.get(index, -math.inf) >= 0.6]
            oldest = min(due, key=lambda item: self.parameter_last_requests.get(
                (int(self._joint_config(item[0])["motor_can_id"]), item[1]), -math.inf)) if due else (
                min(eligible, key=lambda item: self.runtimes[item[0]].parameter_times.get(item[1], -math.inf)) if eligible else None)
            critical_age = max(now-self.runtimes[key].parameter_times.get(index, -math.inf) for key, index in critical)
            optional = [(key, index) for key in JOINT_KEYS for index in PARAMETER_DIAGNOSTICS
                        if index not in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE)]
            if critical_age < 0.6 and self.parameter_poll_cursor % 5 == 4:
                key, index = optional[(self.parameter_poll_cursor // 5) % len(optional)]
            elif oldest:
                key, index = oldest
            else:
                return
            self.parameter_poll_cursor += 1
        self.last_parameter_poll = now
        self._request_parameter(key, index)

    def start_motion(
        self,
        targets_deg: dict[str, float],
        *,
        speed_rad_s: dict[str, float] | None = None,
        torque_limit_nm: dict[str, float] | None = None,
        kp: dict[str, float] | None = None,
        kd: dict[str, float] | None = None,
    ) -> float:
        unknown = set(targets_deg) - set(JOINT_KEYS)
        if unknown:
            raise ValueError("未知关节：" + "、".join(sorted(unknown)))
        keys = tuple(key for key in JOINT_KEYS if key in targets_deg)
        if not keys:
            raise ValueError("至少需要一个关节目标")
        with self.command_lock:
            if self.safety_latched:
                raise RuntimeError(f"安全锁存尚未复位：{self.safety_reason}")
            self._require_all_verified()
            previous = self.motion_plan
            if previous:
                if self.clock() < previous.started_at + previous.duration:
                    raise RuntimeError("请等待当前规划轨迹结束后再给新目标；无需等待到位，停止按钮始终有效")
                if set(keys) - set(previous.keys):
                    raise RuntimeError("新增使能关节前请先支撑机构并停止双机")
                targets_deg = {**previous.target_deg, **targets_deg}
                keys = previous.keys
            ClassicConfigStore.validate(self.config)
            if gravity_config(self.config)["enabled"] and set(keys) != set(JOINT_KEYS):
                raise RuntimeError("开启重力补偿后请使用双关节运动，另一关节目标可设为当前角度")
            speeds: dict[str, float] = {}
            torques: dict[str, float] = {}
            gains_kp: dict[str, float] = {}
            gains_kd: dict[str, float] = {}
            for key in keys:
                cfg = self._joint_config(key)
                runtime = self.runtimes[key]
                if not cfg.get("commissioned"):
                    raise RuntimeError(f"{cfg['display_name']} 尚未完成方向/零点/限位验收")
                if not cfg.get("limits_confirmed"):
                    raise RuntimeError(f"{cfg['display_name']} 软限位尚未确认")
                if cfg.get("software_zero_rad") is None:
                    raise RuntimeError(f"{cfg['display_name']} 尚未设置软件零点")
                target = float(targets_deg[key])
                if not math.isfinite(target):
                    raise ValueError(f"{cfg['display_name']} 目标角度无效")
                if not float(cfg["minimum_angle_deg"]) <= target <= float(cfg["maximum_angle_deg"]):
                    raise ValueError(f"{cfg['display_name']} 目标超过软限位")
                speed = float((speed_rad_s or {}).get(key, cfg["default_speed_rad_s"]))
                torque = float((torque_limit_nm or {}).get(key, previous.torque_limit[key] if previous else cfg["default_torque_limit_nm"]))
                default_kp, default_kd = motion_gains(self.config, key)
                requested_kp = float((kp or {}).get(key, previous.kp[key] if previous else default_kp))
                requested_kd = float((kd or {}).get(key, previous.kd[key] if previous else default_kd))
                if not 0.02 <= speed <= float(cfg["maximum_speed_rad_s"]):
                    raise ValueError(f"{cfg['display_name']} 速度超过配置上限")
                if not 0.10 <= torque <= float(cfg["maximum_torque_nm"]):
                    raise ValueError(f"{cfg['display_name']} 转矩限制超过配置上限")
                if not 0.0 <= requested_kp <= 20.0 or not 0.0 <= requested_kd <= 5.0:
                    raise ValueError(f"{cfg['display_name']} PD 参数超出上限")
                speeds[key] = speed
                torques[key] = torque
                gains_kp[key] = requested_kp
                gains_kd[key] = requested_kd
                if previous and (torque != previous.torque_limit[key] or requested_kp != previous.kp[key] or requested_kd != previous.kd[key]):
                    raise RuntimeError("持续控制中只允许更换目标；修改力矩或 PD 参数前须停止双机")

            if previous is None:
                self._cancel_motion_locked()
                self._wait_for_stationary()
            with self.state_lock:
                if self.safety_latched:
                    raise RuntimeError(f"安全锁存尚未复位：{self.safety_reason}")
                starts = {key: self.runtimes[key].actual_deg for key in keys}
                for joint_key in JOINT_KEYS:
                    runtime = self.runtimes[joint_key]
                    if runtime.fault_bits:
                        raise RuntimeError(f"{self._joint_config(joint_key)['display_name']} 停止后仍存在故障")
                    if runtime.temperature_c > MAX_SAFE_TEMPERATURE_C:
                        raise RuntimeError(f"{self._joint_config(joint_key)['display_name']} 温度过高")
                    if runtime.last_feedback_at is None or self.clock() - runtime.last_feedback_at > START_FEEDBACK_MAX_AGE_S:
                        raise RuntimeError("缺少两关节新鲜反馈，禁止运动")
            if any(value is None for value in starts.values()):
                raise RuntimeError("停止后没有获得完整的关节角度")
            start_values = {key: float(value) for key, value in starts.items() if value is not None}
            for key in keys:
                cfg = self._joint_config(key)
                if not (
                    min(0.0, float(cfg["minimum_angle_deg"])) - float(cfg["hard_limit_margin_deg"])
                    <= start_values[key]
                    <= float(cfg["maximum_angle_deg"]) + float(cfg["hard_limit_margin_deg"])
                ):
                    raise RuntimeError(f"{cfg['display_name']} 当前角度超出硬保护边界")
            if previous is not None:
                # Replan from the previous reference endpoint, not a loaded
                # actual position. Otherwise retargeting would erase PD error
                # instantly and drop support, especially on the unchanged joint.
                start_values = dict(previous.target_deg)
            duration = coordinated_duration(
                start_values,
                {key: float(targets_deg[key]) for key in keys},
                speeds,
                {key: float(self._joint_config(key)["maximum_acceleration_rad_s2"]) for key in keys},
            )
            raw_starts: dict[str, float] = {}
            for key in keys:
                cfg = self._joint_config(key)
                raw_start = float(cfg["software_zero_rad"]) + int(cfg["direction"]) * math.radians(
                    start_values[key]
                )
                raw_target = float(cfg["software_zero_rad"]) + int(cfg["direction"]) * math.radians(
                    float(targets_deg[key])
                )
                if not all(-4.0 * math.pi <= value <= 4.0 * math.pi for value in (raw_start, raw_target)):
                    raise RuntimeError(f"{cfg['display_name']} 原始位置超出 CyberGear 运控范围")
                raw_starts[key] = raw_start
            enabled_after = self.clock()
            try:
                if previous is None:
                    for key in keys:
                        cfg = self._joint_config(key)
                        motor_id = int(cfg["motor_can_id"])
                        self._send(make_param_write(motor_id, PARAM_RUN_MODE, RUN_MODE_CONTROL, "u8", self.host_id), "run_mode=0")
                        self._send(make_param_write(motor_id, PARAM_LIMIT_TORQUE, torques[key], "float", self.host_id), f"limit_torque={torques[key]:.2f}Nm")
                        self._verify_control_parameters(key, torques[key])
                    # Both devices must pass readback before either is enabled.
                    enabled_after = self.clock()
                    for key in keys:
                        motor_id = int(self._joint_config(key)["motor_can_id"])
                        self._send(make_enable(motor_id, self.host_id), "enable")
                        self._send(make_motor_control(motor_id, 0.0, raw_starts[key], 0.0, gains_kp[key], gains_kd[key]), "hold start")
                    self._wait_for_feedback_after(keys, enabled_after, 0.8, mode_status=2)
                with self.state_lock:
                    if self.safety_latched:
                        raise RuntimeError("启动期间触发保护，禁止运动")
                    for key in keys:
                        if self.runtimes[key].mode_status != 2:
                            raise RuntimeError(f"{self._joint_config(key)['display_name']} 未进入 Motor 模式")
                    self.motion_epoch += 1
                    epoch = self.motion_epoch
                    self.motion_plan = MotionPlan(
                        epoch=epoch,
                        started_at=self.clock(),
                        duration=duration,
                        keys=keys,
                        start_deg=start_values,
                        target_deg={key: float(targets_deg[key]) for key in keys},
                        torque_limit=torques,
                        kp=gains_kp,
                        kd=gains_kd,
                        gravity_started_at=previous.gravity_started_at if previous else self.clock(),
                    )
                    self.last_control_at = self.clock()
                    for key in keys:
                        runtime = self.runtimes[key]
                        runtime.target_deg = float(targets_deg[key])
                        runtime.setpoint_deg = start_values[key]
                        runtime.phase = "moving"
                        runtime.on_target = False
                        runtime.notice = ""
                        if previous is None:
                            runtime.load_bias_nm = 0.0
                        runtime.arrival_since = runtime.stall_since = None
                        runtime.overtorque_since = None
                self.log_callback(
                    "同步轨迹已启动："
                    + "，".join(
                        f"{self._joint_config(key)['display_name']} {start_values[key]:.1f}°→{targets_deg[key]:.1f}°"
                        for key in keys
                    )
                    + f"，共同时间 {duration:.2f}s"
                )
                return duration
            except Exception:
                self._cancel_motion_locked()
                self._send_stop_frames("启动失败保护停止")
                raise

    def stop_all(self, reason: str = "用户全局停止", *, latch: bool = False) -> dict[str, str]:
        with self.command_lock:
            self._cancel_motion_locked()
            if latch:
                with self.state_lock:
                    self.safety_latched = True
                    self.safety_reason = reason
            return self._send_stop_frames(reason)

    def _cancel_motion_locked(self) -> None:
        with self.state_lock:
            self.motion_epoch += 1
            self.motion_plan = None
            for runtime in self.runtimes.values():
                runtime.setpoint_deg = None
                runtime.phase = "stopped"
                runtime.on_target = False
                runtime.arrival_since = runtime.stall_since = None
                runtime.overtorque_since = None
                runtime.gravity_torque_nm = runtime.estimated_command_torque_nm = 0.0
                runtime.load_bias_nm = 0.0
                runtime.last_sent_control = None
                runtime.notice = ""

    def _send_stop_frames(self, reason: str, *, quiet: bool = False) -> dict[str, str]:
        results: dict[str, str] = {}
        for key in JOINT_KEYS:
            cfg = self._joint_config(key)
            try:
                self._send(
                    make_stop(int(cfg["motor_can_id"]), self.host_id),
                    f"{cfg['display_name']} stop-all：{reason}",
                    quiet=quiet,
                )
                results[key] = "sent"
            except Exception as exc:
                results[key] = f"error: {exc}"
                self.log_callback(f"{cfg['display_name']} 停止发送失败：{exc}")
        return results

    @staticmethod
    def _require_stops_sent(results: dict[str, str]) -> None:
        failed = [key for key, result in results.items() if result != "sent"]
        if failed:
            raise RuntimeError("双机停止未全部发送成功：" + "、".join(failed))

    def _trip(self, reason: str) -> None:
        with self.state_lock:
            already_latched = self.safety_latched and self.safety_reason == reason
            self.safety_latched = True
            self.safety_reason = reason
        if already_latched:
            return
        self.log_callback(f"安全锁存：{reason}")
        self.stop_all(reason, latch=True)

    def reset_safety(self) -> None:
        now = self.clock()
        with self.state_lock:
            self._require_all_verified()
            for key in JOINT_KEYS:
                runtime = self.runtimes[key]
                if runtime.fault_bits:
                    raise RuntimeError(f"{self._joint_config(key)['display_name']} 故障尚未清除")
                if runtime.last_feedback_at is None or now - runtime.last_feedback_at > START_FEEDBACK_MAX_AGE_S:
                    raise RuntimeError(f"{self._joint_config(key)['display_name']} 反馈不新鲜")
            self.safety_latched = False
            self.safety_reason = ""
        self.log_callback("安全锁存已人工复位；电机保持停止")

    def _supervisor_loop(self) -> None:
        next_tick = self.clock()
        while not self.stop_event.is_set():
            try:
                self.control_tick()
            except Exception as exc:
                self._trip(f"控制循环异常：{exc}")
            next_tick += CONTROL_PERIOD_S
            if next_tick < self.clock() - CONTROL_PERIOD_S:
                next_tick = self.clock()
            time.sleep(max(0.0, next_tick - self.clock()))

    def control_tick(self) -> None:
        """One serialized, offline-testable step. Stop wins over queued control."""
        with self.command_lock:
            now = self.clock()
            with self.state_lock:
                plan = self.motion_plan
                if plan is None:
                    if all(r.verified for r in self.runtimes.values()):
                        self._poll_diagnostics(now)
                    return
                if self.safety_latched:
                    return
                if self.last_control_at is not None and now-self.last_control_at > ACTIVE_FEEDBACK_TIMEOUT_S:
                    self._trip("控制循环超时；检查电脑休眠或阻塞")
                    return
                dt = CONTROL_PERIOD_S if self.last_control_at is None else max(0.0, now-self.last_control_at)
                self.last_control_at = now
                settings = policy(self.config)
                load_settings = load_compensation_config(self.config)
                elapsed = max(0.0, now-plan.started_at)
                u = min(1.0, elapsed/plan.duration)
                blend = 10*u**3-15*u**4+6*u**5
                rate = (30*u**2-60*u**3+30*u**4)/plan.duration if u < 1 else 0.0
                angles = {key: runtime.actual_deg for key, runtime in self.runtimes.items()}
                feedback_keys = JOINT_KEYS if gravity_config(self.config)["enabled"] else plan.keys
                for key in feedback_keys:
                    runtime, cfg = self.runtimes[key], self._joint_config(key)
                    age = math.inf if runtime.last_feedback_at is None else now-runtime.last_feedback_at
                    hard_min = min(0.0, float(cfg["minimum_angle_deg"]))-float(cfg["hard_limit_margin_deg"])
                    hard_max = float(cfg["maximum_angle_deg"])+float(cfg["hard_limit_margin_deg"])
                    reason = None
                    if age > ACTIVE_FEEDBACK_TIMEOUT_S:
                        reason = f"{cfg['display_name']} 反馈超时"
                    elif not runtime.verified or runtime.actual_raw_rad is None or runtime.actual_deg is None:
                        reason = "身份或位置反馈无效"
                    elif runtime.fault_bits or runtime.temperature_c > MAX_SAFE_TEMPERATURE_C:
                        reason = f"{cfg['display_name']} 故障或超温"
                    elif not hard_min <= runtime.actual_deg <= hard_max:
                        reason = f"{cfg['display_name']} 越过硬保护边界"
                    elif key in plan.keys and runtime.mode_status != 2:
                        reason = f"{cfg['display_name']} 已退出 Motor 模式"
                    if reason:
                        self._trip(reason)
                        return
                gravity = gravity_torques(self.config, angles)
                ramp_start = plan.gravity_started_at if plan.gravity_started_at is not None else plan.started_at
                ramp = max(0.0, min(1.0, (now-ramp_start)/settings["gravity_ramp_s"]))
                frames = []
                for key in plan.keys:
                    runtime, cfg = self.runtimes[key], self._joint_config(key)
                    for index in (PARAM_RUN_MODE, PARAM_LIMIT_TORQUE):
                        if now-runtime.parameter_times.get(index, -math.inf) > 3.0:
                            self._trip(f"{cfg['display_name']} 关键参数 0x{index:04X} 诊断读回超时")
                            return
                    vbus = runtime.parameter_values.get(0x701C)
                    if vbus is not None and now-runtime.parameter_times.get(0x701C, -math.inf) < 3 and not 16 <= vbus <= 28:
                        self._trip(f"{cfg['display_name']} 总线电压异常 {vbus:.1f}V")
                        return
                    setpoint = plan.start_deg[key]+(plan.target_deg[key]-plan.start_deg[key])*blend
                    runtime.setpoint_deg = setpoint
                    raw_position = float(cfg["software_zero_rad"])+int(cfg["direction"])*math.radians(setpoint)
                    raw_velocity = int(cfg["direction"])*math.radians((plan.target_deg[key]-plan.start_deg[key])*rate)
                    ff = int(cfg["direction"])*gravity[key]*ramp
                    ff = max(-plan.torque_limit[key], min(plan.torque_limit[key], ff))
                    runtime.gravity_torque_nm = ff
                    pd_torque = plan.kp[key]*(raw_position-runtime.actual_raw_rad)+plan.kd[key]*(raw_velocity-runtime.velocity_rad_s)
                    stable = self._arrival_is_stable(runtime, settings, now)
                    runtime.load_bias_nm = residual_bias(
                        load_settings, runtime.load_bias_nm,
                        raw_position-runtime.actual_raw_rad, ff+pd_torque, plan.torque_limit[key], dt,
                        stationary=stable, trajectory_complete=u >= 1.0,
                        tracking_error_rate_rad_s=self._tracking_error_rate(runtime, raw_velocity, now),
                    )
                    command_ff = max(-plan.torque_limit[key], min(plan.torque_limit[key], ff+runtime.load_bias_nm))
                    runtime.estimated_command_torque_nm = command_ff+pd_torque
                    error = abs(plan.target_deg[key]-runtime.actual_deg)
                    # Arrival is a display indicator only. The same position
                    # controller runs indefinitely, with no settling/recovery timer.
                    runtime.phase = "controlling" if u >= 1.0 else "moving"
                    if u >= 1.0:
                        near = error <= settings["arrival_tolerance_deg"] and stable
                        if near:
                            if runtime.arrival_since is None:
                                runtime.arrival_since = now
                        else:
                            runtime.arrival_since = None
                    else:
                        runtime.arrival_since = None
                    runtime.on_target = (runtime.arrival_since is not None
                                         and now-runtime.arrival_since >= settings["arrival_dwell_s"])
                    # Stall, feedback and hardware checks are independent of arrival.
                    stalled = error > settings["stall_error_deg"] and abs(runtime.velocity_rad_s) < settings["stall_speed_rad_s"] and abs(runtime.torque_nm) >= plan.torque_limit[key]*settings["stall_torque_ratio"]
                    if stalled and runtime.stall_since is None:
                        runtime.stall_since = now
                    if not stalled:
                        runtime.stall_since = None
                    if runtime.stall_since is not None and now-runtime.stall_since >= settings["stall_timeout_s"]:
                        self._trip(f"{cfg['display_name']} 高力矩低速且不到位，疑似堵转")
                        return
                    overtorque = abs(runtime.torque_nm) > max(plan.torque_limit[key]*1.15, plan.torque_limit[key]+0.3)
                    if overtorque and runtime.overtorque_since is None:
                        runtime.overtorque_since = now
                    if not overtorque:
                        runtime.overtorque_since = None
                    if runtime.overtorque_since is not None and now-runtime.overtorque_since >= 0.2:
                        self._trip(f"{cfg['display_name']} 反馈力矩持续超过设定上限")
                        return
                    frame = make_motor_control(int(cfg["motor_can_id"]), command_ff, raw_position, raw_velocity, plan.kp[key], plan.kd[key])
                    frames.append((key, frame, runtime.actual_raw_rad, runtime.velocity_rad_s, runtime.last_feedback_at))
            for key, frame, feedback_position, feedback_velocity, feedback_at in frames:
                with self.state_lock:
                    if self.motion_plan is not plan or self.safety_latched:
                        return
                self._send(frame, f"{key} 运动/保持", quiet=True)
                # Record only successful writes. Decode the actual quantized
                # wire fields; this is not a motor acknowledgement or torque measurement.
                p, v, kp_wire, kd_wire = struct.unpack(">HHHH", frame[1])
                wire_position = uint_to_float(p, -4*math.pi, 4*math.pi)
                wire_velocity = uint_to_float(v, -30, 30)
                wire_kp, wire_kd = uint_to_float(kp_wire, 0, 500), uint_to_float(kd_wire, 0, 5)
                wire_ff = uint_to_float((frame[0] >> 8) & 65535, -12, 12)
                wire_pd = wire_kp*(wire_position-feedback_position)+wire_kd*(wire_velocity-feedback_velocity)
                with self.state_lock:
                    if self.motion_plan is not plan or self.safety_latched:
                        return
                    self.runtimes[key].last_sent_control = {
                        "sent_at": self.clock(), "epoch": plan.epoch,
                        "can_id": f"{frame[0]:08X}", "payload": frame[1].hex().upper(),
                        "position_rad": wire_position, "velocity_rad_s": wire_velocity,
                        "kp": wire_kp, "kd": wire_kd,
                        "feedforward_nm": wire_ff, "pd_estimate_nm": wire_pd,
                        "estimated_total_nm": wire_ff+wire_pd, "torque_limit_nm": plan.torque_limit[key],
                        "feedback_sample_at": feedback_at,
                        "feedback_age_at_send_s": self.clock()-feedback_at,
                    }
                    telemetry = {
                        "timestamp_unix_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(),
                        "joint": key, "motor_can_id": int(self._joint_config(key)["motor_can_id"]),
                        "motion_epoch": plan.epoch, "position_motor_rad": wire_position,
                        "velocity_motor_rad_s": wire_velocity, "kp": wire_kp, "kd": wire_kd,
                        "feedforward_motor_nm": wire_ff, "torque_limit_nm": plan.torque_limit[key],
                        "pd_estimate_nm": wire_pd, "total_estimate_nm": wire_ff+wire_pd,
                        "gravity_feedforward_motor_nm": self.runtimes[key].gravity_torque_nm,
                        "load_bias_motor_nm": self.runtimes[key].load_bias_nm,
                        "feedback_age_at_send_s": self.clock()-feedback_at,
                        "can_id_hex": f"{frame[0]:08X}", "payload_hex": frame[1].hex().upper(),
                    }
                self._emit_telemetry("commands", telemetry)
            self._poll_diagnostics(now)

    @staticmethod
    def _tracking_error_rate(runtime: JointRuntime, reference_velocity: float, now: float) -> float | None:
        # Independent encoder slope avoids accumulating from noisy reported
        # speed. No learning with short, stale or gapped encoder history.
        samples = [s for s in runtime.stationary_feedback if 0 <= now-s[0] <= 0.15]
        if (len(samples) < 3 or samples[-1][0]-samples[0][0] < 0.08
                or now-samples[-1][0] > 0.08
                or any(b[0]-a[0] > 0.08 for a, b in zip(samples, samples[1:]))):
            return None
        t0 = samples[0][0]
        times = [s[0]-t0 for s in samples]
        mean_t = sum(times)/len(times)
        mean_p = sum(s[1] for s in samples)/len(samples)
        denominator = sum((t-mean_t)**2 for t in times)
        if denominator <= 0:
            return None
        velocity = sum((t-mean_t)*(s[1]-mean_p) for t, s in zip(times, samples))/denominator
        # A zero mean slope must not hide back-and-forth encoder motion.
        # 0.1 degrees allows quantization and planned acceleration curvature.
        if max(abs(s[1]-(mean_p+velocity*(t-mean_t))) for t, s in zip(times, samples)) > math.radians(.1):
            return None
        return reference_velocity-velocity

    @staticmethod
    def _arrival_is_stable(runtime: JointRuntime, settings: dict, now: float) -> bool:
        # Live speed feedback has zero-centred spikes even with a stationary
        # encoder. Require independent position stability, sufficient fresh
        # samples, signed mean speed and a gross-speed guard, not one frame.
        samples = [s for s in runtime.stationary_feedback if now-s[0] <= START_STILL_WINDOW_S]
        if len(samples) < START_STILL_MIN_SAMPLES or samples[-1][0]-samples[0][0] < START_STILL_DWELL_S:
            return False
        if now-samples[-1][0] > START_STILL_MAX_GAP_S or any(
                right[0]-left[0] > START_STILL_MAX_GAP_S for left, right in zip(samples, samples[1:])):
            return False
        positions = [s[1] for s in samples]
        velocities = [s[2] for s in samples]
        absolute_speeds = sorted(abs(v) for v in velocities)
        return (max(positions)-min(positions) <= START_STILL_MAX_EXCURSION_RAD
                and abs(sum(velocities)/len(velocities)) <= settings["arrival_speed_rad_s"]
                # Arrival stability only: this is not an active overspeed stop.
                and absolute_speeds[int(.95*(len(absolute_speeds)-1))] <= START_STILL_GROSS_SPEED_RAD_S)

    def snapshot(self) -> dict:
        now = self.clock()
        with self.state_lock:
            joints = {}
            for key in JOINT_KEYS:
                cfg = self._joint_config(key)
                runtime = self.runtimes[key]
                age = None if runtime.last_feedback_at is None else now - runtime.last_feedback_at
                joints[key] = {
                    "key": key,
                    "display_name": cfg["display_name"],
                    "motor_can_id": int(cfg["motor_can_id"]),
                    "expected_uid": str(cfg["expected_uid"]).upper(),
                    "observed_uid": runtime.observed_uid,
                    "verified": runtime.verified,
                    "commissioned": bool(cfg.get("commissioned")),
                    "limits_confirmed": bool(cfg.get("limits_confirmed")),
                    "direction": int(cfg["direction"]),
                    "software_zero_rad": cfg.get("software_zero_rad"),
                    "actual_raw_rad": runtime.actual_raw_rad,
                    "actual_deg": runtime.actual_deg,
                    "target_deg": runtime.target_deg,
                    "setpoint_deg": runtime.setpoint_deg,
                    "velocity_rad_s": runtime.velocity_rad_s,
                    "torque_nm": runtime.torque_nm,
                    "temperature_c": runtime.temperature_c,
                    "fault_bits": runtime.fault_bits,
                    "mode_status": runtime.mode_status,
                    "feedback_age_s": age,
                    "phase": runtime.phase,
                    "on_target": runtime.on_target and age is not None and 0 <= age <= ACTIVE_FEEDBACK_TIMEOUT_S,
                    "arrival_stable": self._arrival_is_stable(runtime, policy(self.config), now),
                    "error_deg": None if runtime.target_deg is None or runtime.actual_deg is None else runtime.target_deg-runtime.actual_deg,
                    "gravity_torque_nm": runtime.gravity_torque_nm,
                    "load_bias_nm": runtime.load_bias_nm,
                    "notice": runtime.notice,
                    "estimated_unclamped_torque_nm": runtime.estimated_command_torque_nm,
                    "sent_control": ({**runtime.last_sent_control,
                        "age_s": now-runtime.last_sent_control["sent_at"]}
                        if runtime.last_sent_control else None),
                    "next_motion_gains": dict(zip(("kp", "kd"), motion_gains(self.config, key))),
                    "diagnostics": {
                        spec[0]: {
                            "value": runtime.parameter_values.get(index),
                            "age_s": None if index not in runtime.parameter_times else now-runtime.parameter_times[index],
                            "fresh": index in runtime.parameter_times and now-runtime.parameter_times[index] <= 3.0,
                        }
                        for index, spec in PARAMETER_DIAGNOSTICS.items()
                    },
                    "limits": {
                        "minimum_angle_deg": float(cfg["minimum_angle_deg"]),
                        "maximum_angle_deg": float(cfg["maximum_angle_deg"]),
                        "maximum_speed_rad_s": float(cfg["maximum_speed_rad_s"]),
                        "maximum_torque_nm": float(cfg["maximum_torque_nm"]),
                    },
                    "control": {
                        "default_speed_rad_s": float(cfg["default_speed_rad_s"]),
                        "default_torque_limit_nm": float(cfg["default_torque_limit_nm"]),
                        "motion_kp": float(cfg["motion_kp"]),
                        "motion_kd": float(cfg["motion_kd"]),
                    },
                }
            return {
                "safety_latched": self.safety_latched,
                "safety_reason": self.safety_reason,
                "motion_active": self.motion_plan is not None,
                "control_phase": self._control_phase(),
                "gravity": {**copy.deepcopy(gravity_config(self.config)), "calibration_valid": calibration_matches(self.config)},
                "motion_policy": policy(self.config),
                "load_compensation": load_compensation_config(self.config),
                "joints": joints,
            }

    def _control_phase(self) -> str:
        if self.motion_plan is None:
            return "stopped"
        return "moving" if self.clock() < self.motion_plan.started_at + self.motion_plan.duration else "controlling"

    def history_snapshot(self, metric: str = "angle") -> dict[str, list[tuple[float, float | None]]]:
        if metric not in ("angle", "velocity", "torque", "temperature", "target", "error", "gravity", "estimated_torque", "iq", "vbus"):
            raise ValueError("未知曲线指标")
        with self.state_lock:
            return {key: list(self.runtimes[key].history[metric]) for key in JOINT_KEYS}
