"""Local dual-joint digital twin backed by the shared CyberGear safety controller."""

from __future__ import annotations

import ctypes
import json
import mimetypes
import os
import queue
import secrets
import shutil
import copy
import threading
import time
import webbrowser
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import serial
from serial.tools import list_ports

from classic_dual_controller import ClassicConfigStore, DualMotorController, JOINT_KEYS
from cybergear_protocol import encode_slcan_extended
from motion_policy import calibration_matches, gravity_config, load_compensation_config, policy
from backend_lifecycle import BackendRegistration, retire_existing_backend, startup_lock, startup_log
from dataset_recorder import DatasetRecorder
from hand_device import HandDevice


APP_ROOT = Path(__file__).resolve().parent
WEB_ROOT = APP_ROOT / "web"
CONFIG_PATH = APP_ROOT / "classic_dual_config.json"
HOST = "127.0.0.1"
PORT = 8765
_INSTANCE_MUTEX = None
_CONSOLE_HANDLER = None


def acquire_single_instance() -> bool:
    global _INSTANCE_MUTEX
    if os.name != "nt":
        return True
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateMutexW.argtypes = (ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p)
    kernel32.CreateMutexW.restype = ctypes.c_void_p
    kernel32.CloseHandle.argtypes = (ctypes.c_void_p,)
    kernel32.CloseHandle.restype = ctypes.c_bool
    handle = kernel32.CreateMutexW(None, False, "Local\\CyberArmDigitalTwin-8765")
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    if ctypes.get_last_error() == 183:
        kernel32.CloseHandle(handle)
        return False
    _INSTANCE_MUTEX = handle
    return True


def release_single_instance() -> None:
    global _INSTANCE_MUTEX
    if _INSTANCE_MUTEX is not None:
        kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel32.CloseHandle.argtypes = (ctypes.c_void_p,)
        kernel32.CloseHandle(_INSTANCE_MUTEX)
        _INSTANCE_MUTEX = None


def install_console_cleanup() -> None:
    """A console close otherwise skips Python's finally blocks on Windows."""
    global _CONSOLE_HANDLER
    if os.name != 'nt':
        return
    handler_type = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_uint32)

    def cleanup(event):
        if event in (1, 2, 5, 6):
            try:
                TWIN.shutdown()
            except Exception:
                pass
        return False  # Preserve normal Ctrl+C/console-exit behavior.

    _CONSOLE_HANDLER = handler_type(cleanup)
    kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel32.SetConsoleCtrlHandler.argtypes = (handler_type, ctypes.c_bool)
    if not kernel32.SetConsoleCtrlHandler(_CONSOLE_HANDLER, True):
        raise ctypes.WinError(ctypes.get_last_error())


def remove_console_cleanup() -> None:
    global _CONSOLE_HANDLER
    if _CONSOLE_HANDLER is not None:
        kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel32.SetConsoleCtrlHandler.argtypes = (type(_CONSOLE_HANDLER), ctypes.c_bool)
        kernel32.SetConsoleCtrlHandler(_CONSOLE_HANDLER, False)
        _CONSOLE_HANDLER = None


class SlcanLink:
    """One serial owner and one receive queue for both motor IDs."""

    def __init__(self, port: str, baudrate: int = 921600) -> None:
        self.port = port
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.reader: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    def connect(self) -> None:
        self.close()
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=1)
            # CANable's documented default: retry CAN arbitration/transmit
            # failures. A0 (single shot) can lose lower-priority read requests
            # during the dual-motor control/feedback stream.
            for command in (b"C\r", b"S8\r", b"M0\r", b"A1\r", b"O\r"):
                self._write(command)
            self.stop_event.clear()
            self.reader = threading.Thread(target=self._read_loop, name="twin-slcan-rx", daemon=True)
            self.reader.start()
        except Exception:
            self.close()
            raise

    def _write(self, data: bytes) -> None:
        if not self.connected:
            raise RuntimeError("CANable 未连接")
        assert self.serial is not None
        with self.write_lock:
            self.serial.write(data)
            self.serial.flush()

    def send(self, can_id: int, payload: bytes) -> None:
        self._write(encode_slcan_extended(can_id, payload))

    def _read_loop(self) -> None:
        buffer = ""
        while not self.stop_event.is_set() and self.serial:
            try:
                chunk = self.serial.read(self.serial.in_waiting or 1).decode("ascii", errors="ignore")
            except (serial.SerialException, OSError):
                break
            buffer += chunk
            while "\r" in buffer:
                line, buffer = buffer.split("\r", 1)
                if line:
                    self.rx_queue.put(line)
            if len(buffer) > 256:
                buffer = ""

    def close(self) -> None:
        self.stop_event.set()
        if self.reader and self.reader is not threading.current_thread():
            self.reader.join(timeout=0.35)
        if self.serial and self.serial.is_open:
            try:
                self._write(b"C\r")
            except (serial.SerialException, OSError, RuntimeError):
                pass
            self.serial.close()
        self.serial = None
        self.reader = None


class CyberArmTwin:
    """HTTP-facing adapter for the tested two-motor controller."""

    def __init__(self, config_path: Path = CONFIG_PATH, link_factory=SlcanLink) -> None:
        self.store = ClassicConfigStore(config_path)
        self.config = self.store.load()
        self.link_factory = link_factory
        self.port = str(self.config["bus"].get("port", "COM11"))
        self.lock = threading.RLock()
        self.lifecycle_lock = threading.RLock()
        self.link: SlcanLink | None = None
        self.controller: DualMotorController | None = None
        self.shutting_down = False
        self.status = "等待连接"
        self.logs: deque[dict] = deque(maxlen=200)
        self.recorder = DatasetRecorder(config_path.parent / "datasets", source_root=APP_ROOT)

    def _log(self, level: str, message: str) -> None:
        with self.lock:
            self.logs.append({"time": time.strftime("%H:%M:%S"), "level": level, "message": message})
        self.recorder.event(level, message)

    def _controller_log(self, message: str) -> None:
        level = "error" if "失败" in message or "锁存" in message else "warn" if "性能告警" in message else "info"
        self._log(level, message)

    def _save_recorded_config(self, config: dict) -> None:
        self.store.save(config)
        self.recorder.event("info", "配置已保存", configuration_json=json.dumps(config, ensure_ascii=False, allow_nan=False))

    def _require_controller(self) -> DualMotorController:
        controller = self.controller
        if controller is None or self.link is None or not self.link.connected:
            raise RuntimeError("请先连接 CANable")
        return controller

    def connect(self, port: str) -> dict[str, bool]:
        with self.lifecycle_lock:
            if self.controller is not None:
                raise RuntimeError("请先断开当前连接")
            if not port.strip():
                raise ValueError("请选择串口")
            # Reload configuration so calibration performed in the classic app is visible.
            self.config = self.store.load()
            link = self.link_factory(port, int(self.config["bus"]["serial_baud"]))
            controller: DualMotorController | None = None
            try:
                link.connect()
                controller = DualMotorController(
                    link, self.config, save_config=self._save_recorded_config, log=self._controller_log,
                    telemetry=self.recorder.append,
                )
                controller.start()
                identities = controller.discover()
                self.config["bus"]["port"] = port
                self.store.save(self.config)
            except Exception:
                try:
                    if controller is not None:
                        controller.shutdown()
                finally:
                    link.close()
                raise
            with self.lock:
                self.link = link
                self.controller = controller
                self.port = port
                self.status = (
                    "肩关节和肘关节 UID 验证通过"
                    if all(identities.values()) else "已连接；请检查未通过 UID 验证的关节"
                )
            self._log("ok" if all(identities.values()) else "warn", self.status)
            return identities

    def disconnect(self) -> None:
        with self.lifecycle_lock:
            with self.lock:
                controller, link = self.controller, self.link
                self.controller = None
                self.link = None
            try:
                if controller is not None:
                    controller.shutdown()
            finally:
                if link is not None:
                    link.close()
                self.recorder.finish("disconnect")
            self.status = "已断开；已尝试停止双机"
            self._log("warn", self.status)

    def discover(self) -> dict[str, bool]:
        result = self._require_controller().discover()
        self.status = "UID 重新识别完成"
        return result

    def start_dataset(self) -> dict:
        with self.lifecycle_lock:
            if self.shutting_down:
                raise RuntimeError("后台正在退出")
            controller = self._require_controller()
            with controller.state_lock:
                config = copy.deepcopy(controller.config)
        # Filesystem setup must not hold the motor-command/lifecycle lock.
        result = self.recorder.start(config)
        with self.lifecycle_lock:
            if controller is not self.controller or self.shutting_down:
                self.recorder.finish("connection_changed_during_start")
                raise RuntimeError("连接状态已变化，记录已结束；未操作电机")
        self.recorder.event("info", "开始数据记录；不使能或移动电机")
        return result

    def emergency_stop(self) -> dict[str, str]:
        controller = self.controller
        if controller is None:
            return {key: "disconnected" for key in JOINT_KEYS}
        result = controller.stop_all("数字孪生界面紧急停止")
        self.status = "已尝试向两台电机发送停止帧"
        return result

    def capture_zero(self, key: str) -> float:
        value = self._require_controller().capture_zero(key)
        self.status = f"{self.config['joints'][key]['display_name']} 软件零点已更新；需重新验收"
        return value

    def configure_joint(self, key: str, changes: dict) -> None:
        if key not in JOINT_KEYS:
            raise ValueError("未知关节")
        allowed = {
            "direction", "minimum_angle_deg", "maximum_angle_deg", "maximum_speed_rad_s",
            "maximum_torque_nm", "motion_kp", "motion_kd", "limits_confirmed",
        }
        if set(changes) != allowed:
            raise ValueError("关节安全配置字段不完整")
        values = {
            "direction": int(changes["direction"]),
            "minimum_angle_deg": float(changes["minimum_angle_deg"]),
            "maximum_angle_deg": float(changes["maximum_angle_deg"]),
            "maximum_speed_rad_s": float(changes["maximum_speed_rad_s"]),
            "maximum_torque_nm": float(changes["maximum_torque_nm"]),
            "motion_kp": float(changes["motion_kp"]),
            "motion_kd": float(changes["motion_kd"]),
            "limits_confirmed": changes["limits_confirmed"] is True,
        }
        controller = self.controller
        if controller is not None:
            controller.update_joint_config(key, **values)
        else:
            candidate = json.loads(json.dumps(self.config))
            candidate["joints"][key].update(values)
            candidate["joints"][key]["default_speed_rad_s"] = min(
                float(candidate["joints"][key]["default_speed_rad_s"]), values["maximum_speed_rad_s"]
            )
            candidate["joints"][key]["default_torque_limit_nm"] = min(
                float(candidate["joints"][key]["default_torque_limit_nm"]), values["maximum_torque_nm"]
            )
            candidate["joints"][key]["commissioned"] = False
            self.store.save(candidate)
            self.config = candidate
        self.status = f"{self.config['joints'][key]['display_name']} 配置已保存；验收已取消"

    def set_commissioned(self, key: str, value: bool) -> None:
        if key not in JOINT_KEYS:
            raise ValueError("未知关节")
        if not isinstance(value, bool):
            raise ValueError("验收状态必须是布尔值")
        self._require_controller().set_commissioned(key, value)
        self.status = f"{self.config['joints'][key]['display_name']} 验收状态已更新"

    def prepare_joints(self, body: dict) -> None:
        if body.get("confirmed") is not True:
            raise ValueError("请先核对方向、限位及机械空间")
        settings = body.get("configs")
        if not isinstance(settings, dict):
            raise ValueError("关节配置无效")
        self._require_controller().prepare_joints(settings)
        self.status = "已确认安全配置，可进行低速试运行"

    def move(self, body: dict) -> float:
        targets = body.get("targets_deg", {})
        if not isinstance(targets, dict) or not targets or set(targets) - set(JOINT_KEYS):
            raise ValueError("请为肘关节、肩关节或两者提供目标角度")
        keys = tuple(targets)
        def per_joint(name: str) -> dict[str, float]:
            supplied = body.get(name, {})
            if not isinstance(supplied, dict) or set(supplied) - set(keys):
                raise ValueError(f"{name} 关节字段无效")
            return {key: float(value) for key, value in supplied.items()}
        duration = self._require_controller().start_motion(
            {key: float(value) for key, value in targets.items()},
            speed_rad_s=per_joint("speeds_rad_s"),
            torque_limit_nm=per_joint("torque_limits_nm"),
            kp=per_joint("kp"),
            kd=per_joint("kd"),
        )
        self.status = "双关节协调运动已启动" if len(keys) == 2 else f"{self.config['joints'][keys[0]]['display_name']} 运动已启动"
        return duration

    def reset_safety(self) -> None:
        self._require_controller().reset_safety()
        self.status = "安全锁存已人工复位"

    def configure_gravity(self, body: dict) -> None:
        allowed = {"enabled", "confirmed", "shoulder_reference_deg", "forearm_reference_deg", "shoulder_angle_sign", "elbow_angle_sign", "vertical_zero_confirmed"}
        if not isinstance(body, dict) or set(body)-allowed or "enabled" not in body:
            raise ValueError("重力补偿配置字段无效")
        self._require_controller().configure_gravity(**body)
        self.status = "空载补偿已标定；下次手动运动时启用" if body["enabled"] else "空载补偿关闭"

    def configure_load_compensation(self, body: dict) -> None:
        if not isinstance(body, dict) or set(body)-{"enabled", "confirmed", "profile", "parameters"} or "enabled" not in body:
            raise ValueError("负载误差补偿字段无效")
        if "parameters" in body and not isinstance(body["parameters"], dict):
            raise ValueError("运控参数必须为对象")
        self._require_controller().configure_load_compensation(**body)
        self.status = "负载误差补偿设置已保存（试验）；不会自动运动"

    def snapshot(self) -> dict:
        # Controller callbacks can take self.lock while holding state_lock.
        # Never acquire those two locks in the opposite order on UI polling.
        controller = self.controller
        joint_state = controller.snapshot() if controller is not None else None
        with self.lock:
            if controller is not self.controller:
                joint_state = None
            joints = joint_state["joints"] if joint_state else {}
            if joint_state is None:
                for key in JOINT_KEYS:
                    cfg = self.config["joints"][key]
                    joints[key] = {
                        "key": key, "display_name": cfg["display_name"],
                        "motor_can_id": int(cfg["motor_can_id"]), "expected_uid": cfg["expected_uid"],
                        "observed_uid": None, "verified": False,
                        "commissioned": bool(cfg["commissioned"]),
                        "limits_confirmed": bool(cfg["limits_confirmed"]),
                        "direction": int(cfg["direction"]), "software_zero_rad": cfg["software_zero_rad"],
                        "actual_raw_rad": None, "actual_deg": None, "target_deg": None,
                        "setpoint_deg": None, "velocity_rad_s": None, "torque_nm": None,
                        "temperature_c": None, "fault_bits": 0, "mode_status": 0,
                        "feedback_age_s": None,
                        "phase": "stopped", "error_deg": None,
                        "gravity_torque_nm": 0.0, "load_bias_nm": 0.0, "notice": "", "estimated_unclamped_torque_nm": None,
                        "diagnostics": {}, "sent_control": None,
                        "limits": {
                            "minimum_angle_deg": float(cfg["minimum_angle_deg"]),
                            "maximum_angle_deg": float(cfg["maximum_angle_deg"]),
                            "maximum_speed_rad_s": float(cfg["maximum_speed_rad_s"]),
                            "maximum_torque_nm": float(cfg["maximum_torque_nm"]),
                        },
                        "control": {
                            "default_speed_rad_s": float(cfg["default_speed_rad_s"]),
                            "default_torque_limit_nm": float(cfg["default_torque_limit_nm"]),
                            "motion_kp": float(cfg["motion_kp"]), "motion_kd": float(cfg["motion_kd"]),
                        },
                    }
            return {
                "version": "2.3.3", "connected": bool(self.link and self.link.connected),
                "port": self.port, "ports": [item.device for item in list_ports.comports()],
                "status": self.status,
                "safety_latched": bool(joint_state and joint_state["safety_latched"]),
                "safety_reason": joint_state["safety_reason"] if joint_state else "",
                "motion_active": bool(joint_state and joint_state["motion_active"]),
                "control_phase": joint_state["control_phase"] if joint_state else "stopped",
                "gravity": joint_state["gravity"] if joint_state else {**gravity_config(self.config), "calibration_valid": calibration_matches(self.config)},
                "motion_policy": joint_state["motion_policy"] if joint_state else policy(self.config),
                "load_compensation": joint_state["load_compensation"] if joint_state else load_compensation_config(self.config),
                "joints": joints, "logs": list(self.logs)[-40:],
                "dataset": self.recorder.status(),
            }

    def history_snapshot(self) -> dict:
        controller = self.controller
        if controller is None:
            return {key: {metric: [] for metric in ("angle", "velocity", "torque", "temperature")} for key in JOINT_KEYS}
        metrics = {metric: controller.history_snapshot(metric) for metric in ("angle", "velocity", "torque", "temperature", "target", "error", "gravity", "estimated_torque", "iq", "vbus")}
        return {key: {metric: metrics[metric][key] for metric in metrics} for key in JOINT_KEYS}

    def shutdown(self) -> None:
        try:
            self.disconnect()
        finally:
            self.recorder.finish("backend_shutdown")
            self.recorder.wait()

    def prepare_backend_exit(self) -> None:
        with self.lifecycle_lock:
            controller = self.controller
            if controller is not None:
                with controller.state_lock:
                    if controller.motion_plan is not None:
                        raise RuntimeError('运动或保持中，不自动退出后台')
            self.shutting_down = True
            try:
                self.disconnect()
            except Exception:
                self.shutting_down = False
                raise


TWIN = CyberArmTwin()
HAND = HandDevice()


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "CyberArmTwin/2.0"

    def _local_request(self) -> bool:
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        return host in {f"{HOST}:{PORT}", f"localhost:{PORT}"} and (
            origin is None or origin in {f"http://{HOST}:{PORT}", f"http://localhost:{PORT}"}
        )

    def log_message(self, _format: str, *_args) -> None:
        return

    def _json(self, data, status: HTTPStatus = HTTPStatus.OK) -> None:
        encoded = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(encoded)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            pass

    def _body(self) -> dict:
        length = min(max(int(self.headers.get("Content-Length", "0")), 0), 65536)
        if not length:
            return {}
        data = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("请求内容必须是 JSON 对象")
        return data

    def do_GET(self) -> None:
        if not self._local_request():
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        path = urlparse(self.path).path
        if path.startswith('/api/dataset/download/'):
            try:
                archive = TWIN.recorder.archive(path.removeprefix('/api/dataset/download/'))
                with archive.open('rb') as handle:
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'application/zip')
                    self.send_header('Content-Disposition', f'attachment; filename="cyberarm_{archive.name}"')
                    self.send_header('Content-Length', str(archive.stat().st_size))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    shutil.copyfileobj(handle, self.wfile, length=1024*1024)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                pass
            except (ValueError, OSError) as exc:
                self._json({'ok':False, 'error':str(exc)}, HTTPStatus.BAD_REQUEST)
            except RuntimeError as exc:
                self._json({'ok':False, 'error':str(exc)}, HTTPStatus.CONFLICT)
            return
        if path == '/api/backend':
            registration = getattr(self.server, 'backend_registration', None)
            if registration is None:
                self.send_error(HTTPStatus.NOT_FOUND)
            else:
                self._json(registration.identity)
            return
        if path == "/api/state":
            self._json(TWIN.snapshot())
            return
        if path == "/api/hand/state":
            self._json(HAND.snapshot())
            return
        if path == "/api/history":
            self._json(TWIN.history_snapshot())
            return
        if path == "/api/diagnostics":
            self._json(TWIN.controller.diagnostics_snapshot() if TWIN.controller else {"trace": [], "pending": []})
            return
        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        relative = "hand-control/index.html" if path == "/hand-control/" else "index.html" if path == "/" else path.lstrip("/")
        candidate = (WEB_ROOT / relative).resolve()
        try:
            candidate.relative_to(WEB_ROOT.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content = candidate.read_bytes()
        mime = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            pass

    def do_POST(self) -> None:
        if not self._local_request():
            self._json({"ok": False, "error": "只允许本地页面发起控制请求"}, HTTPStatus.FORBIDDEN)
            return
        path = urlparse(self.path).path
        try:
            body = self._body()
            if path in ('/api/hand/connect', '/api/hand/disconnect', '/api/hand/command'):
                # Hand I/O never holds the arm lifecycle/stop lock.
                if path.endswith('/command'):
                    state = HAND.command(body)
                else:
                    if body:
                        raise ValueError('手部连接接口不接受额外参数')
                    state = HAND.connect() if path.endswith('/connect') else HAND.disconnect()
                self._json({'ok': True, 'state': state})
                return
            if path in ('/api/dataset/start', '/api/dataset/finish'):
                if body:
                    raise ValueError('数据记录接口不接受额外参数')
                # No blocking disk work behind the lock used by /api/stop.
                result = TWIN.start_dataset() if path.endswith('/start') else TWIN.recorder.finish('user')
                self._json({'ok':True, 'result':result, 'state':TWIN.snapshot()})
                return
            # Serialize takeover against all HTTP commands: nothing may
            # reconnect or start motion after the old instance accepts exit.
            with TWIN.lifecycle_lock:
                if path == '/api/backend/shutdown':
                    registration = getattr(self.server, 'backend_registration', None)
                    token = body.get('token')
                    if registration is None or set(body) != {'token'} or not isinstance(token, str) or not secrets.compare_digest(token, registration.token):
                        self._json({'ok': False, 'error': 'Unauthorized backend shutdown'}, HTTPStatus.FORBIDDEN)
                        return
                    with HAND.backend_exit():
                        TWIN.prepare_backend_exit()
                    self._json({'ok': True})
                    threading.Thread(target=self.server.shutdown, daemon=True).start()
                    return
                if TWIN.shutting_down:
                    raise RuntimeError('Backend is shutting down')
                if path not in {'/api/connect', '/api/disconnect', '/api/discover', '/api/stop', '/api/zero',
                                '/api/configure', '/api/commission', '/api/prepare', '/api/move',
                                '/api/reset-safety', '/api/load-compensation', '/api/gravity'}:
                    self._json({'ok': False, 'error': '未知接口'}, HTTPStatus.NOT_FOUND)
                    return
                result = self._dispatch_post(path, body)
                self._json({"ok": True, "result": result, "state": TWIN.snapshot()})
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self._json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except (RuntimeError, serial.SerialException, OSError) as exc:
            self._json({"ok": False, "error": str(exc)}, HTTPStatus.CONFLICT)
        except Exception as exc:
            TWIN._log("error", f"服务器异常：{exc}")
            self._json({"ok": False, "error": f"操作失败：{exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _dispatch_post(self, path: str, body: dict):
        if path == '/api/connect':
            return TWIN.connect(str(body.get('port', '')))
        if path == '/api/disconnect':
            return TWIN.disconnect()
        if path == '/api/discover':
            return TWIN.discover()
        if path == '/api/stop':
            return TWIN.emergency_stop()
        if path == '/api/zero':
            return TWIN.capture_zero(str(body['joint']))
        if path == '/api/configure':
            return TWIN.configure_joint(str(body['joint']), body['config'])
        if path == '/api/commission':
            return TWIN.set_commissioned(str(body['joint']), body['value'])
        if path == '/api/prepare':
            return TWIN.prepare_joints(body)
        if path == '/api/move':
            return TWIN.move(body)
        if path == '/api/reset-safety':
            return TWIN.reset_safety()
        if path == '/api/load-compensation':
            return TWIN.configure_load_compensation(body)
        if path == '/api/gravity':
            return TWIN.configure_gravity(body)
        raise ValueError('未知接口')


def main() -> None:
    global TWIN, HAND
    url = f"http://{HOST}:{PORT}/"
    server = registration = None
    browser_timer = None
    try:
        with startup_lock():
            if not acquire_single_instance():
                retire_existing_backend(APP_ROOT, url.rstrip('/'), PORT)
                deadline = time.monotonic() + 8
                while not acquire_single_instance():
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Old backend did not release its instance lock')
                    time.sleep(0.1)
            # Reload persisted settings only AFTER the previous process exits.
            TWIN = CyberArmTwin()
            HAND = HandDevice()
            server = ThreadingHTTPServer((HOST, PORT), DashboardHandler)
            registration = BackendRegistration(APP_ROOT)
            server.backend_registration = registration
            registration.publish()
            install_console_cleanup()
        startup_log(APP_ROOT, 'Backend started from current source: ' + registration.identity['source_revision'])
        if os.environ.get('CYBERARM_NO_BROWSER') != '1':
            browser_timer = threading.Timer(0.7, lambda: webbrowser.open(url))
            browser_timer.start()
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # No popup, stale-backend page or version banner; details stay local.
        startup_log(APP_ROOT, f'Startup/exit failed: {type(exc).__name__}: {exc}')
    finally:
        if browser_timer is not None:
            browser_timer.cancel()
        # Always release our own records/handles even if one cleanup fails.
        cleanups = []
        if server is not None:
            cleanups.extend((TWIN.shutdown, HAND.disconnect, server.server_close))
        if registration is not None:
            cleanups.append(registration.cleanup)
        cleanups.extend((remove_console_cleanup, release_single_instance))
        for cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:
                startup_log(APP_ROOT, f'Cleanup failed: {exc}')


if __name__ == "__main__":
    main()
