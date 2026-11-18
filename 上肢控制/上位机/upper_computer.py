"""Classic dual-motor CyberGear upper computer.

This Tk application is intentionally independent from the browser-based
digital twin.  It owns one CANable/SLCAN connection and routes feedback to the
fixed logical roles elbow (127) and shoulder (126), verified by MCU UID.
"""

from __future__ import annotations

import math
import copy
import json
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("缺少 pyserial。请通过 run_classic.bat 启动。") from exc

from classic_dual_controller import (
    JOINT_KEYS,
    ClassicConfigStore,
    DualMotorController,
)
from cybergear_protocol import encode_slcan_extended
from dataset_recorder import DatasetRecorder


APP_ROOT = Path(__file__).resolve().parent
CONFIG_PATH = APP_ROOT / "classic_dual_config.json"
JOINT_COLORS = {"elbow": "#41c4ff", "shoulder": "#ffb454"}


class SlcanTransport:
    """The single owner of the CANable serial port."""

    def __init__(self, port: str, baudrate: int = 921600) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.stop_event = threading.Event()
        self.reader: threading.Thread | None = None
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    def connect(self) -> None:
        self.close()
        device = serial.Serial(self.port_name, self.baudrate, timeout=0.1, write_timeout=1)
        self.serial = device
        # Match the digital twin: retry CAN arbitration instead of single-shot loss.
        for command in (b"C\r", b"S8\r", b"M0\r", b"A1\r", b"O\r"):
            self._write_ascii(command)
            time.sleep(0.04)
        self.stop_event.clear()
        self.reader = threading.Thread(target=self._read_loop, name="classic-slcan-rx", daemon=True)
        self.reader.start()

    def send(self, can_id: int, payload: bytes) -> None:
        if not self.connected:
            raise RuntimeError("CANable 未连接")
        assert self.serial is not None
        with self.write_lock:
            self.serial.write(encode_slcan_extended(can_id, payload))
            self.serial.flush()

    def close(self) -> None:
        self.stop_event.set()
        if self.reader and self.reader is not threading.current_thread():
            self.reader.join(timeout=0.35)
        if self.serial and self.serial.is_open:
            try:
                self._write_ascii(b"C\r")
            except (serial.SerialException, OSError):
                pass
            self.serial.close()
        self.serial = None
        self.reader = None

    def _write_ascii(self, command: bytes) -> None:
        if not self.serial:
            raise RuntimeError("串口未打开")
        with self.write_lock:
            self.serial.write(command)
            self.serial.flush()

    def _read_loop(self) -> None:
        buffer = ""
        while not self.stop_event.is_set() and self.serial:
            try:
                chunk = self.serial.read(self.serial.in_waiting or 1).decode("ascii", errors="ignore")
            except (serial.SerialException, OSError):
                return
            if not chunk:
                continue
            buffer += chunk
            while "\r" in buffer:
                line, buffer = buffer.split("\r", 1)
                if line:
                    self.rx_queue.put(line)


class ClassicDualApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("CyberGear 双电机经典上位机 v2.3.3")
        self.geometry("1120x820")
        self.minsize(980, 720)

        self.store = ClassicConfigStore(CONFIG_PATH)
        self.config = self.store.load()
        self.recorder = DatasetRecorder(APP_ROOT / "datasets", source_root=APP_ROOT)
        self.transport: SlcanTransport | None = None
        self.controller: DualMotorController | None = None
        self.ui_events: queue.Queue[tuple] = queue.Queue()
        self.closing = False
        self.connect_cancel = threading.Event()

        self.port_var = tk.StringVar(value=str(self.config["bus"].get("port", "COM11")))
        self.status_var = tk.StringVar(value="未连接：两台电机保持禁止运动")
        self.safety_var = tk.StringVar(value="安全状态：SAFE / 未连接")
        self.overview_vars: dict[str, dict[str, tk.StringVar]] = {}
        self.joint_vars: dict[str, dict[str, tk.Variable]] = {}
        self.commission_buttons: dict[str, ttk.Button] = {}
        self.metric_var = tk.StringVar(value="angle")
        self.chart_info_var = tk.StringVar(value="等待双路反馈")
        self.dataset_status_var = tk.StringVar(value="未开始记录")

        self._build_ui()
        self._refresh_ports()
        self.after(100, self._poll_ui)
        self.after(180, self._draw_chart)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.LabelFrame(self, text="单 CANable / 双 CyberGear（1 Mbps / 29 位扩展帧）", padding=8)
        top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="串口").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=13, state="readonly")
        self.port_box.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(top, text="连接并验证两台电机", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=8)
        ttk.Button(top, text="重新识别 UID", command=self._discover).grid(row=0, column=4, padx=4)
        ttk.Button(top, text="复位安全锁存", command=self._reset_safety).grid(row=0, column=5, padx=4)
        tk.Button(
            top,
            text="全局紧急停止",
            command=self._global_stop,
            bg="#c62828",
            fg="white",
            font=("Microsoft YaHei", 11, "bold"),
        ).grid(row=0, column=6, padx=(20, 4))

        status = ttk.Frame(self, padding=(12, 0, 12, 6))
        status.pack(fill="x")
        ttk.Label(status, textvariable=self.status_var).pack(side="left")
        ttk.Label(status, textvariable=self.safety_var, foreground="#b00020").pack(side="right")

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        self._build_overview_tab(notebook)
        for key in JOINT_KEYS:
            self._build_joint_tab(notebook, key)
        self._build_coordinated_tab(notebook)
        self._build_gravity_tab(notebook)
        self._build_chart_tab(notebook)
        self._build_log_tab(notebook)
        self._build_dataset_tab(notebook)

    def _build_dataset_tab(self, notebook: ttk.Notebook) -> None:
        page = ttk.Frame(notebook, padding=16)
        notebook.add(page, text="数据集输出")
        ttk.Label(page, textvariable=self.dataset_status_var, wraplength=900).pack(anchor="w", pady=8)
        ttk.Label(page, text="逐帧反馈、成功下发指令、参数读回和事件日志分别保存为 CSV，附 UTC 时间戳、字段字典和配置快照。\n开始/结束记录都不会使能、移动或停止电机。", wraplength=900).pack(anchor="w", pady=8)
        buttons = ttk.Frame(page)
        buttons.pack(anchor="w", pady=8)
        self.dataset_start_button = ttk.Button(buttons, text="开始记录", command=self._start_dataset)
        self.dataset_start_button.pack(side="left", padx=4)
        self.dataset_finish_button = ttk.Button(buttons, text="结束记录", command=lambda: self.recorder.finish("user"))
        self.dataset_finish_button.pack(side="left", padx=4)
        self.dataset_export_button = ttk.Button(buttons, text="导出 ZIP", command=self._export_dataset)
        self.dataset_export_button.pack(side="left", padx=4)

    def _start_dataset(self) -> None:
        controller = self._require_controller()
        if controller is None:
            return
        def action():
            with controller.state_lock:
                config = copy.deepcopy(controller.config)
            result = self.recorder.start(config, interface="classic")
            if controller is not self.controller or self.closing:
                self.recorder.finish("connection_changed_during_start")
                raise RuntimeError("连接状态已变化，记录已结束")
            self.recorder.event("info", "开始数据记录；不使能或移动电机")
            return result
        self._run_async(action, lambda _result: None, "开始记录失败")

    def _export_dataset(self) -> None:
        session_id = self.recorder.status()["session_id"]
        self._run_async(lambda: self.recorder.archive(session_id),
            lambda path: messagebox.showinfo("数据集已导出", str(path)), "数据集导出失败")

    def _save_recorded_config(self, config: dict) -> None:
        self.store.save(config)
        self.recorder.event("info", "配置已保存", configuration_json=json.dumps(config, ensure_ascii=False, allow_nan=False))

    def _controller_log(self, text: str) -> None:
        self.recorder.event("info", text)
        self.ui_events.put(("log", text))

    def _build_overview_tab(self, notebook: ttk.Notebook) -> None:
        page = ttk.Frame(notebook, padding=12)
        notebook.add(page, text="双机总览")
        ttk.Label(
            page,
            text="电机角色固定：肘关节 ID 127，肩/大臂 ID 126；必须同时通过 UID 验证。",
            foreground="#444",
        ).grid(row=0, column=0, columnspan=9, sticky="w", pady=(0, 12))
        headers = ("关节", "CAN ID", "UID 验证", "验收", "角度", "速度", "转矩", "温度", "反馈龄")
        for column, text in enumerate(headers):
            ttk.Label(page, text=text, font=("Microsoft YaHei", 9, "bold")).grid(
                row=1, column=column, padx=8, pady=4, sticky="w"
            )
        for row, key in enumerate(JOINT_KEYS, start=2):
            cfg = self.config["joints"][key]
            values = {
                "name": tk.StringVar(value=str(cfg["display_name"])),
                "id": tk.StringVar(value=str(cfg["motor_can_id"])),
                "uid": tk.StringVar(value="未连接"),
                "commissioned": tk.StringVar(value="未验收"),
                "angle": tk.StringVar(value="--"),
                "velocity": tk.StringVar(value="--"),
                "torque": tk.StringVar(value="--"),
                "temperature": tk.StringVar(value="--"),
                "age": tk.StringVar(value="--"),
            }
            self.overview_vars[key] = values
            for column, field in enumerate(("name", "id", "uid", "commissioned", "angle", "velocity", "torque", "temperature", "age")):
                ttk.Label(page, textvariable=values[field]).grid(row=row, column=column, padx=8, pady=8, sticky="w")
        warning = (
            "经典版使用独立配置 classic_dual_config.json。现有两份历史软件零点仅作为候选保存，"
            "不会自动用于运动；两个关节都需重新采零、确认方向和限位后再验收。"
        )
        ttk.Label(page, text=warning, foreground="#a45b00", wraplength=900).grid(
            row=5, column=0, columnspan=9, sticky="w", pady=(24, 0)
        )

    def _build_joint_tab(self, notebook: ttk.Notebook, key: str) -> None:
        cfg = self.config["joints"][key]
        page = ttk.Frame(notebook, padding=12)
        notebook.add(page, text=f"{cfg['display_name']} · ID {cfg['motor_can_id']}")
        vars_: dict[str, tk.Variable] = {
            "identity": tk.StringVar(value=f"期望 UID {cfg['expected_uid']} / 未连接"),
            "telemetry": tk.StringVar(value="尚无反馈"),
            "zero": tk.StringVar(value="未设置（禁止运动）"),
            "direction": tk.IntVar(value=int(cfg["direction"])),
            "minimum": tk.DoubleVar(value=float(cfg["minimum_angle_deg"])),
            "maximum": tk.DoubleVar(value=float(cfg["maximum_angle_deg"])),
            "max_speed": tk.DoubleVar(value=float(cfg["maximum_speed_rad_s"])),
            "max_torque": tk.DoubleVar(value=float(cfg["maximum_torque_nm"])),
            "kp": tk.DoubleVar(value=float(cfg["motion_kp"])),
            "kd": tk.DoubleVar(value=float(cfg["motion_kd"])),
            "limits_confirmed": tk.BooleanVar(value=bool(cfg["limits_confirmed"])),
            "target": tk.DoubleVar(value=0.0),
            "speed": tk.DoubleVar(value=float(cfg["default_speed_rad_s"])),
            "torque": tk.DoubleVar(value=float(cfg["default_torque_limit_nm"])),
        }
        self.joint_vars[key] = vars_

        ttk.Label(page, textvariable=vars_["identity"], foreground="#555").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 4)
        )
        ttk.Label(page, textvariable=vars_["telemetry"]).grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 12))

        safety = ttk.LabelFrame(page, text="该关节安全配置（保存后自动取消验收）", padding=10)
        safety.grid(row=2, column=0, columnspan=4, sticky="nsew", pady=6)
        fields = (
            ("方向系数 (+1/-1)", "direction"),
            ("最小角度 (°)", "minimum"),
            ("最大角度 (°)", "maximum"),
            ("最大速度 (rad/s，≤0.30)", "max_speed"),
            ("最大转矩 (N·m，≤5.0)", "max_torque"),
            ("Kp (≤20)", "kp"),
            ("Kd (≤5)", "kd"),
        )
        for index, (label, field) in enumerate(fields):
            row, column = divmod(index, 2)
            ttk.Label(safety, text=label).grid(row=row, column=column * 2, sticky="w", padx=(0, 6), pady=4)
            ttk.Entry(safety, textvariable=vars_[field], width=12).grid(row=row, column=column * 2 + 1, sticky="w", padx=(0, 18), pady=4)
        ttk.Checkbutton(
            safety,
            text="已根据真实机械装配确认方向、软限位和运动空间",
            variable=vars_["limits_confirmed"],
        ).grid(row=4, column=0, columnspan=4, sticky="w", pady=6)
        ttk.Button(safety, text="保存安全配置", command=lambda joint=key: self._save_joint_config(joint)).grid(
            row=5, column=0, pady=8, sticky="w"
        )

        calibration = ttk.LabelFrame(page, text="软件零点与验收", padding=10)
        calibration.grid(row=3, column=0, columnspan=4, sticky="ew", pady=6)
        ttk.Label(calibration, text="软件零点").grid(row=0, column=0, sticky="w")
        ttk.Label(calibration, textvariable=vars_["zero"]).grid(row=0, column=1, sticky="w", padx=8)
        ttk.Button(
            calibration,
            text="停止两台并采集本关节零点",
            command=lambda joint=key: self._capture_zero(joint),
        ).grid(row=1, column=0, pady=8, sticky="w")
        button = ttk.Button(
            calibration,
            text="标记本关节已验收",
            command=lambda joint=key: self._toggle_commissioned(joint),
        )
        button.grid(row=1, column=1, pady=8, padx=8, sticky="w")
        self.commission_buttons[key] = button

        motion = ttk.LabelFrame(page, text="独立关节五次曲线运动", padding=10)
        motion.grid(row=4, column=0, columnspan=4, sticky="ew", pady=6)
        for column, (label, field) in enumerate(
            (("目标角度 °", "target"), ("速度 rad/s", "speed"), ("转矩限制 N·m", "torque"))
        ):
            ttk.Label(motion, text=label).grid(row=0, column=column * 2, sticky="w", padx=(0, 4))
            ttk.Entry(motion, textvariable=vars_[field], width=10).grid(row=0, column=column * 2 + 1, sticky="w", padx=(0, 14))
        ttk.Button(motion, text="确认后启动本关节", command=lambda joint=key: self._move_joint(joint)).grid(
            row=1, column=0, pady=10, sticky="w"
        )
        ttk.Button(motion, text="全局停止两台", command=self._global_stop).grid(row=1, column=1, pady=10, sticky="w")

    def _build_coordinated_tab(self, notebook: ttk.Notebook) -> None:
        page = ttk.Frame(notebook, padding=12)
        notebook.add(page, text="双关节协调运动")
        ttk.Label(
            page,
            text="两关节共用起始时刻和轨迹时长；任一关节异常都会停止两台。两关节均验收后才允许启动。",
            foreground="#444",
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 12))
        self.sync_target_vars: dict[str, tk.DoubleVar] = {}
        for row, key in enumerate(JOINT_KEYS, start=1):
            cfg = self.config["joints"][key]
            target = tk.DoubleVar(value=0.0)
            self.sync_target_vars[key] = target
            ttk.Label(page, text=f"{cfg['display_name']}目标角度 (°)", width=24).grid(row=row, column=0, sticky="w", pady=6)
            ttk.Entry(page, textvariable=target, width=12).grid(row=row, column=1, sticky="w", pady=6)
            ttk.Label(page, text="速度和转矩使用各关节页当前输入值").grid(row=row, column=2, sticky="w", padx=12)
        ttk.Button(page, text="确认后启动双关节同步轨迹", command=self._move_coordinated).grid(
            row=4, column=0, pady=18, sticky="w"
        )
        ttk.Button(page, text="全局停止两台", command=self._global_stop).grid(row=4, column=1, pady=18, sticky="w")

    def _build_gravity_tab(self, notebook: ttk.Notebook) -> None:
        page = ttk.Frame(notebook, padding=12)
        notebook.add(page, text="空载补偿标定")
        self.gravity_state_var = tk.StringVar(value="补偿关闭 / 未标定；不会自动运动")
        ttk.Label(page, textvariable=self.gravity_state_var).grid(row=0, column=0, columnspan=4, sticky="w", pady=8)
        ttk.Label(page, wraplength=850, text=(
            "机构须有支撑、无人佩戴。仅适用肩肘转轴平行且水平的抬臂装配。"
            "从同一固定侧观察，水平向右为 0°、向上 +90°、向下 -90°。"
            "下方输入当前大臂、小臂各自相对水平的绝对角度，不是软件目标角或肘的相对角。"
            "补偿只包含已核算的空载结构，尚未包含人臂。"
        )).grid(row=1, column=0, columnspan=4, sticky="w", pady=12)
        self.gravity_vars = {
            "shoulder_reference_deg": tk.StringVar(), "forearm_reference_deg": tk.StringVar(),
            "shoulder_angle_sign": tk.IntVar(value=1), "elbow_angle_sign": tk.IntVar(value=1),
        }
        for row, (name, label) in enumerate((
            ("shoulder_reference_deg", "当前大臂相对水平角度 (°)"),
            ("forearm_reference_deg", "当前小臂相对水平角度 (°)"),
            ("shoulder_angle_sign", "肩软件角增大：逆时针 +1 / 顺时针 -1"),
            ("elbow_angle_sign", "肘软件角增大：相对逆时针 +1 / 顺时针 -1"),
        ), start=2):
            ttk.Label(page, text=label).grid(row=row, column=0, sticky="w", pady=8)
            ttk.Entry(page, textvariable=self.gravity_vars[name], width=15).grid(row=row, column=1, padx=12)
        ttk.Button(page, text="确认真实姿态并标定补偿（不运动）", command=lambda: self._configure_gravity(True)).grid(row=7, column=0, pady=16, sticky="w")
        ttk.Button(page, text="关闭补偿（先支撑并停止）", command=lambda: self._configure_gravity(False)).grid(row=7, column=1, pady=16, sticky="w")
        self.load_state_var = tk.StringVar(value="负载误差补偿：默认关闭（试验功能）")
        ttk.Label(page, textvariable=self.load_state_var).grid(row=8, column=0, columnspan=4, sticky="w", pady=8)
        ttk.Label(page, wraplength=850, text="承托档：Kp=12、Kd=1.5、Ki=2；缓慢跟随与保持时补偿，最多 ±2.5 Nm、变化率 0.4 Nm/s，换目标保留支撑。总力矩上限不变。持续外力可能被视为负载，尚未验证人体负载。").grid(row=9, column=0, columnspan=4, sticky="w")
        ttk.Button(page, text="启用/升级负载承托档（试验）", command=lambda: self._configure_load_compensation(True)).grid(row=10, column=0, pady=12, sticky="w")
        ttk.Button(page, text="关闭负载误差补偿", command=lambda: self._configure_load_compensation(False)).grid(row=10, column=1, pady=12, sticky="w")

    def _configure_load_compensation(self, enabled: bool) -> None:
        controller = self._require_controller()
        if not controller:
            return
        if controller.snapshot()["motion_active"]:
            messagebox.showwarning("请先停止", "先支撑机构并停止双机，再修改负载误差补偿。")
            return
        if enabled and not messagebox.askokcancel("开启试验承托档", "确认机构已支撑、无人佩戴并已停止。下次手动启动采用 Kp=12、Kd=1.5（比原 Kp=3 更硬），Ki=2，附加补偿最多 ±2.5 Nm、0.4 Nm/s。总力矩上限不变，不会自动运动。持续外力可能被视为负载，尚未验证人臂承托，须先低速、小幅验证。"):
            return
        self._run_async(lambda: controller.configure_load_compensation(enabled=enabled, confirmed=enabled,
                        **({"profile": "tracking_support"} if enabled else {})),
                        lambda _result: self.status_var.set("负载误差补偿设置已保存；不会自动运动"), "补偿设置失败")

    def _configure_gravity(self, enabled: bool) -> None:
        controller = self._require_controller()
        if not controller:
            return
        if controller.snapshot()["motion_active"]:
            messagebox.showwarning("请先停止", "先支撑机构并停止双机，再修改补偿。")
            return
        values = {"enabled": enabled}
        if enabled:
            try:
                values.update({name: int(var.get()) if name.endswith("sign") else float(var.get()) for name, var in self.gravity_vars.items()})
            except (ValueError, tk.TclError):
                messagebox.showerror("姿态无效", "请输入实际物理角度和 +1/-1 方向，不能留空。")
                return
            if not messagebox.askokcancel("确认物理标定", "确认机构已支撑、无人穿戴，转轴平行且水平，并已核实输入的物理角度和正方向。方向填错可能施加反向力矩。本操作仅标定，不会运动。"):
                return
            values["confirmed"] = True
        self._run_async(lambda: controller.configure_gravity(**values),
                        lambda _result: self.status_var.set("补偿配置已保存；不会自动运动"), "补偿标定失败")

    def _build_chart_tab(self, notebook: ttk.Notebook) -> None:
        page = ttk.Frame(notebook, padding=10)
        notebook.add(page, text="双路曲线")
        tools = ttk.Frame(page)
        tools.pack(fill="x", pady=4)
        ttk.Label(tools, text="指标").pack(side="left")
        ttk.Combobox(
            tools,
            textvariable=self.metric_var,
            state="readonly",
            values=("angle", "velocity", "torque", "temperature"),
            width=14,
        ).pack(side="left", padx=6)
        ttk.Label(tools, text="蓝色=肘关节，橙色=肩/大臂").pack(side="left", padx=12)
        ttk.Label(tools, textvariable=self.chart_info_var).pack(side="right")
        self.chart = tk.Canvas(page, bg="#10151c", highlightthickness=0, height=500)
        self.chart.pack(fill="both", expand=True)

    def _build_log_tab(self, notebook: ttk.Notebook) -> None:
        page = ttk.Frame(notebook, padding=8)
        notebook.add(page, text="通信与安全日志")
        self.log = tk.Text(page, font=("Consolas", 9), state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True)

    def _refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_box["values"] = ports
        if self.port_var.get() not in ports and ports:
            preferred = "COM11" if "COM11" in ports else ports[0]
            self.port_var.set(preferred)

    def _toggle_connection(self) -> None:
        if self.controller:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("未选择串口", "请选择 CANable 串口。")
            return
        self.connect_cancel.clear()
        self.connect_button.configure(state="disabled")
        self.status_var.set(f"正在连接 {port} 并验证 126/127…")

        def action() -> dict[str, bool]:
            transport = SlcanTransport(port)
            controller: DualMotorController | None = None
            try:
                transport.connect()
                if self.connect_cancel.is_set():
                    raise RuntimeError("连接已取消")
                self.config["bus"]["port"] = port
                self.store.save(self.config)
                controller = DualMotorController(
                    transport,
                    self.config,
                    save_config=self._save_recorded_config,
                    log=self._controller_log,
                    telemetry=self.recorder.append,
                )
                controller.start()
                result = controller.discover()
                if self.connect_cancel.is_set():
                    raise RuntimeError("连接已取消")
            except Exception:
                if controller:
                    controller.shutdown()
                transport.close()
                raise
            self.transport = transport
            self.controller = controller
            return result

        self._run_async(
            action,
            lambda result: self._connected(result),
            "连接失败",
            finally_callback=lambda: self.connect_button.configure(state="normal"),
        )

    def _connected(self, result: dict[str, bool]) -> None:
        self.connect_button.configure(text="断开并停止两台")
        if all(result.values()):
            self.status_var.set("CANable 已连接；两台电机 UID 验证通过，仍需各自完成零点与限位验收")
        else:
            failed = [self.config["joints"][key]["display_name"] for key, ok in result.items() if not ok]
            self.status_var.set("已连接，但以下身份验证失败：" + "、".join(failed))

    def _disconnect(self) -> None:
        controller, transport = self.controller, self.transport
        self.controller = None
        self.transport = None
        try:
            if controller:
                controller.shutdown()
        finally:
            if transport:
                transport.close()
            self.recorder.finish("disconnect")
        self.connect_button.configure(text="连接并验证两台电机")
        self.status_var.set("已断开；两台电机已尝试停止")

    def _discover(self) -> None:
        controller = self._require_controller()
        if not controller:
            return
        self._run_async(
            controller.discover,
            lambda result: self.status_var.set(
                "UID 重新识别完成："
                + "，".join(f"{self.config['joints'][key]['display_name']}={'通过' if ok else '失败'}" for key, ok in result.items())
            ),
            "识别失败",
        )

    def _save_joint_config(self, key: str) -> None:
        values = self.joint_vars[key]
        kwargs = {
            "direction": int(values["direction"].get()),
            "minimum_angle_deg": float(values["minimum"].get()),
            "maximum_angle_deg": float(values["maximum"].get()),
            "maximum_speed_rad_s": float(values["max_speed"].get()),
            "maximum_torque_nm": float(values["max_torque"].get()),
            "motion_kp": float(values["kp"].get()),
            "motion_kd": float(values["kd"].get()),
            "limits_confirmed": bool(values["limits_confirmed"].get()),
        }
        controller = self.controller
        if controller:
            self._run_async(
                lambda: controller.update_joint_config(key, **kwargs),
                lambda _result: self.status_var.set(f"{self.config['joints'][key]['display_name']} 安全配置已保存；验收已取消"),
                "保存失败",
            )
            return
        cfg = self.config["joints"][key]
        cfg.update(kwargs)
        cfg["commissioned"] = False
        try:
            self.store.save(self.config)
        except Exception as exc:
            messagebox.showerror("保存失败", str(exc))
        else:
            self.status_var.set(f"{cfg['display_name']} 安全配置已离线保存；验收已取消")

    def _capture_zero(self, key: str) -> None:
        controller = self._require_controller()
        if not controller:
            return
        name = self.config["joints"][key]["display_name"]
        if not messagebox.askokcancel(
            f"采集{name}软件零点",
            "该操作会先停止两台电机，只使用停止后返回的本关节新鲜反馈。\n"
            "它不会修改电机机械零点；采集后该关节必须重新验收。",
        ):
            return
        self._run_async(
            lambda: controller.capture_zero(key),
            lambda value: self.status_var.set(f"{name} 软件零点已保存：{value:.6f} rad；请重新验收"),
            "零点采集失败",
        )

    def _toggle_commissioned(self, key: str) -> None:
        controller = self._require_controller()
        if not controller:
            return
        current = bool(self.config["joints"][key].get("commissioned"))
        name = self.config["joints"][key]["display_name"]
        if not current and not messagebox.askokcancel(
            f"验收{name}",
            "请确认：UID 对应正确；软件零点、方向和软限位均按真实机械验证；机构固定或卸载；"
            "无人穿戴；急停可达。\n\n确认后仅解除软件运动锁，不会立即使能或运动。",
        ):
            return
        self._run_async(
            lambda: controller.set_commissioned(key, not current),
            lambda _result: self.status_var.set(f"{name} 已{'完成' if not current else '取消'}验收；电机保持停止"),
            "修改验收状态失败",
        )

    def _move_joint(self, key: str) -> None:
        controller = self._require_controller()
        if not controller:
            return
        cfg = self.config["joints"][key]
        values = self.joint_vars[key]
        target = float(values["target"].get())
        speed = float(values["speed"].get())
        torque = float(values["torque"].get())
        if not controller.snapshot()["motion_active"] and not self._confirm_motion(
            f"启动{cfg['display_name']}",
            f"目标 {target:.1f}°，速度 {speed:.2f} rad/s，转矩限制 {torque:.2f} N·m。\n"
            "轨迹结束后持续控制目标角度，不等待到位确认；异常停止双机。停止或断电可能下落，请先支撑。",
        ):
            return
        self._run_async(
            lambda: controller.start_motion(
                {key: target},
                speed_rad_s={key: speed},
                torque_limit_nm={key: torque},
            ),
            lambda duration: self.status_var.set(f"{cfg['display_name']} 轨迹预计 {duration:.2f} 秒，之后持续位置控制"),
            "运动启动失败",
        )

    def _move_coordinated(self) -> None:
        controller = self._require_controller()
        if not controller:
            return
        targets = {key: float(self.sync_target_vars[key].get()) for key in JOINT_KEYS}
        speeds = {key: float(self.joint_vars[key]["speed"].get()) for key in JOINT_KEYS}
        torques = {key: float(self.joint_vars[key]["torque"].get()) for key in JOINT_KEYS}
        description = "，".join(
            f"{self.config['joints'][key]['display_name']} {targets[key]:.1f}°" for key in JOINT_KEYS
        )
        if not controller.snapshot()["motion_active"] and not self._confirm_motion(
            "启动双关节协调运动",
            description + "。\n轨迹结束后持续控制目标角度，不等待到位确认；异常停止双机。停止或断电可能下落，请先支撑。",
        ):
            return
        self._run_async(
            lambda: controller.start_motion(targets, speed_rad_s=speeds, torque_limit_nm=torques),
            lambda duration: self.status_var.set(f"双关节轨迹预计 {duration:.2f} 秒，之后持续位置控制"),
            "协调运动启动失败",
        )

    def _global_stop(self) -> None:
        controller = self.controller
        if not controller:
            self.status_var.set("未连接；没有发送停止帧")
            return
        self._run_async(
            controller.stop_all,
            lambda result: self.status_var.set(
                "双机停止结果：" + "，".join(f"{key}={value}" for key, value in result.items())
            ),
            "全局停止异常",
        )

    def _reset_safety(self) -> None:
        controller = self._require_controller()
        if not controller:
            return
        self._run_async(
            controller.reset_safety,
            lambda _result: self.status_var.set("安全锁存已复位；两台仍保持停止"),
            "无法复位安全锁存",
        )

    def _require_controller(self) -> DualMotorController | None:
        if self.controller:
            return self.controller
        messagebox.showwarning("未连接", "请先连接 CANable 并完成两台 UID 验证。")
        return None

    def _confirm_motion(self, title: str, detail: str) -> bool:
        load = self.config["load_compensation"]
        if load["enabled"] and load["strategy"] == "tracking_support":
            detail += f"\n当前承托档：Kp={load['position_kp']}、Kd={load['velocity_kd']}、Ki={load['ki_nm_per_rad_s']}，比原档更硬；首次须低速小幅验证。"
        return messagebox.askokcancel(
            title,
            detail
            + "\n\n继续前必须确认机构固定或卸载、无人穿戴、路径清空、电源限流合适且实体急停可用。",
        )

    def _run_async(
        self,
        action,
        success,
        error_title: str,
        *,
        finally_callback=None,
    ) -> None:
        def worker() -> None:
            try:
                result = action()
            except Exception as exc:
                self.ui_events.put(("error", error_title, str(exc)))
            else:
                self.ui_events.put(("success", success, result))
            finally:
                if finally_callback:
                    self.ui_events.put(("finally", finally_callback))

        threading.Thread(target=worker, name="classic-ui-action", daemon=True).start()

    def _poll_ui(self) -> None:
        if self.closing:
            return
        while True:
            try:
                event = self.ui_events.get_nowait()
            except queue.Empty:
                break
            kind = event[0]
            if kind == "log":
                self._log(event[1])
            elif kind == "error":
                messagebox.showerror(event[1], event[2])
                self.status_var.set(f"{event[1]}：{event[2]}")
            elif kind == "success":
                event[1](event[2])
            elif kind == "finally":
                event[1]()
        self._update_snapshot()
        self.after(100, self._poll_ui)

    def _update_snapshot(self) -> None:
        dataset = self.recorder.status()
        state_text = {'idle':'未开始记录','starting':'正在准备','recording':'正在记录',
                      'finalizing':'正在整理','finished':'记录已完成','error':'记录失败'}[dataset['state']]
        self.dataset_status_var.set(f"{state_text} · {dataset['counts']['feedback']} 帧反馈 / {sum(dataset['counts'].values())} 条\n{dataset['directory'] or ''}\n{dataset['error']}")
        self.dataset_start_button.configure(state="normal" if self.controller and not dataset['busy'] else "disabled")
        self.dataset_finish_button.configure(state="normal" if dataset['recording'] else "disabled")
        self.dataset_export_button.configure(state="normal" if dataset['download_ready'] else "disabled")
        controller = self.controller
        if not controller:
            self.safety_var.set("安全状态：SAFE / 未连接")
            return
        snapshot = controller.snapshot()
        if snapshot["safety_latched"]:
            self.safety_var.set("安全状态：TRIPPED · " + snapshot["safety_reason"])
        elif snapshot["motion_active"]:
            phase = {"moving": "轨迹运动中", "controlling": "持续位置控制"}
            self.safety_var.set("安全状态：" + phase.get(snapshot["control_phase"], "控制中"))
        else:
            self.safety_var.set("安全状态：SAFE / 已连接")
        self.gravity_state_var.set("空载补偿已标定并开启（下次运动时生效）" if snapshot["gravity"]["enabled"] else "补偿关闭 / 标定未完成；不会自动施加力矩")
        load = snapshot["load_compensation"]
        if not load['enabled']:
            self.load_state_var.set("负载控制关闭：原关节 PD 生效；点击下方按钮可明确启用新承托档")
        elif load['strategy'] == 'tracking_support':
            self.load_state_var.set(f"承托档（试验）Kp={load['position_kp']} Kd={load['velocity_kd']} Ki={load['ki_nm_per_rad_s']} · 附加上限 {load['maximum_bias_nm']} Nm · 变化率 {load['maximum_rate_nm_s']} Nm/s · 总上限不变")
        else:
            self.load_state_var.set(f"旧版静态补偿：附加上限 {load['maximum_bias_nm']} Nm · 可停止后明确升级承托档")
        for key in JOINT_KEYS:
            joint = snapshot["joints"][key]
            overview = self.overview_vars[key]
            overview["uid"].set("通过" if joint["verified"] else "未通过")
            overview["commissioned"].set("已验收" if joint["commissioned"] else "未验收")
            overview["angle"].set("--" if joint["actual_deg"] is None else f"{joint['actual_deg']:.2f}°")
            overview["velocity"].set(f"{joint['velocity_rad_s']:.3f}")
            overview["torque"].set(f"{joint['torque_nm']:.3f}")
            overview["temperature"].set(f"{joint['temperature_c']:.1f}°C")
            overview["age"].set("--" if joint["feedback_age_s"] is None else f"{joint['feedback_age_s']:.2f}s")
            values = self.joint_vars[key]
            sent = joint["sent_control"]
            sent_text = (f"最近下发 Kp={sent['kp']:.3f} Kd={sent['kd']:.3f} · 前馈字段 {sent['feedforward_nm']:.3f} Nm · PD估算 {sent['pd_estimate_nm']:.3f} Nm · 本次总上限 {sent['torque_limit_nm']:.2f} Nm（非实测）\n"
                         if sent and sent['age_s'] <= .3 and joint['feedback_age_s'] is not None and joint['feedback_age_s'] <= .3
                         else "当前无新鲜运动指令/反馈，不据缓存判断出力\n")
            values["identity"].set(
                f"期望 UID {joint['expected_uid']} / 实际 {joint['observed_uid'] or '--'} / "
                + ("验证通过" if joint["verified"] else "未验证")
            )
            values["telemetry"].set(
                f"角度 {overview['angle'].get()} · 原始位置 "
                f"{joint['actual_raw_rad'] if joint['actual_raw_rad'] is not None else '--'} rad · "
                f"速度 {joint['velocity_rad_s']:.3f} rad/s · 转矩 {joint['torque_nm']:.3f} N·m · "
                f"温度 {joint['temperature_c']:.1f}°C · 故障 0x{joint['fault_bits']:02X}\n"
                f"状态 {joint['phase']} · 目标误差 {joint['error_deg'] if joint['error_deg'] is not None else '--'}° · "
                f"重力补偿 {joint['gravity_torque_nm']:.3f} Nm · 误差补偿 {joint['load_bias_nm']:.3f} Nm（电机端）\n"
                + sent_text + f"提示 {joint['notice'] or '无'}\n" +
                " · ".join(f"{label} {joint['diagnostics'][name]['value'] if joint['diagnostics'][name]['fresh'] else '--（未读到/已过期）'}" for name, label in (
                    ("run_mode", "模式"), ("limit_torque_nm", "上限 Nm"), ("iq_a", "Iq A"), ("vbus_v", "电压 V"),
                ))
            )
            zero = joint["software_zero_rad"]
            values["zero"].set("未设置（禁止运动）" if zero is None else f"{zero:.6f} rad")
            self.commission_buttons[key].configure(
                text="取消本关节验收" if joint["commissioned"] else "标记本关节已验收"
            )

    def _log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{stamp}] {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _draw_chart(self) -> None:
        if self.closing:
            return
        canvas = self.chart
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 500), max(canvas.winfo_height(), 320)
        controller = self.controller
        if not controller:
            canvas.create_text(width / 2, height / 2, text="连接后显示双路反馈曲线", fill="#9ba8b6")
            self.after(180, self._draw_chart)
            return
        metric = self.metric_var.get()
        try:
            histories = controller.history_snapshot(metric)
        except ValueError:
            histories = {key: [] for key in JOINT_KEYS}
        valid = [point for data in histories.values() for point in data if point[1] is not None]
        if not valid:
            canvas.create_text(width / 2, height / 2, text="等待有效反馈", fill="#9ba8b6")
            self.after(180, self._draw_chart)
            return
        newest = max(point[0] for point in valid)
        start = newest - 30.0
        recent = {
            key: [(stamp, value) for stamp, value in data if value is not None and stamp >= start]
            for key, data in histories.items()
        }
        values = [value for data in recent.values() for _stamp, value in data]
        low, high = min(values), max(values)
        if abs(high - low) < 1e-9:
            padding = max(1.0, abs(high) * 0.1)
        else:
            padding = (high - low) * 0.12
        low -= padding
        high += padding
        left, right, top, bottom = 70, width - 20, 20, height - 35
        canvas.create_rectangle(left, top, right, bottom, outline="#36414f")
        canvas.create_text(8, top + 10, anchor="w", fill="#d6dde7", text=metric)
        canvas.create_text(left - 6, top, anchor="e", fill="#9ba8b6", text=f"{high:.2f}")
        canvas.create_text(left - 6, bottom, anchor="e", fill="#9ba8b6", text=f"{low:.2f}")
        for key in JOINT_KEYS:
            points: list[float] = []
            for stamp, value in recent[key]:
                x = left + (stamp - start) / 30.0 * (right - left)
                y = bottom - (value - low) / (high - low) * (bottom - top)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=JOINT_COLORS[key], width=2, smooth=True)
        self.chart_info_var.set(f"最近 30 秒 · {metric} · 样本 {len(values)}")
        self.after(180, self._draw_chart)

    def _on_close(self) -> None:
        self.closing = True
        self.connect_cancel.set()
        try:
            self._disconnect()
        finally:
            self.recorder.finish("classic_shutdown")
            self.recorder.wait()
            self.destroy()


if __name__ == "__main__":
    ClassicDualApp().mainloop()
