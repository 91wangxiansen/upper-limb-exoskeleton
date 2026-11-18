"""Event-based CSV dataset writer. No serial access, motion, or synchronous disk I/O in append()."""
from __future__ import annotations

import copy
import csv
import hashlib
import json
import math
import queue
import re
import threading
import time
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path

SCHEMA_VERSION = "cyberarm.dataset.v1"
COMMON = {
    "session_id": ("string", "", "Unique recording session"),
    "sample_index": ("integer", "", "Global enqueue order, not a hardware sequence counter"),
    "timestamp_utc": ("string", "UTC", "Host event processing time, RFC 3339"),
    "timestamp_unix_ns": ("integer", "ns", "Host wall clock; import as int64/string, never float"),
    "monotonic_ns": ("integer", "ns", "Host monotonic clock; import as int64/string"),
    "elapsed_s": ("number", "s", "Monotonic time since recording began"),
}
FIELDS = {
    "feedback": {
        "joint": ("string", "", "shoulder or elbow"),
        "motor_can_id": ("integer", "", "Motor CAN address"),
        "observed_uid": ("string", "", "Most recently received MCU UID"),
        "identity_verified": ("boolean", "", "UID matches configured joint"),
        "software_zero_rad": ("number", "rad", "Software zero used for this row"),
        "direction": ("integer", "", "Software coordinate sign, +1 or -1"),
        "position_motor_rad": ("number", "rad", "Type 2 raw motor-coordinate position"),
        "position_joint_rad": ("number", "rad", "direction * (motor position - software zero)"),
        "position_joint_deg": ("number", "deg", "Same joint position in degrees"),
        "velocity_motor_rad_s": ("number", "rad/s", "Type 2 motor-coordinate velocity"),
        "velocity_joint_rad_s": ("number", "rad/s", "direction * motor velocity"),
        "torque_feedback_motor_nm": ("number", "N*m", "Motor-reported torque, not an independent force sensor"),
        "temperature_c": ("number", "degC", "Motor-reported temperature"),
        "mode_status": ("integer", "", "Type 2 status: 0 reset, 1 calibration, 2 motor"),
        "fault_bits": ("integer", "", "Type 2 fault bitmask"),
        "target_joint_rad": ("number", "rad", "Host final target; not measured"),
        "setpoint_joint_rad": ("number", "rad", "Host current trajectory reference; not measured"),
        "target_error_rad": ("number", "rad", "Final target minus joint position; derived"),
        "control_phase": ("string", "", "Host control state at feedback processing"),
        "safety_latched": ("boolean", "", "Host safety latch state at processing"),
        "can_id_hex": ("string", "", "Original CAN identifier, hexadecimal"),
        "payload_hex": ("string", "", "Original eight-byte Type 2 payload"),
    },
    "commands": {
        "joint": ("string", "", "shoulder or elbow"),
        "motor_can_id": ("integer", "", "Motor CAN address"),
        "motion_epoch": ("integer", "", "Host motion-plan identifier"),
        "position_motor_rad": ("number", "rad", "Successfully sent quantized position command"),
        "velocity_motor_rad_s": ("number", "rad/s", "Successfully sent quantized velocity command"),
        "kp": ("number", "N*m/rad", "Successfully sent quantized position gain"),
        "kd": ("number", "N*m*s/rad", "Successfully sent quantized velocity gain"),
        "feedforward_motor_nm": ("number", "N*m", "Sent torque field; not measured torque"),
        "torque_limit_nm": ("number", "N*m", "Requested total torque ceiling"),
        "pd_estimate_nm": ("number", "N*m", "Host PD estimate; not measured"),
        "total_estimate_nm": ("number", "N*m", "PD plus sent feedforward estimate; not measured"),
        "gravity_feedforward_motor_nm": ("number", "N*m", "Host gravity feedforward component"),
        "load_bias_motor_nm": ("number", "N*m", "Host adaptive load compensation component"),
        "feedback_age_at_send_s": ("number", "s", "Age of feedback used for the command estimate"),
        "can_id_hex": ("string", "", "Sent CAN identifier"),
        "payload_hex": ("string", "", "Sent Type 1 payload"),
    },
    "parameters": {
        "joint": ("string", "", "shoulder or elbow"),
        "motor_can_id": ("integer", "", "Motor CAN address"),
        "parameter_index_hex": ("string", "", "Accepted Type 17 parameter index"),
        "parameter_name": ("string", "", "Readback name"),
        "value": ("number", "", "Accepted finite readback; see unit column"),
        "unit": ("string", "", "Physical unit, or empty for an enum"),
        "can_id_hex": ("string", "", "Original CAN identifier"),
        "payload_hex": ("string", "", "Original parameter response payload"),
    },
    "events": {
        "level": ("string", "", "Application log level"),
        "message": ("string", "", "Application event; dangerous spreadsheet prefixes escaped"),
        "configuration_json": ("string", "", "Full configuration saved during this recording, when present"),
    },
}
CSV_NAMES = tuple(f"{name}.csv" for name in FIELDS)
PACKAGE_NAMES = (*CSV_NAMES, "metadata.json", "schema.json", "README.txt")


def utc_timestamp(ns: int) -> str:
    seconds, fraction = divmod(ns, 1_000_000_000)
    return datetime.fromtimestamp(seconds, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S") + f".{fraction:09d}Z"


def _json_write(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def _cell(value):
    if value is None or isinstance(value, float) and not math.isfinite(value):
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str) and value.startswith(("=", "+", "-", "@", "\t", "\r", "\n")):
        return "'" + value
    return value


class DatasetRecorder:
    def __init__(self, root: Path, *, source_root: Path | None = None, queue_size: int = 10000):
        self.root = Path(root).resolve()
        self.source_root = Path(source_root) if source_root else None
        self.queue_size = queue_size
        self.lock = threading.RLock()
        self.archive_lock = threading.Lock()
        self.thread = None
        self.state = "idle"
        self.session_id = None
        self.directory = None
        self.error = ""
        self.counts = dict.fromkeys(FIELDS, 0)
        self.feedback_counts = {"shoulder":0, "elbow":0}
        self.dropped_rows = 0
        self.metadata_ready = False

    def status(self) -> dict:
        with self.lock:
            return {"supported": True, "state": self.state, "recording": self.state == "recording",
                    "busy": self.state in ("starting", "recording", "finalizing"), "session_id": self.session_id,
                    "directory": str(self.directory) if self.directory else None,
                    "counts": dict(self.counts), "dropped_rows": self.dropped_rows, "error": self.error,
                    "download_ready": self.state in ("finished", "error") and self.metadata_ready}

    def start(self, config: dict, *, interface: str = "digital_twin") -> dict:
        with self.lock:
            if self.state in ("starting", "recording", "finalizing"):
                raise RuntimeError("已有数据记录正在进行或整理，请稍后重试")
            self.state = "starting"
            self.metadata_ready = False
            self.session_id = uuid.uuid4().hex
            self.directory = self.root / self.session_id
            self.queue = queue.Queue(maxsize=self.queue_size)
            self.finish_event = threading.Event()
            self.counts = dict.fromkeys(FIELDS, 0)
            self.feedback_counts = {"shoulder":0, "elbow":0}
            self.unverified_feedback_rows = self.fault_feedback_rows = 0
            self.dropped_rows = self.sequence = self.ignored_prestart_rows = 0
            self.error = ""
            self.stop_reason = ""
            self.started_wall_ns, self.started_mono_ns = time.time_ns(), time.monotonic_ns()
            self.ended_wall_ns = self.ended_mono_ns = None
            self.metadata = {
                "schema_version": SCHEMA_VERSION, "session_id": self.session_id, "interface": interface,
                "application_version": "2.3.3", "status": "recording", "complete": False,
                "data_origin": "synthetic_test_only" if interface == "offline_synthetic" else "controller_telemetry",
                "started_at_utc": utc_timestamp(self.started_wall_ns),
                "started_unix_ns": str(self.started_wall_ns),
                "started_monotonic_ns": str(self.started_mono_ns),
                "timestamp_source": "host event processing; not hardware acquisition or CAN wire arrival",
                "time_basis": "UTC wall clock plus host monotonic clock; use elapsed_s for durations",
                "sampling": "event-driven, one feedback row per accepted Type 2 frame; no interpolation or resampling",
                "synchronization": "motors and parameter readbacks are asynchronous; not hardware synchronized",
                "missing_values": "empty CSV cells; never forward-fill missing parameters or substitute zero",
                "integer_import": "Unix/monotonic nanoseconds require int64 or string; spreadsheet floats lose precision",
                "configuration_at_start": copy.deepcopy(config),
                "configuration_changes": "events.csv configuration_json records subsequent successful configuration saves",
                "feedback_coordinates": "joint position = direction * (motor position - software zero)",
                "command_stream": "successful periodic Type 1 control writes, not motor acknowledgements; startup commands appear in events",
                "durability": "writer flushes at most once per second; abnormal process exit may leave an incomplete session",
                "files": list(PACKAGE_NAMES),
            }
            try:
                self.directory.mkdir(parents=True, exist_ok=False)
                if self.source_root:
                    self.metadata["source_sha256"] = {
                        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in sorted(self.source_root.glob("*.py"))
                    }
                schema = {"schema_version": SCHEMA_VERSION, "csv_encoding": "UTF-8 without BOM",
                          "csv_format": "RFC 4180 conventions, header, comma delimiter, CRLF, double-quote escaping",
                          "tables": {name + ".csv": [{"name": key, "type": spec[0], "unit": spec[1],
                              "description": spec[2], "nullable": key not in COMMON}
                              for key, spec in {**COMMON, **columns}.items()] for name, columns in FIELDS.items()}}
                _json_write(self.directory / "schema.json", schema)
                _json_write(self.directory / "metadata.json", self.metadata)
                (self.directory / "README.txt").write_text(
                    "CyberArm dataset v1\n"
                    "feedback.csv: motor-reported feedback, one joint per row.\n"
                    "commands.csv: successfully sent periodic commands and HOST ESTIMATES, not measured output.\n"
                    "parameters.csv: asynchronous accepted parameter readbacks; no forward-filled values.\n"
                    "events.csv: application events; spreadsheet formula prefixes escaped with an apostrophe.\n"
                    "metadata.json: configuration, provenance, row counts, integrity and completion status.\n"
                    "schema.json: field names, data types, coordinates and units. Missing cells are null, not zero.\n"
                    "UTC timestamps follow RFC 3339. Nanosecond digits are clock representation, not measurement accuracy.\n"
                    "Use elapsed_s/monotonic_ns for time differences; wall clock may change during recording.\n"
                    "sample_index is enqueue order; timestamps across threads may arrive out of order.\n"
                    "Use pandas.read_csv(..., dtype={'timestamp_unix_ns':'Int64','monotonic_ns':'Int64'}).\n"
                    "No hardware timestamp, synchronization, calibrated force sensor, or human-load labels.\n"
                    "Recording or ending a recording does NOT enable, move, or stop motors.\n",
                    encoding="utf-8")
                self.state = "recording"
                self.thread = threading.Thread(target=self._write_loop, name="dataset-writer", daemon=True)
                self.thread.start()
            except Exception as exc:
                self.state, self.error = "error", f"创建数据记录失败：{exc}"
                raise RuntimeError(self.error) from exc
            return self.status()

    def append(self, kind: str, payload: dict) -> bool:
        """Non-blocking enqueue only. Failure affects recording, never motion."""
        # start() does filesystem setup while holding its lock; inactive hooks
        # must return without waiting for that setup on the control thread.
        if self.state != "recording":
            return False
        with self.lock:
            if self.state != "recording":
                return False
            if kind not in FIELDS:
                return False
            wall = payload.get("timestamp_unix_ns", time.time_ns())
            mono = payload.get("monotonic_ns", time.monotonic_ns())
            if mono < self.started_mono_ns:
                self.ignored_prestart_rows += 1
                return False
            self.sequence += 1
            row = {**payload, "session_id": self.session_id, "sample_index": self.sequence,
                   "timestamp_utc": utc_timestamp(wall), "timestamp_unix_ns": wall,
                   "monotonic_ns": mono, "elapsed_s": (mono-self.started_mono_ns)/1e9}
            try:
                self.queue.put_nowait((kind, row))
            except queue.Full:
                self.dropped_rows += 1
                self.error = "记录队列已满，数据集不完整；已结束数据采集，未改变电机控制"
                self._finish_locked("queue_overflow")
                return False
            return True

    def event(self, level: str, message: str, **fields) -> None:
        self.append("events", {"level": level, "message": message, **fields})

    def _finish_locked(self, reason: str) -> None:
        self.state = "finalizing"
        self.stop_reason = reason
        self.ended_wall_ns, self.ended_mono_ns = time.time_ns(), time.monotonic_ns()
        self.finish_event.set()

    def finish(self, reason: str = "user") -> dict:
        with self.lock:
            if self.state == "recording":
                self.event("info", f"结束数据记录：{reason}；未停止电机")
                if self.state == "recording":
                    self._finish_locked(reason)
            return self.status()

    def wait(self, timeout: float = 5) -> bool:
        thread = self.thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout)
        return not (thread and thread.is_alive())

    def _write_loop(self) -> None:
        handles = {}
        try:
            writers = {}
            for kind, columns in FIELDS.items():
                handle = (self.directory / f"{kind}.csv").open("w", encoding="utf-8", newline="")
                handles[kind] = handle
                writers[kind] = csv.DictWriter(handle, fieldnames=[*COMMON, *columns], extrasaction="ignore", lineterminator="\r\n")
                writers[kind].writeheader()
            last_flush = time.monotonic()
            while True:
                try:
                    kind, row = self.queue.get(timeout=.1)
                except queue.Empty:
                    if self.finish_event.is_set():
                        break
                else:
                    writers[kind].writerow({key: _cell(value) for key, value in row.items()})
                    with self.lock:
                        self.counts[kind] += 1
                        if kind == "feedback":
                            if row.get("joint") in self.feedback_counts:
                                self.feedback_counts[row["joint"]] += 1
                            self.unverified_feedback_rows += row.get("identity_verified") is not True
                            self.fault_feedback_rows += bool(row.get("fault_bits"))
                if time.monotonic()-last_flush >= 1:
                    for handle in handles.values():
                        handle.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            with self.lock:
                self.error = f"数据写入失败：{exc}；未改变电机控制"
                if self.state == "recording":
                    self._finish_locked("writer_error")
        finally:
            for handle in handles.values():
                try:
                    handle.close()
                except OSError as exc:
                    self.error = f"数据关闭失败：{exc}"
            try:
                metadata = copy.deepcopy(self.metadata)
                metadata.update(status="error" if self.error else "finished", complete=not bool(self.error),
                    ended_at_utc=utc_timestamp(self.ended_wall_ns or time.time_ns()),
                    duration_s=((self.ended_mono_ns or time.monotonic_ns())-self.started_mono_ns)/1e9,
                    stop_reason=self.stop_reason, row_counts=dict(self.counts), dropped_rows=self.dropped_rows,
                    unwritten_rows=self.sequence-sum(self.counts.values()),
                    ignored_prestart_rows=self.ignored_prestart_rows, error=self.error)
                metadata["feedback_counts_by_joint"] = dict(self.feedback_counts)
                metadata["unverified_feedback_rows"] = self.unverified_feedback_rows
                metadata["fault_feedback_rows"] = self.fault_feedback_rows
                metadata["quality_flags"] = (["no_feedback"] if not self.counts["feedback"] else []) + [
                    f"no_{key}_feedback" for key,count in self.feedback_counts.items() if not count]
                if self.unverified_feedback_rows:
                    metadata["quality_flags"].append("unverified_identity")
                if self.fault_feedback_rows:
                    metadata["quality_flags"].append("motor_fault_reported")
                if self.error:
                    metadata["quality_flags"].append("incomplete_recording")
                metadata["completion_note"] = "complete means writer finalized without error, not that motors were synchronized or the experiment succeeded"
                metadata["sha256"] = {}
                for name in (*CSV_NAMES, "schema.json", "README.txt"):
                    path = self.directory / name
                    if path.is_file():
                        digest = hashlib.sha256()
                        with path.open("rb") as handle:
                            for block in iter(lambda: handle.read(1024*1024), b""):
                                digest.update(block)
                        metadata["sha256"][name] = digest.hexdigest()
                _json_write(self.directory / "metadata.json", metadata)
                self.metadata_ready = True
            except Exception as exc:
                self.error = f"数据集元数据保存失败：{exc}"
            with self.lock:
                self.state = "error" if self.error else "finished"

    def archive(self, session_id: str) -> Path:
        if not isinstance(session_id, str) or not re.fullmatch(r"[0-9a-f]{32}", session_id):
            raise ValueError("无效的数据集编号")
        candidate = self.root / session_id
        directory = candidate.resolve()
        if directory.parent != self.root or candidate.is_symlink():
            raise ValueError("无效的数据集路径")
        if any((directory / name).is_symlink() for name in PACKAGE_NAMES):
            raise ValueError("数据集文件不能是符号链接")
        with self.lock:
            if session_id == self.session_id and self.state in ("recording", "finalizing"):
                raise RuntimeError("请先结束记录，等待数据整理完成后再下载")
        try:
            metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ValueError("数据集不存在或元数据无效") from exc
        if metadata.get("schema_version") != SCHEMA_VERSION or metadata.get("status") not in ("finished", "error"):
            raise RuntimeError("数据集尚未正常整理完成；请检查本地原始文件")
        with self.archive_lock:
            destination = self.root / f"{session_id}.zip"
            if destination.is_symlink():
                raise ValueError("无效的导出路径")
            if destination.is_file() and zipfile.is_zipfile(destination):
                return destination
            temporary = destination.with_suffix(".zip.tmp")
            if temporary.is_symlink():
                raise ValueError("无效的导出临时路径")
            with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_STORED) as archive:
                for name in PACKAGE_NAMES:
                    path = directory / name
                    if path.is_file() and not path.is_symlink():
                        archive.write(path, arcname=f"{session_id}/{name}")
            temporary.replace(destination)
            return destination
