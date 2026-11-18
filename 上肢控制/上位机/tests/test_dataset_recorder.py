"""Dataset tests use temporary directories and fake CAN links only."""
import copy
import csv
import hashlib
import json
import math
import tempfile
import threading
import time
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

from dataset_recorder import DatasetRecorder, FIELDS, COMMON, PACKAGE_NAMES, utc_timestamp
from classic_dual_controller import DEFAULT_CONFIG, ClassicConfigStore, DualMotorController
from test_classic_dual_controller import FakeTransport, feedback_response, uid_response
from cybergear_protocol import CMD_PARAM_READ, PARAM_RUN_MODE, payload_param, encode_slcan_extended
import digital_twin_server as server_module


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.recorder = DatasetRecorder(self.root / "datasets")
        self.addCleanup(self.close)

    def close(self):
        self.recorder.finish()
        self.recorder.wait()

    def finish(self):
        self.recorder.finish()
        self.assertTrue(self.recorder.wait())
        return self.recorder.directory

    def rows(self, kind):
        with (self.recorder.directory / f"{kind}.csv").open(encoding="utf-8", newline="") as handle:
            return list(csv.DictReader(handle))

    def test_utc_timestamp_is_exact_and_has_explicit_timezone(self):
        self.assertEqual(utc_timestamp(1_000_000_123), "1970-01-01T00:00:01.000000123Z")

    def test_full_package_headers_units_nulls_and_hashes(self):
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        self.recorder.start(cfg)
        cfg["bus"]["port"] = "changed"
        self.recorder.append("feedback", {"joint":"shoulder", "motor_can_id":126,
            "position_motor_rad":0., "position_joint_rad":None, "identity_verified":True})
        self.recorder.append("commands", {"joint":"shoulder", "pd_estimate_nm":1.2})
        self.recorder.append("parameters", {"parameter_name":"iq_a", "value":.11, "unit":"A"})
        self.recorder.event("info", '含逗号,双引号"和换行\n事件')
        folder = self.finish()
        metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
        self.assertTrue(metadata["complete"])
        self.assertEqual(metadata["configuration_at_start"]["bus"]["port"], "COM11")
        feedback = self.rows("feedback")[0]
        self.assertEqual(feedback["position_motor_rad"], "0.0")
        self.assertEqual(feedback["position_joint_rad"], "")
        self.assertEqual(feedback["identity_verified"], "true")
        self.assertTrue(feedback["timestamp_utc"].endswith("Z"))
        self.assertGreater(int(feedback["timestamp_unix_ns"]), 2**53)
        self.assertEqual(self.rows("events")[0]["message"], '含逗号,双引号"和换行\n事件')
        for kind in FIELDS:
            with (folder / f"{kind}.csv").open(encoding="utf-8", newline="") as handle:
                self.assertEqual(next(csv.reader(handle)), [*COMMON, *FIELDS[kind]])
            self.assertIn(b"\r\n", (folder / f"{kind}.csv").read_bytes())
        for name, digest in metadata["sha256"].items():
            self.assertEqual(hashlib.sha256((folder / name).read_bytes()).hexdigest(), digest)
        archive = self.recorder.archive(self.recorder.session_id)
        with zipfile.ZipFile(archive) as handle:
            self.assertEqual({Path(n).name for n in handle.namelist()}, set(PACKAGE_NAMES))

    def test_monotonic_time_survives_wall_clock_going_backwards(self):
        self.recorder.start(DEFAULT_CONFIG)
        mono = self.recorder.started_mono_ns
        wall = self.recorder.started_wall_ns
        for delta, wall_delta in ((1000, 1000), (2000, -1_000_000_000)):
            self.recorder.append("feedback", {"monotonic_ns":mono+delta, "timestamp_unix_ns":wall+wall_delta})
        self.finish()
        a, b = self.rows("feedback")
        self.assertLess(int(b["timestamp_unix_ns"]), int(a["timestamp_unix_ns"]))
        self.assertGreater(float(b["elapsed_s"]), float(a["elapsed_s"]))

    def test_sessions_unique_and_duplicate_start_does_not_overwrite(self):
        self.recorder.start(DEFAULT_CONFIG)
        first = self.recorder.session_id
        with self.assertRaises(RuntimeError):
            self.recorder.start(DEFAULT_CONFIG)
        self.finish()
        self.recorder.start(DEFAULT_CONFIG)
        self.assertNotEqual(first, self.recorder.session_id)
        self.finish()
        self.assertTrue((self.recorder.root / first / "metadata.json").exists())
        self.assertTrue(self.recorder.archive(first).exists())

    def test_active_export_and_path_traversal_rejected(self):
        self.recorder.start(DEFAULT_CONFIG)
        with self.assertRaises(RuntimeError):
            self.recorder.archive(self.recorder.session_id)
        for value in ("../classic_dual_config.json", "", None, "A"*32, "0"*32+"/x"):
            with self.assertRaises(ValueError):
                self.recorder.archive(value)

    def test_queue_overflow_marks_incomplete_and_never_blocks(self):
        self.recorder = DatasetRecorder(self.root / "small", queue_size=1)
        with patch("dataset_recorder.threading.Thread"):
            self.recorder.start(DEFAULT_CONFIG)
        self.assertTrue(self.recorder.append("feedback", {}))
        self.assertFalse(self.recorder.append("feedback", {}))
        self.recorder._write_loop()
        metadata = json.loads((self.recorder.directory / "metadata.json").read_text(encoding="utf-8"))
        self.assertFalse(metadata["complete"])
        self.assertEqual(metadata["dropped_rows"], 1)
        self.assertEqual(metadata["stop_reason"], "queue_overflow")

    def test_writer_error_is_reported_without_escaping_to_producer(self):
        with patch("dataset_recorder.threading.Thread"):
            self.recorder.start(DEFAULT_CONFIG)
        self.recorder.append("feedback", {})
        with patch("dataset_recorder.csv.DictWriter.writerow", side_effect=OSError("disk full")):
            self.recorder._write_loop()
        self.assertEqual(self.recorder.state, "error")
        self.assertIn("disk full", self.recorder.error)
        self.assertFalse(json.loads((self.recorder.directory / "metadata.json").read_text(encoding="utf-8"))["complete"])

    def test_nonfinite_values_are_missing_and_formula_prefixes_escaped(self):
        self.recorder.start(DEFAULT_CONFIG)
        self.recorder.append("parameters", {"value":float("nan")})
        self.recorder.event("info", "=HYPERLINK(\"bad\")")
        self.finish()
        self.assertEqual(self.rows("parameters")[0]["value"], "")
        self.assertTrue(self.rows("events")[0]["message"].startswith("'="))

    def test_starting_recorder_hook_does_not_wait_for_filesystem_lock(self):
        self.recorder.state = "starting"
        result = []
        with self.recorder.lock:
            thread = threading.Thread(target=lambda: result.append(self.recorder.append("feedback", {})))
            thread.start()
            thread.join(.5)
            self.assertFalse(thread.is_alive())
        self.assertEqual(result, [False])
        self.recorder.state = "idle"

    def test_prestart_events_are_not_mislabelled_as_part_of_session(self):
        self.recorder.start(DEFAULT_CONFIG)
        self.assertFalse(self.recorder.append("feedback", {"monotonic_ns":self.recorder.started_mono_ns-1}))
        self.finish()
        self.assertEqual(self.rows("feedback"), [])

    def test_empty_recording_is_finalized_but_explicitly_marked_no_feedback(self):
        self.recorder.start(DEFAULT_CONFIG,interface="offline_synthetic")
        self.finish()
        metadata = json.loads((self.recorder.directory/"metadata.json").read_text(encoding="utf-8"))
        self.assertTrue(metadata["complete"])
        self.assertIn("no_feedback",metadata["quality_flags"])
        self.assertIn("no_shoulder_feedback",metadata["quality_flags"])
        self.assertEqual(metadata["data_origin"],"synthetic_test_only")

    def test_validator_accepts_directory_zip_and_detects_modified_data(self):
        from tools.validate_dataset import validate
        self.recorder.start(DEFAULT_CONFIG)
        self.recorder.append("feedback", {"joint":"shoulder", "position_motor_rad":0.})
        folder = self.finish()
        self.assertTrue(validate(folder)["schema_valid"])
        archive = self.recorder.archive(self.recorder.session_id)
        self.assertEqual(validate(archive)["row_counts"]["feedback"],1)
        with (folder/"feedback.csv").open('a',encoding='utf-8') as handle:
            handle.write('modified')
        with self.assertRaisesRegex(ValueError,'Checksum mismatch'):
            validate(folder)


class ControllerTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        for joint in self.config["joints"].values():
            joint.update(software_zero_rad=.1, direction=-1)
        self.link = FakeTransport()
        self.events = []
        self.controller = DualMotorController(self.link, self.config,
            telemetry=lambda kind,payload: self.events.append((kind,payload)))

    def test_feedback_sample_is_emitted_once_with_coordinates_and_no_cached_parameter_fill(self):
        self.controller.feed_line(feedback_response(126, position_rad=.2, velocity_rad_s=.3))
        kind, row = self.events[0]
        self.assertEqual(kind, "feedback")
        self.assertAlmostEqual(row["position_joint_rad"], -.1, delta=.001)
        self.assertLess(row["velocity_joint_rad_s"], 0)
        self.assertNotIn("iq_a", row)
        self.assertIsNone(row["target_joint_rad"])
        self.assertEqual(len(row["payload_hex"]), 16)
        self.assertEqual(self.link.sent, [])

    def test_only_correlated_accepted_parameter_is_recorded(self):
        cid = (CMD_PARAM_READ<<24)|(126<<8)|253
        line = encode_slcan_extended(cid, payload_param(PARAM_RUN_MODE,0,"u8")).decode()
        self.controller.feed_line(line)
        self.assertEqual(self.events, [])
        self.controller._request_parameter("shoulder", PARAM_RUN_MODE)
        self.controller.feed_line(line)
        self.assertEqual(self.events[0][0], "parameters")
        self.assertEqual(self.events[0][1]["value"], 0)

    def test_failed_record_callback_does_not_latch_or_prevent_feedback(self):
        self.controller.telemetry_callback = lambda *_: (_ for _ in ()).throw(OSError("recorder failed"))
        self.controller.feed_line(feedback_response(126))
        self.assertFalse(self.controller.safety_latched)
        self.assertIsNotNone(self.controller.runtimes["shoulder"].actual_deg)
        self.assertIn("recorder failed", self.controller.telemetry_error)

    def test_periodic_command_stream_contains_actual_wire_fields_and_not_duplicate_pd(self):
        from test_hold_gravity import HoldGravityTests
        fixture = HoldGravityTests()
        fixture.setUp()
        fixture.controller.telemetry_callback = lambda kind,payload: self.events.append((kind,payload))
        fixture.plan(20.)
        fixture.advance(1., angles={"elbow":18., "shoulder":18.})
        commands = [row for kind,row in self.events if kind == "commands"]
        self.assertEqual({row["motor_can_id"] for row in commands}, {126,127})
        self.assertAlmostEqual(commands[-1]["feedforward_motor_nm"], 0., delta=.001)
        self.assertGreater(commands[-1]["pd_estimate_nm"], .05)
        self.assertEqual(commands[-1]["payload_hex"], fixture.controller.runtimes["shoulder"].last_sent_control["payload"])

    def test_finishing_recording_does_not_cancel_active_control(self):
        from test_hold_gravity import HoldGravityTests
        fixture = HoldGravityTests()
        fixture.setUp()
        with tempfile.TemporaryDirectory() as folder:
            recorder = DatasetRecorder(Path(folder))
            fixture.controller.telemetry_callback = recorder.append
            recorder.start(fixture.config)
            fixture.plan()
            plan = fixture.controller.motion_plan
            fixture.advance(.5)
            recorder.finish()
            self.assertTrue(recorder.wait())
            fixture.link.sent.clear()
            fixture.advance(.5)
            self.assertIs(fixture.controller.motion_plan,plan)
            self.assertFalse(fixture.controller.safety_latched)
            self.assertTrue(any((cid>>24)&31 == 1 for cid,_ in fixture.link.sent))
            self.assertFalse(any((cid>>24)&31 == 4 for cid,_ in fixture.link.sent))


class DatasetHttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        ClassicConfigStore(root / "config.json").save(copy.deepcopy(DEFAULT_CONFIG))
        self.twin = server_module.CyberArmTwin(root / "config.json")
        self.link = FakeTransport()
        self.link.connected = True
        self.twin.link = self.link
        self.twin.controller = DualMotorController(self.link, self.twin.config, telemetry=self.twin.recorder.append)
        self.patches = [patch.object(server_module,"TWIN",self.twin), patch.object(server_module.list_ports,"comports",return_value=[])]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.server = ThreadingHTTPServer(("127.0.0.1",0),server_module.DashboardHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close_server)
        self.addCleanup(self.close_recorder)

    def close_recorder(self):
        self.twin.recorder.finish()
        self.twin.recorder.wait()

    def close_server(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)

    def request(self, path, body=None, origin=None):
        headers = {"Host":"127.0.0.1:8765", "Content-Type":"application/json"}
        if origin:
            headers["Origin"] = origin
        return urlopen(Request(f"http://127.0.0.1:{self.server.server_port}"+path,
            data=None if body is None else json.dumps(body).encode(), headers=headers), timeout=5)

    def test_start_finish_download_do_not_send_any_motor_commands(self):
        with self.request("/api/dataset/start", {}) as response:
            state = json.load(response)["state"]
        self.assertTrue(state["dataset"]["recording"])
        self.twin.controller.feed_line(feedback_response(126))
        with self.request("/api/dataset/finish", {}) as response:
            json.load(response)
        self.assertTrue(self.twin.recorder.wait())
        self.assertEqual(self.link.sent, [])
        self.assertIsNone(self.twin.controller.motion_plan)
        with self.request("/api/dataset/download/"+state["dataset"]["session_id"]) as response:
            self.assertEqual(response.headers["Content-Type"], "application/zip")
            self.assertTrue(response.read().startswith(b"PK"))

    def test_cross_origin_and_extra_fields_rejected(self):
        for body, origin, code in [({},"https://example.com",403), ({"targets_deg":{"shoulder":30}},None,400)]:
            with self.assertRaises(HTTPError) as error:
                self.request("/api/dataset/start",body,origin)
            self.assertEqual(error.exception.code,code)
        self.assertEqual(self.link.sent, [])

    def test_disconnected_start_rejected_without_recording(self):
        self.link.connected = False
        with self.assertRaises(HTTPError) as error:
            self.request("/api/dataset/start",{})
        self.assertEqual(error.exception.code,409)
        self.assertFalse(self.twin.recorder.status()["busy"])

    def test_saved_configuration_change_is_part_of_the_dataset(self):
        self.twin.start_dataset()
        self.twin._save_recorded_config(self.twin.config)
        self.close_recorder()
        with (self.twin.recorder.directory/"events.csv").open(encoding="utf-8",newline="") as handle:
            rows=list(csv.DictReader(handle))
        configuration = next(row for row in rows if row["configuration_json"])
        self.assertEqual(json.loads(configuration["configuration_json"]), self.twin.config)

    def test_slow_dataset_setup_does_not_hold_emergency_command_lock(self):
        ready, release = threading.Event(), threading.Event()
        def slow_start(*_args,**_kwargs):
            ready.set()
            release.wait(2)
            return self.twin.recorder.status()
        with patch.object(self.twin.recorder,'start',side_effect=slow_start):
            thread=threading.Thread(target=self.twin.start_dataset)
            thread.start()
            try:
                self.assertTrue(ready.wait(1))
                acquired=self.twin.lifecycle_lock.acquire(timeout=.3)
                if acquired:
                    self.twin.lifecycle_lock.release()
                self.assertTrue(acquired)
            finally:
                release.set()
                thread.join(2)
            self.assertFalse(thread.is_alive())
        self.assertEqual(self.link.sent,[])


if __name__ == "__main__":
    unittest.main()
