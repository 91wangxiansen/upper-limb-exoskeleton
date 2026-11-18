import copy
import json
import sys
import threading
import time
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import Mock, patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs
from urllib.request import Request, ProxyHandler, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hand_device import HandBoard, HandDevice, LinkError
import digital_twin_server as server


class FakeBoard:
    def __init__(self):
        self.state = dict(token="test-token", owner="", epoch=1, armed=0, ready=1, gripping=0,
                          disconnectRelease=0, speedControl=1, gestures=[], fault="", fingers=[
                              dict(id=i+1, online=1, p=500, q=0, phase="held", load=-210,
                                   limit=300, speedSet=350) for i in range(4)])
        self.sent = []
        self.fail = False
        self.drop_ack = False
        self.entered, self.unblock = threading.Event(), threading.Event()
        self.block = False

    def request(self, path, fields=None, headers=None):
        if self.block:
            self.entered.set()
            self.unblock.wait(2)
        if self.fail:
            raise LinkError("offline")
        if fields is None:
            return copy.deepcopy(self.state)
        self.sent.append(copy.deepcopy(fields))
        assert headers["X-Token"] == self.state["token"]
        assert fields["epoch"] == self.state["epoch"]
        op = fields["op"]
        if op != "beat":
            self.state["epoch"] += 1
            self.state["owner"] = headers["X-Client"]
        if op == "arm":
            self.state["armed"] = 1
        if op == "grip":
            self.state["gripping"] = 1
        if op == "stop":
            self.state["gripping"] = 0
        if op == "release":
            self.state.update(armed=0, gripping=0, owner="")
        if self.drop_ack:
            raise LinkError("lost response")
        return {"ok": True}


class HandTests(unittest.TestCase):
    def setUp(self):
        self.board = FakeBoard()
        self.hand = HandDevice(self.board)
        self.hand.connect()

    def tearDown(self):
        self.board.unblock.set()
        self.hand.disconnect()

    def send(self, op, **params):
        return self.hand.command(dict(op=op, revision=self.hand.snapshot()["revision"], **params))

    def test_commands_and_no_configuration_or_credentials(self):
        self.assertEqual(self.board.sent, [])
        self.send("arm")
        self.send("pose", targets=[0, 20, 60, 100])
        self.send("grip")
        self.send("stop")
        self.send("release")
        self.assertEqual([x["op"] for x in self.board.sent], ["arm", "pose", "grip", "stop", "release"])
        self.assertEqual([self.board.sent[1][f"q{i}"] for i in range(4)], [0, 20, 60, 100])
        snapshot = self.hand.snapshot()
        self.assertEqual(snapshot["fingers"][0]["load"], -210)
        self.assertEqual(snapshot["fingers"][0]["limit"], 300)
        self.assertNotIn("test-token", json.dumps(snapshot))
        self.assertNotIn(self.hand.client, json.dumps(snapshot))

    def test_other_owner_blocks_even_stop_and_release(self):
        self.board.state.update(armed=1, owner="other-client")
        self.hand.connect()
        for op in ("arm", "pose", "grip", "stop", "release"):
            with self.assertRaisesRegex(RuntimeError, "另一终端"):
                self.send(op, **({"targets":[0]*4} if op == "pose" else {}))
        self.assertEqual(self.board.sent, [])

    def test_stale_feedback_epoch_and_duplicate_commands(self):
        self.hand.sample_at -= 3
        with self.assertRaises(RuntimeError):
            self.send("arm")
        self.hand.connect()
        old = self.hand.snapshot()["revision"]
        self.board.state["epoch"] += 1
        with self.assertRaisesRegex(RuntimeError, "已变化"):
            self.hand.command(dict(op="arm", revision=old))
        old = self.hand.snapshot()["revision"]
        self.send("arm")
        with self.assertRaises(RuntimeError):
            self.hand.command(dict(op="arm", revision=old))
        self.assertEqual(len(self.board.sent), 1)

    def test_lost_ack_no_replay_and_restart_guard(self):
        self.board.drop_ack = True
        with self.assertRaisesRegex(RuntimeError, "结果未知"):
            self.send("arm")
        self.hand.disconnect()
        with self.assertRaises(RuntimeError):
            with self.hand.backend_exit():
                pass
        self.board.drop_ack = False
        self.hand.connect()
        self.assertEqual(len(self.board.sent), 1)
        self.send("release")
        with self.hand.backend_exit():
            pass
        with self.assertRaises(RuntimeError):
            self.hand.connect()

    def test_disconnect_reconnect_and_validation(self):
        for q in ([True]*4, [101]*4, [1,2], ["0"]*4):
            with self.assertRaises(ValueError):
                self.send("pose", targets=q)
        with self.assertRaises(ValueError):
            self.send("config")
        self.send("arm")
        self.hand.disconnect()
        self.assertEqual(self.board.state["armed"], 1)
        self.hand.connect()
        self.assertEqual(len(self.board.sent), 1)

    def test_legacy_heartbeat_only_after_own_enable(self):
        self.board.state["disconnectRelease"] = 1
        self.send("arm")
        time.sleep(.45)
        self.assertTrue(any(x["op"] == "beat" for x in self.board.sent))

    def test_full_console_commands_use_original_board_fields(self):
        self.send("speed", speed=400)
        self.send("save", name="握姿", targets=[10, 20, 30, 40])
        self.send("delete", slot=3)
        self.send("scan")
        self.send("set_id", old_id=1, new_id=5)
        cfg = [dict(id=i+1, open=500, close=900, limit=300, speedSet=350) for i in range(4)]
        self.send("config", configs=cfg, confirmed=True)
        self.assertEqual([x["op"] for x in self.board.sent], ["speed","save","delete","scan","set_id","config"])
        self.assertEqual(self.board.sent[1]["q3"], 40)
        self.assertEqual(self.board.sent[-1]["speed2"], 350)
        self.assertNotIn("configs", self.board.sent[-1])
        self.assertEqual(self.hand.snapshot()["gestures"], [])

    def test_configuration_guards_and_validation(self):
        cfg = [dict(id=i+1, open=500, close=900, limit=300) for i in range(4)]
        for bad in (dict(op="speed",speed=501), dict(op="save",name="a"*49,targets=[0]*4),
                    dict(op="set_id",old_id=1,new_id=1), dict(op="config",configs=cfg,confirmed=False)):
            op = bad.pop("op")
            with self.assertRaises(ValueError):
                self.send(op, **bad)
        self.send("arm")
        for op in ("config","scan","set_id"):
            params = {"configs":cfg,"confirmed":True} if op == "config" else {"old_id":1,"new_id":5} if op == "set_id" else {}
            with self.assertRaisesRegex(RuntimeError, "关闭手部驱动"):
                self.send(op, **params)
        self.board.state["fingers"][0]["phase"] = "moving"
        with self.assertRaisesRegex(RuntimeError, "停止并保持"):
            self.send("speed", speed=400)
        self.assertEqual([x["op"] for x in self.board.sent], ["arm"])

    def test_network_failure_hides_old_feedback_and_no_commands(self):
        self.board.fail = True
        with self.assertRaises(RuntimeError):
            self.hand.connect()
        self.assertFalse(self.hand.snapshot()["connected"])
        self.assertEqual(self.hand.snapshot()["fingers"], [])
        self.assertEqual(self.board.sent, [])


class HttpIsolationTests(unittest.TestCase):
    def test_real_transport_form_encoding_and_no_redirect_following(self):
        received = []
        class BoardHandler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                received.append((self.path, self.headers.get("X-Token"),
                                 parse_qs(self.rfile.read(int(self.headers["Content-Length"])).decode())))
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'{"ok":true}')

            def do_GET(self):
                self.send_response(302)
                self.send_header("Location", "/should-not-follow")
                self.end_headers()
                self.wfile.write(b'{}')

        http = ThreadingHTTPServer(("127.0.0.1", 0), BoardHandler)
        threading.Thread(target=http.serve_forever, daemon=True).start()
        try:
            board = HandBoard("127.0.0.1", http.server_port)
            self.assertTrue(board.request("/api/command", {"op":"pose", "q0":20},
                                          {"X-Token":"test-token"})["ok"])
            self.assertEqual(received, [("/api/command", "test-token", {"op":["pose"], "q0":["20"]})])
            with self.assertRaises(LinkError):
                board.request("/api/state")
        finally:
            http.shutdown()
            http.server_close()

    def test_hand_wait_never_blocks_arm_stop_or_state(self):
        board = FakeBoard()
        hand = HandDevice(board)
        twin = Mock()
        twin.snapshot.return_value = {"connected":False}
        twin.emergency_stop.return_value = None
        twin.lifecycle_lock = threading.RLock()
        twin.shutting_down = False
        http = ThreadingHTTPServer(("127.0.0.1", 0), server.DashboardHandler)
        port = http.server_port
        opener = build_opener(ProxyHandler({}))
        def request(path, body=None):
            with opener.open(Request(f"http://127.0.0.1:{port}"+path,
                                     data=None if body is None else json.dumps(body).encode()), timeout=3) as r:
                return json.load(r)
        def connect():
            request("/api/hand/connect", {})
        with patch.object(server, "PORT", port), patch.object(server, "TWIN", twin), patch.object(server, "HAND", hand):
            threading.Thread(target=http.serve_forever, daemon=True).start()
            board.block = True
            thread = threading.Thread(target=connect)
            thread.start()
            try:
                self.assertTrue(board.entered.wait(1))
                started = time.monotonic()
                self.assertEqual(request("/api/state"), {"connected":False})
                self.assertTrue(request("/api/stop", {})["ok"])
                self.assertFalse(request("/api/hand/state")["connected"])
                self.assertLess(time.monotonic() - started, .5)
                twin.emergency_stop.assert_called_once()
            finally:
                board.unblock.set()
                thread.join(3)
                hand.disconnect()
                http.shutdown()
                http.server_close()


if __name__ == "__main__":
    unittest.main()
