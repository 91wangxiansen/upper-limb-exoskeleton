"""Independent Hand-Exo HTTP channel; credentials stay in backend memory."""
import copy
import http.client
import json
import secrets
import threading
import time
from contextlib import contextmanager
from urllib.parse import urlencode


class LinkError(RuntimeError):
    pass


class HandBoard:
    def __init__(self, host="192.168.4.1", port=80):
        self.host, self.port = host, port

    def request(self, path, fields=None, headers=None):
        # Direct socket ignores system proxies and does not follow redirects.
        timeout = 4 if fields and fields.get("op") in ("scan", "set_id") else 1
        conn = http.client.HTTPConnection(self.host, self.port, timeout=timeout)
        try:
            conn.request("GET" if fields is None else "POST", path,
                         None if fields is None else urlencode(fields),
                         {"Content-Type": "application/x-www-form-urlencoded", **(headers or {})})
            response = conn.getresponse()
            payload = json.loads(response.read(65537))
            if not isinstance(payload, dict):
                raise ValueError()
            if response.status != 200:
                if response.status in (400, 403, 409):
                    raise RuntimeError(str(payload.get("error", "控制板拒绝指令")))
                raise LinkError("控制板响应异常")
            return payload
        except (OSError, http.client.HTTPException, ValueError) as exc:
            raise LinkError("手部通信超时或响应无效，请检查 Hand-Exo Wi-Fi") from exc
        finally:
            conn.close()


FINGER_FIELDS = ("id", "online", "verified", "p", "q", "phase", "load", "speed",
                 "temp", "v", "alarm", "limit", "speedSet", "open", "close",
                 "readError", "failures", "mode", "contactRawLoad", "contactFilteredLoad",
                 "contactVotes", "contactNoProgressMs", "contactPositionError", "commandedP", "contactSampleMs")
STATE_FIELDS = ("speedControl", "contactFilter", "disconnectRelease", "leaseMs",
                "scanDone", "idMessage", "detectedIDs", "gestures")


def integer(value, low, high, label):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{label}范围为 {low}～{high} 的整数")
    return value


def command_fields(body):
    op = body.get("op")
    if op not in ("arm", "grip", "pose", "stop", "release", "save", "delete",
                  "speed", "config", "scan", "set_id"):
        raise ValueError("未知手部操作")
    keys, fields = {"op", "revision"}, {"op": op}
    if op in ("pose", "save"):
        keys.add("targets")
        q = body.get("targets")
        if not isinstance(q, list) or len(q) != 4:
            raise ValueError("需要四指目标")
        fields.update({f"q{i}": integer(v, 0, 100, "闭合量") for i, v in enumerate(q)})
    if op == "save":
        keys.add("name")
        name = body.get("name")
        if not isinstance(name, str) or not name.strip() or len(name.strip().encode("utf-8")) > 48 or any(ord(c) < 32 for c in name):
            raise ValueError("手势名称需要1～48字节且不含控制字符")
        fields["name"] = name.strip()
    if op == "delete":
        keys.add("slot")
        fields["slot"] = integer(body.get("slot"), 0, 15, "手势槽位")
    if op == "speed":
        keys.add("speed")
        fields["speed"] = integer(body.get("speed"), 20, 500, "速度")
    if op == "set_id":
        keys.update(("old_id", "new_id"))
        for k in ("old_id", "new_id"):
            fields[k] = integer(body.get(k), 1, 20, "舵机ID")
        if fields["old_id"] == fields["new_id"]:
            raise ValueError("原ID与新ID必须不同")
    if op == "config":
        keys.update(("configs", "confirmed"))
        configs = body.get("configs")
        if body.get("confirmed") is not True or not isinstance(configs, list) or len(configs) != 4:
            raise ValueError("需要确认四指方向和行程并提交四组校准")
        for i, cfg in enumerate(configs):
            if not isinstance(cfg, dict) or set(cfg) not in ({"id", "open", "close", "limit"}, {"id", "open", "close", "limit", "speedSet"}):
                raise ValueError("校准字段无效")
            for k, low, high in (("id", 1, 20), ("open", 0, 1023), ("close", 0, 1023), ("limit", 50, 1000)):
                fields[f"{k}{i}"] = integer(cfg.get(k), low, high, k)
            if abs(cfg["open"]-cfg["close"]) < 20:
                raise ValueError("行程至少相差20")
            if "speedSet" in cfg:
                fields[f"speed{i}"] = integer(cfg["speedSet"], 20, 500, "速度")
        if len({c["id"] for c in configs}) != 4:
            raise ValueError("四指ID必须互异")
    if set(body) != keys or type(body.get("revision")) is not int:
        raise ValueError("手部请求字段无效")
    return fields


class HandDevice:
    def __init__(self, board=None):
        self.board = board or HandBoard()
        self.client = secrets.token_hex(16)
        self.io = threading.Lock()
        self.lock = threading.RLock()
        self.stop_poll = None
        self.closed = False
        self.revision = secrets.randbits(48)
        self.raw = None
        self.error = ""
        self.sample_at = 0.
        self.may_own_drive = False

    def _invalidate(self, error):
        with self.lock:
            self.revision += 1
            self.raw = None
            self.error = error

    def _refresh(self):
        state = self.board.request("/api/state")
        if not (isinstance(state.get("token"), str) and state["token"]
                and isinstance(state.get("owner"), str) and type(state.get("epoch")) is int
                and all(type(state.get(k)) in (int, bool) and state[k] in (0, 1)
                        for k in ("armed", "ready", "gripping"))
                and isinstance(state.get("fingers"), list) and len(state["fingers"]) == 4
                and all(isinstance(f, dict) and "id" in f and "online" in f for f in state["fingers"])):
            raise LinkError("控制板状态格式不兼容")
        with self.lock:
            keys = ("token", "epoch", "owner", "armed", "gripping")
            if self.raw is None or any(state[k] != self.raw[k] for k in keys):
                self.revision += 1
            self.raw, self.error, self.sample_at = state, "", time.monotonic()
            self.may_own_drive = bool(state["armed"] and state["owner"] == self.client)

    def _post(self, fields):
        reply = self.board.request("/api/command", {**fields, "epoch": self.raw["epoch"]},
                                   {"X-Token": self.raw["token"], "X-Client": self.client})
        if reply.get("ok") is not True:
            raise LinkError("控制板未确认指令")

    def _poll(self, stop):
        while not stop.wait(0.35):
            with self.io:
                if stop.is_set():
                    return
                try:
                    self._refresh()
                    if self.may_own_drive and self.raw.get("disconnectRelease", 1):
                        self._post({"op": "beat"})
                except RuntimeError as exc:
                    self._invalidate(str(exc))

    def connect(self):
        with self.io:
            if self.closed:
                raise RuntimeError("后台正在退出")
            if self.stop_poll is None:
                self.stop_poll = threading.Event()
                threading.Thread(target=self._poll, args=(self.stop_poll,), daemon=True).start()
            try:
                self._refresh()
            except RuntimeError as exc:
                self._invalidate(str(exc))
                raise
            return self.snapshot()

    def _disconnect(self):
        with self.lock:
            if self.stop_poll:
                self.stop_poll.set()
            self.stop_poll = None
            self._invalidate("")

    def disconnect(self):
        with self.io:
            self._disconnect()
            return self.snapshot()

    @contextmanager
    def backend_exit(self):
        with self.io:
            if self.may_own_drive:
                raise RuntimeError("手部驱动仍可能由本机启用，请核对反馈并关闭手部驱动后再重启后台")
            yield
            self.closed = True
            self._disconnect()

    def snapshot(self):
        with self.lock:
            result = {"monitoring": self.stop_poll is not None, "connected": False,
                      "revision": self.revision, "error": self.error, "fingers": []}
            state = self.raw
            if state is None:
                return result
            age = time.monotonic() - self.sample_at
            if age > 2:
                return {**result, "error": "手部反馈已过期"}
            ownership = "local" if state["owner"] == self.client else "other" if state["owner"] else "free"
            return {**result, "connected": True, "ageMs": round(age * 1000), "ownership": ownership,
                    "armed": bool(state["armed"]), "ready": bool(state["ready"]),
                    "gripping": bool(state["gripping"]), "fault": state.get("fault", ""),
                    **{k: copy.deepcopy(state[k]) for k in STATE_FIELDS if k in state},
                    "fingers": [{k: copy.deepcopy(f[k]) for k in FINGER_FIELDS if k in f} for f in state["fingers"]]}

    def command(self, body):
        fields = command_fields(body)
        op = fields["op"]
        if not self.io.acquire(blocking=False):
            raise RuntimeError("手部通信忙，请稍后重新操作")
        try:
            if self.closed or not self.stop_poll or not self.snapshot()["connected"]:
                raise RuntimeError("请先连接手部并等待新鲜反馈")
            try:
                self._refresh()
            except RuntimeError as exc:
                self._invalidate(str(exc))
                raise
            if self.revision != body["revision"]:
                raise RuntimeError("手部状态已变化，请核对后重新操作")
            state = self.raw
            if state["owner"] and state["owner"] != self.client:
                raise RuntimeError("另一终端持有手部控制权")
            if op == "arm":
                if state["armed"] or not state["ready"]:
                    raise RuntimeError("手部尚未就绪或已经启用")
            elif op in ("pose", "grip") and not self.may_own_drive:
                raise RuntimeError("请先启用手部驱动")
            elif op in ("pose", "grip") and state["gripping"]:
                raise RuntimeError("抓握进行中，请先停止并保持")
            if op in ("config", "scan", "set_id") and (state["armed"] or state["gripping"]):
                raise RuntimeError("请先关闭手部驱动再配置或修改ID")
            if op == "speed" and (state["gripping"] or any(f.get("phase") in ("moving", "gripping") for f in state["fingers"])):
                raise RuntimeError("请先停止并保持，再修改速度")
            if op == "speed" and not state.get("speedControl"):
                raise RuntimeError("当前固件不支持调速")
            with self.lock:
                self.revision += 1
                if op == "arm":
                    self.may_own_drive = True
            try:
                self._post(fields)
            except LinkError as exc:
                self._invalidate("指令结果未知，请核对反馈，勿重复发送")
                raise RuntimeError(self.error) from exc
            try:
                self._refresh()
            except RuntimeError as exc:
                self._invalidate("指令已确认，后续反馈读取失败")
                raise RuntimeError(self.error) from exc
            return self.snapshot()
        finally:
            self.io.release()
