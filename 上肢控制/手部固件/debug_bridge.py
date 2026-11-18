"""Serve the local UI and forward its API to the ESP32 on the PC's Wi-Fi.

Run from Windows Explorer using debug_bridge.bat. Starting the bridge only
serves requests. It never enables motors or changes device configuration.
"""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import build_opener, HTTPRedirectHandler, ProxyHandler, Request
import webbrowser


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class BridgeHandler(SimpleHTTPRequestHandler):
    server_version = 'HandDebug/1.0'

    def local_request(self):
        host = self.headers.get('Host', '')
        allowed = {f'127.0.0.1:{self.server.server_port}',
                   f'localhost:{self.server.server_port}'}
        if host not in allowed:
            self.send_error(403, 'Use localhost or 127.0.0.1')
            return False
        origin = self.headers.get('Origin')
        if origin and origin not in {'http://' + h for h in allowed}:
            self.send_error(403, 'Cross-origin requests are disabled')
            return False
        return True

    def json_reply(self, code, data):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def proxy(self, body=None):
        if not self.server.api_lock.acquire(timeout=0.2):
            self.json_reply(503, {'error': '控制板请求处理中，请稍后重试'})
            return
        try:
            headers = {}
            if body is not None:
                headers['Content-Type'] = 'application/x-www-form-urlencoded'
                for key in ('X-Token', 'X-Client'):
                    headers[key] = self.headers.get(key, '')
            request = Request(self.server.upstream + self.path,
                              data=body, headers=headers,
                              method='GET' if body is None else 'POST')
            opener = build_opener(ProxyHandler({}), NoRedirect())
            try:
                response = opener.open(request, timeout=1.8)
            except HTTPError as exc:
                response = exc
            with response:
                data = response.read(65537)
                if len(data) > 65536:
                    self.json_reply(502, {'error': '控制板响应超出允许长度'})
                    return
                self.send_response(response.code)
                self.send_header('Content-Type', response.headers.get(
                    'Content-Type', 'application/json'))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
        except (URLError, TimeoutError, socket.timeout, ConnectionError, OSError) as exc:
            print(f'ESP connection failed: {exc}', flush=True)
            self.json_reply(502, {'error': '电脑无法连接 ESP32，请确认 Wi-Fi 已连接 Hand-Exo'})
        finally:
            self.server.api_lock.release()

    def do_GET(self):
        if not self.local_request():
            return
        if self.path == '/api/state':
            self.proxy()
        elif self.path == '/bridge/health':
            self.json_reply(200, {'bridge': 'running', 'upstream': self.server.upstream})
        elif self.path.startswith('/api/'):
            self.json_reply(404, {'error': '接口不存在'})
        else:
            super().do_GET()

    def do_POST(self):
        if not self.local_request():
            return
        if self.path != '/api/command':
            self.json_reply(404, {'error': '接口不存在'})
            return
        try:
            length = int(self.headers.get('Content-Length', '-1'))
        except ValueError:
            length = -1
        if length < 0 or length > 4096:
            self.json_reply(400, {'error': '请求长度无效'})
            return
        if not self.headers.get('Content-Type', '').startswith(
                'application/x-www-form-urlencoded'):
            self.json_reply(415, {'error': '请求类型无效'})
            return
        self.connection.settimeout(3)
        try:
            body = self.rfile.read(length)
        except (TimeoutError, OSError):
            self.json_reply(408, {'error': '请求正文超时'})
            return
        if len(body) != length:
            self.json_reply(400, {'error': '请求正文不完整'})
            return
        self.proxy(body)

    def list_directory(self, path):
        self.send_error(403, 'Directory listing is disabled')
        return None


def make_server(port=8767, upstream='http://192.168.4.1'):
    parsed = urlsplit(upstream)
    if (parsed.scheme != 'http' or parsed.hostname not in ('192.168.4.1', '127.0.0.1')
            or parsed.username or parsed.password or parsed.path not in ('', '/')
            or parsed.query or parsed.fragment):
        raise ValueError('Upstream must be the ESP32 or a loopback test server')
    root = Path(__file__).resolve().parent / 'web'
    server = ThreadingHTTPServer(('127.0.0.1', port), partial(BridgeHandler, directory=str(root)))
    server.upstream = upstream.rstrip('/')
    server.api_lock = threading.Lock()
    return server


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--no-browser', action='store_true')
    options = parser.parse_args()
    try:
        httpd = make_server(options.port)
    except OSError as exc:
        raise SystemExit(f'Cannot start debug bridge: {exc}')
    address = f'http://127.0.0.1:{httpd.server_port}'
    print(f'ESP32 debug page: {address}', flush=True)
    print('Connect this PC Wi-Fi to Hand-Exo. Keep this window open.', flush=True)
    print('This page uses live ESP32 feedback. Internet access is needed separately for Codex.', flush=True)
    if not options.no_browser:
        webbrowser.open(address)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
