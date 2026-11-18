"""Integration checks use a loopback fake ESP32, never physical hardware."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import build_opener, ProxyHandler, Request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from debug_bridge import make_server


class FakeESP(BaseHTTPRequestHandler):
    received = []

    def do_GET(self):
        payload = b'{"token":"fake-token","fingers":[],"armed":0}'
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self):
        body = self.rfile.read(int(self.headers['Content-Length']))
        self.received.append((body, self.headers.get('X-Token'), self.headers.get('X-Client')))
        self.send_response(409)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"error":"fake device rejects change"}')

    def log_message(self, *args):
        pass


class BridgeTest(unittest.TestCase):
    def test_proxy_contract(self):
        device = ThreadingHTTPServer(('127.0.0.1', 0), FakeESP)
        bridge = make_server(0, f'http://127.0.0.1:{device.server_port}')
        for server in (device, bridge):
            threading.Thread(target=server.serve_forever, daemon=True).start()
        client = build_opener(ProxyHandler({}))
        base = f'http://127.0.0.1:{bridge.server_port}'
        try:
            with client.open(base + '/bridge/health', timeout=3) as r:
                self.assertEqual(json.load(r)['bridge'], 'running')
            with client.open(base + '/', timeout=3) as r:
                self.assertIn('关节孪生'.encode(), r.read())
            with client.open(base + '/api/state', timeout=3) as r:
                self.assertEqual(json.load(r)['token'], 'fake-token')
            body = b'op=config&id0=1'
            request = Request(base + '/api/command', data=body, headers={
                'Content-Type': 'application/x-www-form-urlencoded',
                'X-Token': 'test-token', 'X-Client': 'test-client'})
            with self.assertRaises(HTTPError) as error:
                client.open(request, timeout=3)
            self.assertEqual(error.exception.code, 409)
            self.assertEqual(FakeESP.received[-1], (body, 'test-token', 'test-client'))
            request = Request(base + '/api/state', headers={'Origin': 'https://example.com'})
            with self.assertRaises(HTTPError) as error:
                client.open(request, timeout=3)
            self.assertEqual(error.exception.code, 403)
            with self.assertRaises(HTTPError) as error:
                client.open(base + '/api/unknown', timeout=3)
            self.assertEqual(error.exception.code, 404)
        finally:
            for server in (bridge, device):
                server.shutdown()
                server.server_close()


if __name__ == '__main__':
    unittest.main()
