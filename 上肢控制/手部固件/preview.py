"""Local preview for desktop or phone on the same WLAN. No hardware connection."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import webbrowser

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('--port', type=int, default=8766)
    args.add_argument('--lan', action='store_true', help='listen on local network for phone preview')
    args.add_argument('--no-browser', action='store_true')
    opt = args.parse_args()
    root = Path(__file__).resolve().parent / 'web'
    server = ThreadingHTTPServer(('0.0.0.0' if opt.lan else '127.0.0.1', opt.port),
                                 partial(SimpleHTTPRequestHandler, directory=str(root)))
    url = f'http://127.0.0.1:{opt.port}/?demo=1'
    print(url, flush=True)
    if not opt.no_browser: webbrowser.open(url)
    server.serve_forever()

