"""Link proxy: stands in for one satellite link between the boat and the shore server.

Listens on one port and forwards every request (method, path, body) to the shore server, then passes
the answer back. Kill it and that link is down; start it again and the boat's queue drains.

Run one per link:
    python3 shore/proxy.py --name starlink --listen 8001     # -> shore on :8000
    python3 shore/proxy.py --name iridium  --listen 8002

If the shore server itself is down the proxy answers 502, so the boat keeps the data queued.
"""
import argparse
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def make_handler(name, upstream):
    class Forward(BaseHTTPRequestHandler):
        def _forward(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(n) if n else None
            req = urllib.request.Request(upstream + self.path, data=body, method=self.command)
            if self.headers.get("Content-Type"):
                req.add_header("Content-Type", self.headers["Content-Type"])
            try:
                with urllib.request.urlopen(req, timeout=10) as r:
                    status, ctype, out = r.status, r.headers.get("Content-Type"), r.read()
            except urllib.error.HTTPError as e:
                status, ctype, out = e.code, e.headers.get("Content-Type"), e.read()
            except OSError as e:
                status, ctype, out = 502, "text/plain", f"{name} proxy: shore unreachable ({e})".encode()
            self.send_response(status)
            self.send_header("Content-Type", ctype or "application/octet-stream")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)

        do_GET = do_POST = _forward

        def log_message(self, fmt, *args):
            print(f"{name}: {self.command} {self.path} -> {args[1] if len(args) > 1 else ''}", flush=True)

    return Forward


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True, help="starlink | iridium (only used in the log)")
    ap.add_argument("--listen", type=int, required=True)
    ap.add_argument("--to", default="http://127.0.0.1:8000")
    a = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", a.listen), make_handler(a.name, a.to.rstrip("/")))
    print(f"{a.name} proxy: :{a.listen} -> {a.to}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
