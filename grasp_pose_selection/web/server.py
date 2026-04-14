#!/usr/bin/env python3
"""本地 API + 静态资源。运行: python server.py

- 经典单页: http://127.0.0.1:8765/test.html
- React 构建产物: 在 ../webapp 执行 npm run build 后访问 http://127.0.0.1:8765/app/
- Render: 设置环境变量 PORT；若存在 RENDER 则默认监听 0.0.0.0。也可显式设置 GRASP_WEB_HOST。
"""
from __future__ import annotations

import json
import os
import time
import traceback
import uuid
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import unquote

from grasp_web_compute import compute_web_demo, default_payload, inspect_clearance, preview_payload

WEB_DIR = Path(__file__).resolve().parent
WEBAPP_DIST = WEB_DIR.parent / "webapp" / "dist"
PORT = int(os.environ.get("PORT", "8765"))
HOST = os.environ.get(
    "GRASP_WEB_HOST",
    "0.0.0.0" if os.environ.get("RENDER") else "127.0.0.1",
)


def _content_type(path: Path) -> str:
    s = path.suffix.lower()
    if s == ".html":
        return "text/html; charset=utf-8"
    if s == ".js":
        return "application/javascript; charset=utf-8"
    if s == ".css":
        return "text/css; charset=utf-8"
    if s == ".json":
        return "application/json; charset=utf-8"
    if s in (".png",):
        return "image/png"
    if s in (".svg",):
        return "image/svg+xml"
    if s in (".ico",):
        return "image/x-icon"
    return "application/octet-stream"


def _safe_under(root: Path, candidate: Path) -> bool:
    try:
        r = root.resolve()
        c = candidate.resolve()
    except OSError:
        return False
    return c == r or r in c.parents


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print(fmt % args)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")

    def _json(self, code: int, obj: object) -> None:
        b = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self._cors()
        self.end_headers()
        self.wfile.write(b)

    def _file(self, path: Path, content_type: str) -> None:
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self._cors()
        self.end_headers()
        self.wfile.write(data)

    def _post_json(self) -> dict:
        n = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(n).decode("utf-8")
        return json.loads(raw) if raw.strip() else {}

    def _compute_response(self, payload: dict, preview: bool) -> dict:
        pl = preview_payload(payload) if preview else payload
        data = compute_web_demo(pl)
        return {
            "ok": True,
            "data": data,
            "job": {
                "id": str(uuid.uuid4()),
                "preview": preview,
                "ts": time.time(),
            },
        }

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        path_only = unquote(self.path.split("?", 1)[0])

        if path_only in ("/", "/test.html"):
            self._file(WEB_DIR / "test.html", "text/html; charset=utf-8")
            return
        if path_only == "/api/defaults":
            self._json(200, {"ok": True, "data": default_payload()})
            return

        if path_only == "/app" or path_only == "/app/":
            if (WEBAPP_DIST / "index.html").is_file():
                self._file(WEBAPP_DIST / "index.html", "text/html; charset=utf-8")
                return
            self.send_error(404, "webapp not built; run npm run build in webapp/")
            return

        if path_only.startswith("/app/") and WEBAPP_DIST.is_dir():
            rel = path_only[len("/app/") :].lstrip("/")
            if not rel:
                self._file(WEBAPP_DIST / "index.html", "text/html; charset=utf-8")
                return
            p = WEBAPP_DIST / rel
            if p.is_file() and _safe_under(WEBAPP_DIST, p):
                self._file(p, _content_type(p))
                return
            self._file(WEBAPP_DIST / "index.html", "text/html; charset=utf-8")
            return

        rel = path_only.lstrip("/")
        if rel and ".." not in rel and "/" not in rel:
            p = WEB_DIR / rel
            if p.is_file() and p.parent == WEB_DIR:
                self._file(p, _content_type(p))
                return
        self.send_error(404)

    def do_POST(self) -> None:
        path_only = self.path.split("?", 1)[0]
        if path_only == "/api/inspect-clearance":
            try:
                body = self._post_json()
                payload = body["payload"]
                q1 = float(body["q1"])
                q2 = float(body["q2"])
                data = inspect_clearance(payload, q1, q2)
                self._json(
                    200,
                    {
                        "ok": True,
                        "data": data,
                        "job": {"id": str(uuid.uuid4()), "preview": False, "ts": time.time()},
                    },
                )
            except Exception as e:
                tb = traceback.format_exc()
                print(tb)
                self._json(200, {"ok": False, "error": str(e), "traceback": tb})
            return
        if path_only == "/api/compute-preview":
            preview = True
        elif path_only in ("/api/compute", "/api/compute-full"):
            preview = False
        else:
            self.send_error(404)
            return
        try:
            payload = self._post_json()
            out = self._compute_response(payload, preview=preview)
            self._json(200, out)
        except Exception as e:
            tb = traceback.format_exc()
            print(tb)
            self._json(200, {"ok": False, "error": str(e), "traceback": tb})


def main() -> None:
    httpd = HTTPServer((HOST, PORT), Handler)
    print(f"Listening on http://{HOST}:{PORT}/")
    print(f"Grasp web demo: http://127.0.0.1:{PORT}/test.html")
    if (WEBAPP_DIST / "index.html").is_file():
        print(f"React app:      http://127.0.0.1:{PORT}/app/")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
