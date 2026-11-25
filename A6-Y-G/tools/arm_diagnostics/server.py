#!/usr/bin/env python3
"""Loopback-only UI for the role-selected, status-only scanner."""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import webbrowser

from scanner import DEFAULT_REPORTS, DEFAULT_ROLES, inventory, load_roles
from session import ScanSession



def make_server(session, port=8765):
    page = Path(__file__).with_name("ui") / "index.html"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, code, body, mime="application/json; charset=utf-8", download=False):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
            if download:
                self.send_header("Content-Disposition", 'attachment; filename="arm_scan_report.json"')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def valid_host(self):
            port = self.server.server_address[1]
            return self.headers.get("Host") in {f"127.0.0.1:{port}", f"localhost:{port}"}

        def do_GET(self):
            if not self.valid_host():
                return self.respond(403, {"error": "仅接受本机地址"})
            if self.path == "/":
                return self.respond(200, page.read_bytes(), "text/html; charset=utf-8")
            if self.path == "/api/init":
                return self.respond(200, {"arms": inventory(session.roles), "token": session.token})
            if self.path == "/api/state":
                return self.respond(200, session.snapshot())
            if self.path == "/report.json":
                report = session.snapshot()["report"]
                return self.respond(200, report, download=True) if report else self.respond(404, {"error": "尚无扫描报告"})
            self.respond(404, {"error": "地址不存在"})

        def do_POST(self):
            origin = self.headers.get("Origin")
            allowed_origins = {f"http://{host}:{self.server.server_address[1]}" for host in ("127.0.0.1", "localhost")}
            if (not self.valid_host() or origin not in allowed_origins
                    or not secrets.compare_digest(self.headers.get("X-Scan-Token", ""), session.token)):
                return self.respond(403, {"error": "请求来源校验失败，请刷新本机页面"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096 or self.headers.get_content_type() != "application/json":
                    raise ValueError("请求格式无效")
                self.connection.settimeout(3)
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("请求必须为对象")
                if self.path == "/api/scan":
                    if set(data) != {"arms", "passes"}:
                        raise ValueError("请明确提供 arms 和 passes")
                    session.start(data["arms"], data["passes"])
                elif self.path == "/api/stop":
                    session.cancel.set()
                else:
                    return self.respond(404, {"error": "地址不存在"})
                self.respond(202, {"ok": True})
            except RuntimeError as exc:
                self.respond(409, {"error": str(exc)})
            except (ValueError, TypeError, OSError) as exc:
                self.respond(400, {"error": str(exc)})

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description="A6Y 七电机扫描面板（仅本机访问）")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--roles", type=Path, default=DEFAULT_ROLES)
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORTS)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("端口必须为 1～65535")
    try:
        session = ScanSession(load_roles(args.roles), args.report_dir)
        server = make_server(session, args.port)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(2, f"启动失败：{exc}\n")
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"机械臂扫描面板：{url}\n仅状态查询；打开页面不会扫描或打开串口。\nCtrl+C 关闭面板服务。", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        print("\n正在关闭扫描面板…", flush=True)
    finally:
        session.cancel.set()
        if session.worker:
            session.worker.join(timeout=6)
        server.server_close()


if __name__ == "__main__":
    main()
