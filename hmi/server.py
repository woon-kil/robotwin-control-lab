"""Operator HMI HTTP view. 실행 판단은 OperatorSession이 한다."""

from __future__ import annotations

import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from hmi.actions import OperatorSession
from hmi.render import render

HOST = "127.0.0.1"
PORT = 8765


class Handler(BaseHTTPRequestHandler):
    session = OperatorSession()

    def do_GET(self) -> None:
        if self.path.split("?", 1)[0] != "/":
            self.send_error(404)
            return
        self._send(render(self.session.snapshot))

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        length = int(self.headers.get("Content-Length", "0"))
        form = urllib.parse.parse_qs(self.rfile.read(length).decode("utf-8"))
        if path == "/select":
            self.session.select(form["case"][0])
        elif path == "/validate":
            self.session.validate()
        elif path == "/execute":
            self.session.execute()
        elif path == "/fault":
            self.session.fault()
        else:
            self.send_error(404)
            return
        self.send_response(303)
        self.send_header("Location", "/")
        self.end_headers()

    def _send(self, page: str) -> None:
        body = page.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def main() -> None:
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
