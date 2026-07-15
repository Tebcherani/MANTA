#!/usr/bin/env python3
"""Simple local web app for Odyssey mission construction and ksim verification."""

import argparse
import datetime
import http.server
import json
import os
import pathlib
import subprocess
import sys
import urllib.parse

APP_DIR = pathlib.Path(__file__).resolve().parent
ROOT_DIR = APP_DIR.parent
HTML_FILE = APP_DIR / "mission_constructor.html"
RUN_DIR = APP_DIR / "ksim_runs"
RUN_DIR.mkdir(parents=True, exist_ok=True)


def find_ksim_executable():
    for candidate in ("ksim_bin", "ksim.exe", "ksim"):
        path = ROOT_DIR / candidate
        if path.exists() and path.is_file():
            return str(path)
    raise FileNotFoundError("Could not find ksim executable in the repository root. Build ksim first.")


class MissionConstructorHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path in ("/", "/mission_constructor.html"):
            self.serve_file(HTML_FILE, "text/html; charset=utf-8")
            return
        self.send_error(404, "Not found")

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/run-ksim":
            self.handle_run_ksim()
            return
        self.send_error(404, "Not found")

    def handle_run_ksim(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8")
        try:
            payload = json.loads(body)
        except json.JSONDecodeError:
            return self.send_json({"success": False, "error": "Invalid JSON payload."}, status=400)

        mission_text = payload.get("mission", "")
        filename = pathlib.Path(payload.get("filename", "mission.mi")).name or "mission.mi"
        if not mission_text.strip():
            return self.send_json({"success": False, "error": "Mission script is empty."}, status=400)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        mission_file = RUN_DIR / f"{timestamp}_{filename}"
        behavior_file = RUN_DIR / f"behavior_{timestamp}.json"
        mission_file.write_text(mission_text, encoding="utf-8")

        try:
            ksim_path = find_ksim_executable()
        except FileNotFoundError as exc:
            return self.send_json({"success": False, "error": str(exc)}, status=500)

        command = [ksim_path, "--nolog", "--dump-behavior", str(behavior_file), str(mission_file)]
        process = subprocess.run(command, cwd=str(ROOT_DIR), capture_output=True, text=True)

        if process.returncode != 0:
            return self.send_json({
                "success": False,
                "error": process.stderr.strip() or process.stdout.strip() or f"ksim failed with exit code {process.returncode}",
                "stdout": process.stdout,
                "stderr": process.stderr,
            }, status=500)

        if not behavior_file.exists():
            return self.send_json({
                "success": False,
                "error": "Expected behavior log file was not created.",
                "stdout": process.stdout,
                "stderr": process.stderr,
            }, status=500)

        try:
            behavior_log = json.loads(behavior_file.read_text(encoding="utf-8"))
        except Exception as exc:
            return self.send_json({
                "success": False,
                "error": f"Failed to parse behavior log: {exc}",
                "stdout": process.stdout,
                "stderr": process.stderr,
            }, status=500)

        return self.send_json({
            "success": True,
            "mission_file": mission_file.name,
            "behavior_file": behavior_file.name,
            "behavior_log": behavior_log,
            "stdout": process.stdout,
            "stderr": process.stderr,
        })

    def serve_file(self, path, content_type):
        if not path.exists():
            self.send_error(404, "File not found")
            return
        content = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, data, status=200):
        body = json.dumps(data, indent=None).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        return


def run_server(host, port):
    server_address = (host, port)
    httpd = http.server.HTTPServer(server_address, MissionConstructorHandler)
    print(f"Serving mission constructor at http://{host}:{port}/")
    print(f"ksim_runs directory: {RUN_DIR}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        httpd.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the Odyssey mission constructor app and invoke ksim.")
    parser.add_argument("--host", default="127.0.0.1", help="Host to bind the web server")
    parser.add_argument("--port", default=8000, type=int, help="Port to listen on")
    args = parser.parse_args()

    if not HTML_FILE.exists():
        print(f"Error: mission_constructor.html not found at {HTML_FILE}", file=sys.stderr)
        sys.exit(1)

    run_server(args.host, args.port)
