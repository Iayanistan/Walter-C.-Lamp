"""Procedure builder: compose, check, save and run skill procedures from a browser.

    ros2 run walter_app server                    # http://localhost:8080 on the laptop
    ros2 run walter_app server --host 0.0.0.0     # reachable from other machines on the lab network

It only offers the registered skills and the stations in stations.yaml, and every procedure goes
through the same compiler as `ros2 run walter_skills run`, so the app cannot ask the arm for
anything the command line couldn't. Uses only the Python standard library.

Anyone who can open the page can start the arm: keep the default localhost binding unless the
network is the cell's own. The Halt button is a convenience, not an emergency stop.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

from walter_skills.cli import stations_path
from walter_skills.skills import Cell, ProcedureError, compile_procedure, describe_skills

STATIC = Path(__file__).resolve().parent / "static"
NAME = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
SERVICES = {
    "confirm": ("/task_runner/confirm", "std_srvs/srv/Trigger"),
    "halt": ("/supervisor/halt", "std_srvs/srv/Trigger"),
    "reset": ("/supervisor/reset", "std_srvs/srv/Trigger"),
}


def procedures_dir() -> Path:
    path = Path(os.environ.get("WALTER_PROCEDURES", "~/walter_cell/procedures")).expanduser()
    path.mkdir(parents=True, exist_ok=True)
    return path


class Runner:
    """One procedure at a time, as a `ros2 run walter_skills run` child process."""

    def __init__(self):
        self.proc = None
        self.started = 0.0
        self.name = ""
        self.lock = threading.Lock()

    def start(self, path: Path, name: str, speed_scale: float):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                raise RuntimeError(f"'{self.name}' is still running")
            self.started, self.name = time.time(), name
            self.proc = subprocess.Popen(
                ["ros2", "run", "walter_skills", "run", str(path), "--speed-scale", str(speed_scale)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def status(self) -> dict:
        running = bool(self.proc and self.proc.poll() is None)
        events = []
        runs = Path.home() / ".ros" / "walter_runs"
        if self.started and runs.exists():
            dirs = [d for d in runs.iterdir() if d.is_dir() and d.name != "compiled"
                    and d.stat().st_mtime >= self.started - 1]
            if dirs:
                log = max(dirs, key=lambda d: d.stat().st_mtime) / "events.jsonl"
                if log.exists():
                    events = [json.loads(line) for line in log.read_text().splitlines()[-200:] if line.strip()]
        return {"name": self.name, "running": running, "events": events}


RUNNER = Runner()


def stations() -> list[dict]:
    cell = yaml.safe_load(stations_path().read_text())
    return [{"name": n, "description": s.get("description", ""), "poses": list(s.get("poses", {})),
             "tagged": "tag" in s} for n, s in cell["stations"].items()]


def check(procedure: dict) -> dict:
    try:
        task = compile_procedure(procedure, Cell.load(stations_path()))
    except (ProcedureError, KeyError, TypeError, AttributeError) as e:
        return {"ok": False, "error": str(e)}
    return {"ok": True, "steps": task["steps"]}


class Handler(BaseHTTPRequestHandler):
    server_version = "WalterApp/0.1"

    def log_message(self, fmt, *args):  # quieter than the default
        pass

    def _send(self, code: int, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n) or b"{}")

    def _proc_path(self, name: str) -> Path:
        if not NAME.match(name):
            raise ValueError("names may use letters, digits, - and _ only")
        return procedures_dir() / f"{name}.yaml"

    def do_GET(self):
        path = self.path.split("?")[0]
        try:
            if path in ("/", "/index.html"):
                return self._send(200, (STATIC / "index.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/api/skills":
                return self._send(200, describe_skills())
            if path == "/api/stations":
                return self._send(200, stations())
            if path == "/api/procedures":
                return self._send(200, sorted(p.stem for p in procedures_dir().glob("*.yaml")))
            if path.startswith("/api/procedures/"):
                p = self._proc_path(path.rsplit("/", 1)[1])
                if not p.exists():
                    return self._send(404, {"error": "no such procedure"})
                return self._send(200, yaml.safe_load(p.read_text()))
            if path == "/api/run":
                return self._send(200, RUNNER.status())
            return self._send(404, {"error": "not found"})
        except ValueError as e:
            return self._send(400, {"error": str(e)})

    def do_PUT(self):
        try:
            if not self.path.startswith("/api/procedures/"):
                return self._send(404, {"error": "not found"})
            p = self._proc_path(self.path.rsplit("/", 1)[1])
            procedure = self._body()
            procedure["name"] = p.stem
            result = check(procedure)
            if not result["ok"]:
                return self._send(400, result)
            p.write_text(yaml.safe_dump(procedure, sort_keys=False, default_flow_style=None, width=120))
            return self._send(200, {"ok": True, "saved": str(p)})
        except (ValueError, json.JSONDecodeError) as e:
            return self._send(400, {"ok": False, "error": str(e)})

    def do_POST(self):
        try:
            if self.path == "/api/compile":
                result = check(self._body())
                return self._send(200 if result["ok"] else 400, result)
            if self.path == "/api/run":
                body = self._body()
                p = self._proc_path(body.get("name", ""))
                if not p.exists():
                    return self._send(404, {"ok": False, "error": "save the procedure first"})
                result = check(yaml.safe_load(p.read_text()))
                if not result["ok"]:
                    return self._send(400, result)
                RUNNER.start(p, p.stem, float(body.get("speed_scale", 1.0)))
                return self._send(200, {"ok": True})
            action = self.path.rsplit("/", 1)[-1]
            if self.path.startswith("/api/") and action in SERVICES:
                srv, srv_type = SERVICES[action]
                out = subprocess.run(["ros2", "service", "call", srv, srv_type], capture_output=True,
                                     text=True, timeout=15)
                ok = out.returncode == 0 and "success=True" in out.stdout
                return self._send(200 if ok else 502, {"ok": ok, "output": out.stdout[-500:] + out.stderr[-500:]})
            return self._send(404, {"error": "not found"})
        except RuntimeError as e:
            return self._send(409, {"ok": False, "error": str(e)})
        except (ValueError, json.JSONDecodeError, subprocess.TimeoutExpired) as e:
            return self._send(400, {"ok": False, "error": str(e)})


def make_server(host: str, port: int) -> ThreadingHTTPServer:
    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Walter procedure builder")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    args, _ = ap.parse_known_args(argv)
    server = make_server(args.host, args.port)
    print(f"procedure builder on http://{args.host}:{args.port}  (procedures in {procedures_dir()})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
