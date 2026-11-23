"""Command-line entry points: compile a procedure, run it, or list the skills.

    ros2 run walter_skills compile my_procedure.yaml -o /tmp/task.yaml
    ros2 run walter_skills run my_procedure.yaml              # compiles, then starts task_runner
    ros2 run walter_skills skills                             # what the arm can do, as JSON

The stations file is $WALTER_STATIONS, else ~/walter_cell/stations.yaml, else the packaged one.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path

import yaml

from .skills import Cell, ProcedureError, compile_procedure, describe_skills


def stations_path() -> Path:
    if os.environ.get("WALTER_STATIONS"):
        return Path(os.environ["WALTER_STATIONS"]).expanduser()
    mine = Path.home() / "walter_cell" / "stations.yaml"
    if mine.exists():
        return mine
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("walter_skills")) / "config" / "stations.yaml"


def _compile(path: str) -> dict:
    procedure = yaml.safe_load(Path(path).expanduser().read_text())
    try:
        return compile_procedure(procedure, Cell.load(stations_path()))
    except ProcedureError as e:
        sys.exit(f"{path}: {e}")


def compile_main(argv=None):
    ap = argparse.ArgumentParser(description="Compile a skill procedure into a task_runner task file")
    ap.add_argument("procedure")
    ap.add_argument("-o", "--output", help="write here instead of stdout")
    args = ap.parse_args(argv)
    text = yaml.safe_dump(_compile(args.procedure), sort_keys=False, default_flow_style=None, width=120)
    if args.output:
        Path(args.output).write_text(text)
        print(f"wrote {args.output} (stations: {stations_path()})")
    else:
        print(text)


def run_main(argv=None):
    ap = argparse.ArgumentParser(description="Compile a procedure and run it with task_runner")
    ap.add_argument("procedure")
    ap.add_argument("--speed-scale", type=float, default=1.0)
    args = ap.parse_args(argv)
    task = _compile(args.procedure)
    out = Path.home() / ".ros" / "walter_runs" / "compiled"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{task['name']}_{datetime.now():%Y%m%d_%H%M%S}.yaml"
    path.write_text(yaml.safe_dump(task, sort_keys=False, default_flow_style=None, width=120))
    print(f"compiled {args.procedure} -> {path}")
    os.execvp("ros2", ["ros2", "run", "walter_control", "task_runner", "--ros-args",
                       "-p", f"task_file:={path}", "-p", f"speed_scale:={args.speed_scale}"])


def skills_main(argv=None):
    print(json.dumps(describe_skills(), indent=2))
