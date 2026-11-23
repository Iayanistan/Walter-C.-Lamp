"""The arm's vocabulary: named skills that expand into task_runner steps.

A procedure is a list of skills with parameters, written against station names (stations.yaml):

    name: flask_to_hotplate
    steps:
      - pick:  {station: rack_1}
      - place: {station: hotplate}
      - wait:  {seconds: 30, tolerance_s: 2, label: boil}

`compile_procedure` checks it against the stations and turns it into the task file format that
walter_control's task_runner already executes (poses + move/gripper/wait/shake/vla/operator/check
steps). The procedure-builder app uses `describe_skills` to know which blocks it may offer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import yaml


class ProcedureError(ValueError):
    pass


@dataclass
class Param:
    type: type
    default: Any = None          # None means required
    help: str = ""
    station: bool = False        # value must be a station name (the app shows a dropdown)


@dataclass
class Skill:
    name: str
    help: str
    params: dict[str, Param]
    expand: Callable[["Cell", dict], list[dict]]
    timed: bool = False          # carries a D4607 time tolerance


SKILLS: dict[str, Skill] = {}


def skill(name: str, help: str, timed: bool = False, **params: Param):
    def register(fn):
        SKILLS[name] = Skill(name, help, params, fn, timed)
        return fn
    return register


class Cell:
    """stations.yaml, loaded and checked."""

    def __init__(self, data: dict):
        self.data = data
        self.stations: dict[str, dict] = data["stations"]
        self.grip_open = float(data.get("gripper", {}).get("open", 0.04))
        self.grip_closed = float(data.get("gripper", {}).get("closed", 0.0))
        for name, st in self.stations.items():
            for pose, q in st.get("poses", {}).items():
                if len(q) != 6:
                    raise ProcedureError(f"stations.yaml: {name}.{pose} needs 6 joint values, has {len(q)}")

    @classmethod
    def load(cls, path: str | Path) -> "Cell":
        return cls(yaml.safe_load(Path(path).expanduser().read_text()))

    def station(self, name: str) -> dict:
        if name not in self.stations:
            raise ProcedureError(f"unknown station '{name}' (known: {', '.join(self.stations)})")
        return self.stations[name]

    def pose(self, station: str, pose: str) -> str:
        self.station(station)
        if pose not in self.stations[station].get("poses", {}):
            raise ProcedureError(f"station '{station}' has no taught pose '{pose}'")
        return f"{station}.{pose}"

    def has_tag(self, station: str) -> bool:
        return "tag" in self.station(station)

    def all_poses(self) -> dict[str, list[float]]:
        return {f"{s}.{p}": list(map(float, q))
                for s, st in self.stations.items() for p, q in st.get("poses", {}).items()}


# --- primitive step helpers ---------------------------------------------------------------------

def _move(cell, station, poses, seconds):
    return {"type": "move", "through": [cell.pose(station, p) for p in poses], "segment_s": seconds}


def _grip(position):
    return {"type": "gripper", "position": position, "settle_s": 1.0}


def _check(cell, station, p):
    if not p.get("verify", True) or not cell.has_tag(station):
        return []
    step = {"type": "check", "station": station}
    if p.get("recover"):
        step["recover"] = p["recover"]
    return [step]


# --- the skills ---------------------------------------------------------------------------------

@skill("move_to", "Move to a taught pose at a station.",
       station=Param(str, help="Station", station=True),
       pose=Param(str, "above", "Taught pose name"),
       seconds=Param(float, 3.0, "Duration of the move"))
def _move_to(cell, p):
    return [_move(cell, p["station"], [p["pose"]], p["seconds"])]


@skill("home", "Return to the safe home pose.", seconds=Param(float, 3.0, "Duration of the move"))
def _home(cell, p):
    return [_move(cell, "home", ["home"], p["seconds"])]


@skill("pick", "Check the station, then grasp the item there and lift it.",
       station=Param(str, help="Where to pick from", station=True),
       verify=Param(bool, True, "Check the station's AprilTag first"),
       recover=Param(str, "", "GR00T instruction to try if the check fails"),
       seconds=Param(float, 2.0, "Duration of each move"))
def _pick(cell, p):
    s = p["station"]
    return _check(cell, s, p) + [
        _grip(cell.grip_open),
        _move(cell, s, ["above", "grasp"], p["seconds"]),
        _grip(cell.grip_closed),
        _move(cell, s, ["above"], p["seconds"]),
    ]


@skill("place", "Check the station, then set the held item down there and back away.",
       station=Param(str, help="Where to place", station=True),
       verify=Param(bool, True, "Check the station's AprilTag first"),
       recover=Param(str, "", "GR00T instruction to try if the check fails"),
       seconds=Param(float, 2.0, "Duration of each move"))
def _place(cell, p):
    s = p["station"]
    return _check(cell, s, p) + [
        _move(cell, s, ["above", "grasp"], p["seconds"]),
        _grip(cell.grip_open),
        _move(cell, s, ["above"], p["seconds"]),
    ]


@skill("verify", "Check a station's AprilTag without moving; recover or ask the operator if it is off.",
       station=Param(str, help="Station to check", station=True),
       recover=Param(str, "", "GR00T instruction to try if the check fails"),
       retries=Param(int, 1, "GR00T attempts before asking the operator"))
def _verify(cell, p):
    if not cell.has_tag(p["station"]):
        raise ProcedureError(f"station '{p['station']}' has no tag to verify")
    step = {"type": "check", "station": p["station"]}
    if p["recover"]:
        step.update(recover=p["recover"], retries=p["retries"])
    return [step]


@skill("hold_under", "Hold the item under a fixed station, e.g. the dispenser.",
       station=Param(str, "dispenser", "Station", station=True),
       seconds=Param(float, 2.0, "Duration of the move"))
def _hold_under(cell, p):
    return [_move(cell, p["station"], ["under"], p["seconds"])]


@skill("pour", "Tilt the held flask over a station, hold, and tilt back.",
       station=Param(str, "funnel", "Station with 'above' and 'pour' poses", station=True),
       hold_s=Param(float, 5.0, "How long to hold the pour"),
       tilt_s=Param(float, 3.0, "How slowly to tilt"),
       verify=Param(bool, True, "Check the station's AprilTag first"),
       recover=Param(str, "", "GR00T instruction to try if the check fails"))
def _pour(cell, p):
    s = p["station"]
    return _check(cell, s, p) + [
        _move(cell, s, ["above"], 2.0),
        _move(cell, s, ["pour"], p["tilt_s"]),
        {"type": "wait", "seconds": p["hold_s"]},
        _move(cell, s, ["above"], p["tilt_s"]),
    ]


@skill("shake", "Move to the shake station and shake the held flask.", timed=True,
       seconds=Param(float, 30.0, "Shake duration"),
       tolerance_s=Param(float, 1.0, "Allowed timing error"),
       amplitude_deg=Param(float, 10.0, "Wrist swing each way"),
       frequency_hz=Param(float, 2.5, "Shakes per second"))
def _shake(cell, p):
    return [
        _move(cell, "shake", ["start"], 1.5),
        {"type": "shake", "joint": "joint_5", "amplitude_deg": p["amplitude_deg"],
         "frequency_hz": p["frequency_hz"], "duration_s": p["seconds"], "ramp_s": 0.5,
         "tolerance_s": p["tolerance_s"], "timed": True},
    ]


@skill("wait", "Wait a fixed time (e.g. the 30 s boil).", timed=True,
       seconds=Param(float, help="How long"),
       tolerance_s=Param(float, 0.0, "Allowed timing error (0 = not a timed D4607 step)"))
def _wait(cell, p):
    step = {"type": "wait", "seconds": p["seconds"]}
    if p["tolerance_s"]:
        step.update(tolerance_s=p["tolerance_s"], timed=True)
    return [step]


@skill("operator", "Ask the operator to do something and wait for them to confirm.",
       prompt=Param(str, help="What the operator should do"))
def _operator(cell, p):
    return [{"type": "operator", "prompt": p["prompt"]}]


@skill("vla", "Hand the arm to GR00T with a language instruction.",
       instruction=Param(str, help="What GR00T should do"),
       max_duration=Param(float, 30.0, "Give up after this many seconds"))
def _vla(cell, p):
    return [{"type": "vla", "instruction": p["instruction"], "max_duration": p["max_duration"]}]


# --- compiler -----------------------------------------------------------------------------------

def _coerce(skill_name, name, spec: Param, value):
    if spec.type is float and isinstance(value, int) and not isinstance(value, bool):
        value = float(value)
    if not isinstance(value, spec.type):
        raise ProcedureError(f"{skill_name}.{name} should be {spec.type.__name__}, got {value!r}")
    return value


def compile_procedure(procedure: dict, cell: Cell) -> dict:
    """Procedure (skills) -> task file (task_runner steps). Raises ProcedureError on any mistake."""
    steps = []
    for i, entry in enumerate(procedure.get("steps") or []):
        where = f"step {i + 1}"
        if not isinstance(entry, dict) or len(entry) != 1:
            raise ProcedureError(f"{where}: write each step as '- skill_name: {{params}}'")
        (name, given), = entry.items()
        given = dict(given or {})
        if name not in SKILLS:
            raise ProcedureError(f"{where}: unknown skill '{name}' (known: {', '.join(SKILLS)})")
        sk = SKILLS[name]
        label = str(given.pop("label", f"{i + 1:02d}_{name}"))
        spec = given.pop("spec", None)
        unknown = set(given) - set(sk.params)
        if unknown:
            raise ProcedureError(f"{where} ({name}): unknown parameter(s) {sorted(unknown)}")
        params = {}
        for pname, pspec in sk.params.items():
            if pname in given:
                params[pname] = _coerce(name, pname, pspec, given[pname])
            elif pspec.default is None:
                raise ProcedureError(f"{where} ({name}): missing required parameter '{pname}'")
            else:
                params[pname] = pspec.default
        try:
            expanded = sk.expand(cell, params)
        except ProcedureError as e:
            raise ProcedureError(f"{where} ({name}): {e}") from None
        for j, st in enumerate(expanded):
            st["label"] = label if len(expanded) == 1 else f"{label}/{j + 1}_{st['type']}"
            st["skill"] = name
            if spec:
                st["spec"] = spec
        steps.extend(expanded)
    if not steps:
        raise ProcedureError("procedure has no steps")
    return {"name": procedure.get("name", "procedure"), "poses": cell.all_poses(), "steps": steps}


def describe_skills() -> list[dict]:
    """JSON-friendly list of skills and their parameters, for the procedure-builder app."""
    return [{"name": s.name, "help": s.help, "timed": s.timed,
             "params": [{"name": n, "type": p.type.__name__, "required": p.default is None,
                         "default": p.default, "help": p.help, "station": p.station}
                        for n, p in s.params.items()]}
            for s in SKILLS.values()]
