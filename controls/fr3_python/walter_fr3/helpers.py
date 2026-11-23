"""Helper functions for the FAIRINO FR3: the arm's vocabulary for ASTM D4607.

Each public method of `Walter` marked with @helper is one block in the procedure builder, with the
same name and the same parameters. A procedure from the builder is just these calls in order:

    w = Walter(DryRunArm())
    w.pick("rack_1")                       # pick up the flask in rack slot 1
    w.swirl(seconds=10)                    # 10.4: swirl gently until the carbon is wetted
    w.place("hotplate")
    w.boil(seconds=30, tolerance_s=2)      # 10.4: boil gently 30 +/- 2 s
    ...

Stations (rack_1, hotplate, funnel, ...) and their taught joint poses come from a stations YAML
(stations_fr3_d4607.yaml by default). Steps the arm cannot do with one gripper (pipetting,
stoppering, titrating) go to the operator through `operator()` and `record()`.

Timed D4607 steps (boil, shake) are measured. A timed step outside its tolerance cannot be redone
on the same sample, so it is logged as a deviation and the run carries on; the report flags it.
"""

from __future__ import annotations

import inspect
import json
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np
import yaml

from .arm import Arm, ArmError, SERVO_PERIOD

DEFAULT_STATIONS = Path(__file__).with_name("stations_fr3_d4607.yaml")


class ProcedureError(ValueError):
    pass


# --- registry: what the procedure builder may offer -----------------------------------------------

@dataclass
class HelperInfo:
    name: str
    label: str
    group: str            # move | timed | check | people (the builder's shelf groups)
    help: str
    timed: bool = False


HELPERS: dict[str, HelperInfo] = {}


def helper(label: str, group: str, timed: bool = False):
    def register(fn):
        doc = (fn.__doc__ or "").strip().splitlines()[0]
        HELPERS[fn.__name__] = HelperInfo(fn.__name__, label, group, doc, timed)
        fn._helper = True
        return fn
    return register


def describe_helpers() -> list[dict]:
    """JSON-friendly list of helpers and their parameters (name, default, required)."""
    out = []
    for name, info in HELPERS.items():
        sig = inspect.signature(getattr(Walter, name))
        params = [{"name": p.name, "required": p.default is inspect._empty,
                   "default": None if p.default is inspect._empty else p.default}
                  for p in list(sig.parameters.values())[1:] if p.kind == p.POSITIONAL_OR_KEYWORD]
        out.append({**info.__dict__, "params": params})
    return out


# --- run log ---------------------------------------------------------------------------------------

@dataclass
class RunLog:
    path: Path | None = None
    events: list[dict] = field(default_factory=list)
    deviations: list[dict] = field(default_factory=list)
    values: dict[str, float] = field(default_factory=dict)

    def log(self, event: str, **fields):
        rec = {"wall": time.strftime("%Y-%m-%dT%H:%M:%S"), "event": event, **fields}
        self.events.append(rec)
        if self.path:
            with self.path.open("a") as f:
                f.write(json.dumps(rec) + "\n")


# --- operator I/O ----------------------------------------------------------------------------------

# Made-up numbers a dry run answers `record` with: a carbon with iodine number 964 and slope 0.111.
DEMO_ANSWERS = {"N1": 0.1000, "N2": 0.1000, "E": 950.0,
                "mass_flask_1": 1.210, "mass_flask_2": 1.027, "mass_flask_3": 0.810,
                "thio_flask_1": 6.00, "thio_flask_2": 10.00, "thio_flask_3": 16.00}


class Operator:
    """Asks a person at the terminal. `auto=True` answers by itself (dry runs and tests)."""

    def __init__(self, auto: bool = False, answers: dict[str, float] | None = None):
        self.auto = auto
        self.answers = answers or {}

    def confirm(self, prompt: str) -> bool:
        print(f"\n  >> OPERATOR: {prompt}")
        if self.auto:
            print("     (auto-confirmed)")
            return True
        reply = input("     Press Enter when done, or type 'abort': ").strip().lower()
        return reply != "abort"

    def number(self, key: str, prompt: str, default: float | None = None) -> float:
        print(f"\n  >> OPERATOR: {prompt}")
        if self.auto:
            v = self.answers.get(key, DEMO_ANSWERS.get(key, default))
            if v is None:
                v = 1.0
            print(f"     (auto, made-up dry-run value: {v})")
            return float(v)
        while True:
            raw = input(f"     {key} = ").strip()
            try:
                return float(raw)
            except ValueError:
                print("     Please type a number.")


# --- the helpers -----------------------------------------------------------------------------------

def _ramp(t: np.ndarray, duration: float, ramp: float) -> np.ndarray:
    """Raised-cosine envelope so ServoJ never sees a velocity step at the start or end."""
    env = np.ones_like(t)
    if ramp > 0:
        up, down = t < ramp, t > duration - ramp
        env[up] = 0.5 - 0.5 * np.cos(np.pi * t[up] / ramp)
        env[down] = 0.5 - 0.5 * np.cos(np.pi * (duration - t[down]) / ramp)
    return env


class Walter:
    """The FR3 plus the cell it works in. Every @helper method is a builder block."""

    def __init__(self, arm: Arm, stations: str | Path | dict = DEFAULT_STATIONS,
                 operator: Operator | None = None, log: RunLog | None = None,
                 checker: Callable[[str], tuple[bool, str]] | None = None,
                 max_vel_pct: float = 30.0):
        self.arm = arm
        self.cell = stations if isinstance(stations, dict) else yaml.safe_load(Path(stations).read_text())
        self.stations: dict[str, dict] = self.cell["stations"]
        g = self.cell.get("gripper", {})
        self.grip_settle = float(g.get("settle_s", 1.0))
        d = self.cell.get("dispenser", {})
        self.dispense_do, self.dispense_pulse = int(d.get("do", -1)), float(d.get("pulse_s", 0.5))
        self.op = operator or Operator()
        self.run_log = log or RunLog()
        self.checker = checker
        self.max_vel_pct = max_vel_pct
        self.holding: str | None = None      # name of the item in the gripper, for checks and logs
        self.items: dict[str, str] = {}      # station -> item sitting there (seeded by the procedure)
        self.step_spec: str | None = None
        for name, st in self.stations.items():
            for pose, q in st.get("poses", {}).items():
                if len(q) != 6:
                    raise ProcedureError(f"stations: {name}.{pose} needs 6 joint values, has {len(q)}")

    # ---- plumbing ----
    def log(self, event: str, **f):
        if self.step_spec:
            f.setdefault("spec", self.step_spec)
        self.run_log.log(event, **f)

    def _pose(self, station: str, pose: str) -> list[float]:
        if station not in self.stations:
            raise ProcedureError(f"unknown station '{station}' (known: {', '.join(self.stations)})")
        poses = self.stations[station].get("poses", {})
        if pose not in poses:
            raise ProcedureError(f"station '{station}' has no taught pose '{pose}' (has: {', '.join(poses)})")
        return poses[pose]

    def _go(self, station: str, pose: str, seconds: float):
        self.arm.move_joints(self._pose(station, pose), seconds, self.max_vel_pct)
        self.at = station

    def _grip(self, closed: bool):
        self.arm.gripper(closed)
        self.arm.sleep(self.grip_settle)

    def _timed(self, label: str, target: float, tolerance: float, started: float):
        actual = self.arm.clock() - started
        ok = abs(actual - target) <= tolerance
        self.log("timed_step", label=label, target_s=target, tolerance_s=tolerance,
                 actual_s=round(actual, 3), ok=ok)
        if not ok:
            dev = {"label": label, "spec": self.step_spec, "target_s": target, "tolerance_s": tolerance,
                   "actual_s": round(actual, 3), "item": self.holding or self.items.get("hotplate")}
            self.run_log.deviations.append(dev)
            print(f"  !! D4607 DEVIATION: {label} took {actual:.2f} s, allowed {target} +/- {tolerance} s")
        return ok

    def _oscillate(self, what: str, offsets: Callable[[np.ndarray], np.ndarray], duration: float, ramp: float):
        q0 = self.arm.joints()
        t = np.arange(1, int(round(duration / SERVO_PERIOD)) + 1) * SERVO_PERIOD
        positions = q0 + _ramp(t, duration, ramp)[:, None] * offsets(t)
        self.arm.follow(t, positions, what)

    # ---- motion ----
    @helper("Go home", "move")
    def home(self, seconds: float = 3.0):
        """Return to the safe pose clear of everything."""
        self._go("home", "home", seconds)
        self.log("home")

    @helper("Move to", "move")
    def move_to(self, station: str, pose: str = "above", seconds: float = 3.0):
        """Move to a taught pose at a station."""
        self._go(station, pose, seconds)
        self.log("move_to", station=station, pose=pose)

    @helper("Pick up", "move")
    def pick(self, station: str, verify: bool = True, seconds: float = 2.0):
        """Pick up the flask (or beaker) at a station: check it, open, reach in, grasp, lift."""
        if self.holding:
            raise ProcedureError(f"pick({station}): the gripper already holds {self.holding}")
        if verify:
            self.verify(station, quiet=True)
        self._grip(False)
        self._go(station, "above", seconds)
        self._go(station, "grasp", seconds)
        self._grip(True)
        self._go(station, "above", seconds)
        self.holding = self.items.pop(station, f"item from {station}")
        self.log("pick", station=station, item=self.holding)

    @helper("Place", "move")
    def place(self, station: str, verify: bool = True, seconds: float = 2.0):
        """Set the held item down at a station and back away."""
        if not self.holding:
            raise ProcedureError(f"place({station}): the gripper is empty")
        if station in self.items:
            raise ProcedureError(f"place({station}): {self.items[station]} is already there")
        if verify:
            self.verify(station, quiet=True)
        self._go(station, "above", seconds)
        self._go(station, "grasp", seconds)
        self._grip(False)
        self._go(station, "above", seconds)
        self.log("place", station=station, item=self.holding)
        self.items[station], self.holding = self.holding, None

    @helper("Hold under", "move")
    def hold_under(self, station: str = "dispenser", seconds: float = 2.0):
        """Hold the item under a fixed station, like the dispenser."""
        self._need_holding("hold_under")
        self._go(station, "under", seconds)
        self.log("hold_under", station=station, item=self.holding)

    @helper("Dispense", "move")
    def dispense(self, ml: float = 100.0, station: str = "dispenser", reagent: str = "0.100 N iodine"):
        """Hold the flask under the dispenser and dispense a volume (DO pulse, or the operator presses it)."""
        self._need_holding("dispense")
        self.hold_under(station)
        if self.dispense_do >= 0:
            self.arm.pulse_do(self.dispense_do, self.dispense_pulse)
            self.arm.sleep(max(3.0, ml / 25.0))   # let the dispenser finish its stroke
        else:
            self._ask(f"Dispense {ml:.1f} mL {reagent} into {self.holding} (held under the {station})")
        self.log("dispense", ml=ml, reagent=reagent, item=self.holding)

    @helper("Pour", "move")
    def pour(self, station: str = "funnel", hold_s: float = 5.0, tilt_s: float = 3.0, verify: bool = True):
        """Tilt the held flask over a station (the funnel), hold, and tilt back."""
        self._need_holding("pour")
        if verify:
            self.verify(station, quiet=True)
        self._go(station, "above", 2.0)
        self._go(station, "pour", tilt_s)       # slow tilt so the stream starts gently
        self.arm.sleep(hold_s)
        self._go(station, "above", tilt_s)
        self.log("pour", station=station, item=self.holding, hold_s=hold_s)

    # ---- timed / agitation ----
    @helper("Swirl", "timed")
    def swirl(self, seconds: float = 10.0, amplitude_deg: float = 6.0, frequency_hz: float = 1.5):
        """Swirl the held flask or beaker in place: wrist joints 4 and 5 trace a small circle."""
        self._need_holding("swirl")
        a, w = np.radians(amplitude_deg), 2 * np.pi * frequency_hz

        def offsets(t):
            off = np.zeros((len(t), 6))
            off[:, 3] = a * np.sin(w * t)          # J4 and J5 a quarter period apart -> circular path
            off[:, 4] = a * (1 - np.cos(w * t))    # starts at zero offset, so no jump at t = 0
            return off
        self._oscillate("swirl", offsets, seconds, ramp=min(1.0, seconds / 4))
        self.log("swirl", seconds=seconds, amplitude_deg=amplitude_deg, frequency_hz=frequency_hz, item=self.holding)

    @helper("Shake", "timed", timed=True)
    def shake(self, seconds: float = 30.0, tolerance_s: float = 1.0, amplitude_deg: float = 10.0,
              frequency_hz: float = 2.5):
        """Move to the shake station and shake the held (stoppered) flask hard, e.g. 30 +/- 1 s."""
        self._need_holding("shake")
        self._go("shake", "start", 1.5)
        a, w = np.radians(amplitude_deg), 2 * np.pi * frequency_hz

        def offsets(t):
            off = np.zeros((len(t), 6))
            off[:, 4] = a * np.sin(w * t)          # J5 swing, as in walter_control/shake.py
            return off
        start = self.arm.clock()
        self._oscillate("shake", offsets, seconds, ramp=0.5)
        self._timed("shake", seconds, tolerance_s, start)

    @helper("Boil", "timed", timed=True)
    def boil(self, seconds: float = 30.0, tolerance_s: float = 2.0, station: str = "hotplate",
             confirm_start: bool = True):
        """Time the boil of the flask on the hot plate (30 +/- 2 s), then lift it off on time."""
        if self.holding:
            raise ProcedureError("boil: the gripper must be empty (place the flask on the hot plate first)")
        if station not in self.items:
            raise ProcedureError(f"boil: nothing is on the {station}")
        flask = self.items[station]
        # Get the open gripper around the flask before the clock starts, so lifting it off takes
        # one grip and not a whole pick: the boil window is only +/- 2 s wide.
        self._grip(False)
        self._go(station, "above", 2.0)
        self._go(station, "grasp", 2.0)
        if confirm_start:
            self._ask(f"Watch {flask} on the hot plate and confirm the moment it starts to boil")
        start = self.arm.clock()
        self.log("boil_started", item=flask)
        self.arm.sleep(max(0.0, seconds - self.grip_settle))
        self._grip(True)
        self.holding = self.items.pop(station)
        self._timed("boil", seconds, tolerance_s, start)      # measured to the moment the lift begins
        self._go(station, "above", 1.5)
        self.log("boil_lifted", item=flask)

    @helper("Wait", "timed", timed=True)
    def wait(self, seconds: float, tolerance_s: float = 0.0):
        """Wait a fixed time."""
        start = self.arm.clock()
        self.arm.sleep(seconds)
        if tolerance_s:
            self._timed("wait", seconds, tolerance_s, start)
        self.log("wait", seconds=seconds)

    @helper("Cool", "timed")
    def cool(self, minutes: float = 15.0, confirm: bool = True):
        """Let flasks cool to room temperature: wait, then have the operator confirm."""
        self.arm.sleep(minutes * 60)
        if confirm:
            self._ask("Confirm the flasks on the cooling pad are at room temperature")
        self.log("cool", minutes=minutes)

    # ---- checks / people ----
    @helper("Check placement", "check")
    def verify(self, station: str, quiet: bool = False):
        """Check that a station's equipment sits on its marks (camera checker, or the operator)."""
        st = self.stations.get(station)
        if st is None:
            raise ProcedureError(f"unknown station '{station}'")
        if "tag" not in st:
            if not quiet:
                self.log("verify_skipped", station=station, reason="no tag")
            return True
        if self.checker:
            ok, msg = self.checker(station)
        else:
            ok, msg = self.op.confirm(f"Check that the {st.get('description', station).lower()} "
                                      f"is on its marks (tag {st['tag']})"), "operator"
        self.log("verify", station=station, ok=ok, how=msg)
        if not ok:
            self.arm.halt(f"{station} is not where it should be")
            self.arm.raise_if_halted()
        return ok

    @helper("Ask the operator", "people")
    def operator(self, prompt: str):
        """Pause until a person does something and confirms."""
        self._ask(prompt)

    @helper("Record a value", "people")
    def record(self, key: str, prompt: str, default: float | None = None) -> float:
        """Ask the operator for a number (a mass, a titration volume) and keep it for the report."""
        v = self.op.number(key, prompt, default)
        self.run_log.values[key] = v
        self.log("record", key=key, value=v)
        return v

    @helper("Report iodine number", "people")
    def report(self, hcl_ml: float = 10.0, iodine_ml: float = 100.0, filtrate_ml: float = 50.0):
        """D4607 section 11: iodine number from the recorded N1, N2, mass_<flask> and thio_<flask> values."""
        from . import calc
        v = self.run_log.values
        missing = [k for k in ("N1", "N2") if k not in v]
        flasks = [{"flask": k[5:], "mass_g": v[k], "thio_ml": v["thio_" + k[5:]]}
                  for k in v if k.startswith("mass_") and "thio_" + k[5:] in v]
        if missing or len(flasks) < 2:
            raise ProcedureError("report: record N1, N2 and, for at least two flasks, mass_<flask> and thio_<flask> first")
        iso = calc.iodine_number(v["N1"], v["N2"], flasks, i_ml=iodine_ml, h_ml=hcl_ml, f_ml=filtrate_ml)
        print("\n== ASTM D4607 result ==")
        for f, (c, xm) in zip(flasks, iso.points):
            print(f"  {f['flask']}: M = {f['mass_g']:.3f} g, S = {f['thio_ml']:.2f} mL, C = {c:.4f} N, X/M = {xm:.0f} mg/g")
        print(f"  Iodine number (X/M at C = 0.02 N): {iso.iodine_number:.0f} mg/g   slope {iso.slope:.3f}   r {iso.r:.4f}")
        for msg in iso.warnings:
            print(f"  !! {msg}")
        for d in self.run_log.deviations:
            print(f"  !! timed step '{d['label']}' on {d['item']}: {d['actual_s']} s, allowed {d['target_s']} +/- {d['tolerance_s']} s")
        self.result = {"standard": "ASTM D4607-94(2006)", "N1": v["N1"], "N2": v["N2"], "flasks": flasks,
                       **iso.as_dict(), "timed_step_deviations": self.run_log.deviations}
        self.log("result", **self.result)
        if self.run_log.path:
            (self.run_log.path.parent / "d4607_report.json").write_text(json.dumps(self.result, indent=2))
        return iso

    # ---- internals ----
    def _need_holding(self, what):
        if not self.holding:
            raise ProcedureError(f"{what}: the gripper is empty")

    def _ask(self, prompt):
        self.log("operator", prompt=prompt)
        if not self.op.confirm(prompt):
            self.arm.halt(f"operator aborted at: {prompt}")
            self.arm.raise_if_halted()

    # ---- running builder procedures -----------------------------------------------------------
    def call(self, name: str, params: dict[str, Any]):
        """Run one builder block: `name` is the block, `params` its fields (spec/label are metadata)."""
        if name not in HELPERS:
            raise ProcedureError(f"unknown block '{name}' (known: {', '.join(HELPERS)})")
        params = dict(params or {})
        self.step_spec = params.pop("spec", None) or None
        label = params.pop("label", None)
        params.pop("recover", None)          # GR00T recovery is the ROS 2 path's job, not the SDK's
        params.pop("retries", None)
        fn = getattr(self, name)
        sig = inspect.signature(fn)
        unknown = set(params) - set(sig.parameters)
        if unknown:
            raise ProcedureError(f"{name}: unknown parameter(s) {sorted(unknown)}")
        if label:
            self.log("step", block=name, label=label)
        try:
            return fn(**params)
        finally:
            self.step_spec = None


# --- safety: the spill-detection PoC halts the arm ------------------------------------------------

def listen_for_spills(arm: Arm, port: int = 50555) -> threading.Thread:
    """Halt the arm when the spill PoC (integrations/spill_udp.py) sends a spill event."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))

    def loop():
        while True:
            data, _ = sock.recvfrom(65535)
            try:
                rec = json.loads(data)
            except ValueError:
                continue
            if "spill" in str(rec.get("event", "")):
                arm.halt(f"spill detected ({rec.get('material', 'unknown')})")

    th = threading.Thread(target=loop, daemon=True, name="spill-listener")
    th.start()
    return th
