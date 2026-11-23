"""Vision-gated state machine that walks through the protocol defined in config.yaml.

Each step names a `done_when` condition. The machine evaluates it on every frame and
only advances once it has held for `stable_s` seconds. The spill monitor is checked on
every frame independently of the step; a confirmed spill halts the protocol.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .scene import SceneState
from .spill_detector import SpillReport

Condition = Callable[[SceneState], bool]


@dataclass
class Step:
    index: int
    name: str
    arm_command: str
    done_when: str
    timeout_s: float


class Protocol:
    HALT = "SPILL_HALT"
    COMPLETE = "COMPLETE"
    TIMEOUT = "TIMEOUT"

    def __init__(self, cfg: dict):
        p = cfg["protocol"]
        self.name = p.get("name", "protocol")
        self.stable_s = float(p.get("stable_s", 2.0))
        self.required = list(p.get("required_vessels", []))
        self.optional = list(p.get("optional_vessels", []))
        pf = p.get("presence_fraction", {})
        self.f_solid_scale = float(pf.get("solid_on_scale", 0.01))
        self.f_solid_dish = float(pf.get("solid_in_dish", 0.015))
        self.f_liquid_dish = float(pf.get("liquid_in_dish", 0.015))
        self.steps = [Step(i, s["name"], s.get("arm_command", ""), s["done_when"],
                           float(s.get("timeout_s", 0))) for i, s in enumerate(p["steps"])]
        self.conditions: dict[str, Condition] = {
            "all_required_vessels_visible": lambda sc: sc.has(*self.required),
            "solid_on_scale": lambda sc: sc.material_fraction_in("solid", "scale") >= self.f_solid_scale,
            "solid_in_dish": lambda sc: sc.material_fraction_in("solid", "petri_dish") >= self.f_solid_dish,
            "pipette_at_beaker": lambda sc: sc.vessels_overlap("pipette", "beaker"),
            # Liquid appearing in the dish is the evidence of a dispense; we deliberately don't
            # also require the pipette box (a hand often hides it at that moment).
            "liquid_in_dish": lambda sc: sc.material_fraction_in("liquid", "petri_dish") >= self.f_liquid_dish,
            "solid_and_liquid_in_dish": lambda sc: (
                sc.material_fraction_in("solid", "petri_dish") >= self.f_solid_dish
                and sc.material_fraction_in("liquid", "petri_dish") >= self.f_liquid_dish),
        }
        unknown = {s.done_when for s in self.steps} - set(self.conditions)
        if unknown:
            raise ValueError(f"Unknown done_when condition(s) in config: {unknown}")
        self.reset()

    # ---- state ------------------------------------------------------------
    def reset(self):
        self.idx = 0
        self.state = self.steps[0].name
        self.step_started_at: float | None = None
        self.condition_since: float | None = None
        self.halted_step: Step | None = None
        self.halt_reason = ""
        self.pending_events: list[tuple[str, dict]] = []

    @property
    def current(self) -> Step | None:
        return self.steps[self.idx] if self.idx < len(self.steps) else None

    @property
    def next_step(self) -> Step | None:
        return self.steps[self.idx + 1] if self.idx + 1 < len(self.steps) else None

    @property
    def is_running(self) -> bool:
        return self.state not in (self.HALT, self.COMPLETE, self.TIMEOUT)

    def condition_holding_s(self, now: float) -> float:
        return 0.0 if self.condition_since is None else now - self.condition_since

    def progress_fraction(self, now: float) -> float:
        return min(1.0, self.condition_holding_s(now) / self.stable_s) if self.stable_s > 0 else 1.0

    # ---- per-frame ---------------------------------------------------------
    def update(self, scene: SceneState, spill: SpillReport) -> list[tuple[str, dict]]:
        """Advance the machine. Returns a list of (event, fields) to log."""
        events: list[tuple[str, dict]] = []
        now = scene.timestamp

        if spill.just_started and self.state != self.HALT:
            self.halted_step = self.current
            self.halt_reason = f"{'+'.join(spill.materials)} outside allowed vessels ({spill.total_area}px)"
            self.state = self.HALT
            events.append(("spill_halt", {"step": self.halted_step.name if self.halted_step else "-",
                                          "material": "+".join(spill.materials), "area": spill.total_area,
                                          "arm": "ARM.emergency_stop()"}))
            return events

        if not self.is_running:
            return events

        step = self.current
        if self.step_started_at is None:
            self.step_started_at = now
            self.condition_since = None
            events.append(("step_started", {"step": step.name, "arm": step.arm_command}))

        if self.conditions[step.done_when](scene):
            if self.condition_since is None:
                self.condition_since = now
            if now - self.condition_since >= self.stable_s:
                events.append(("step_completed", {"step": step.name,
                                                  "duration_s": round(now - self.step_started_at, 2)}))
                self.idx += 1
                self.step_started_at = None
                self.condition_since = None
                if self.current is None:
                    self.state = self.COMPLETE
                    events.append(("protocol_complete", {"arm": "ARM.home()"}))
                else:
                    self.state = self.current.name
        else:
            self.condition_since = None
            if step.timeout_s > 0 and now - self.step_started_at > step.timeout_s:
                self.state = self.TIMEOUT
                events.append(("step_timeout", {"step": step.name, "arm": "ARM.pause()"}))
        return events

    def acknowledge_spill(self) -> list[tuple[str, dict]]:
        """Operator cleared the spill: resume the halted step from scratch."""
        if self.state != self.HALT:
            return []
        step = self.halted_step or self.steps[0]
        self.idx = step.index
        self.state = step.name
        self.step_started_at = None
        self.condition_since = None
        return [("spill_acknowledged", {"step": step.name, "arm": "ARM.resume()"})]
