"""HUD drawing: vessel boxes, material tints, spill highlights, protocol panel, event ticker."""

from __future__ import annotations

import cv2
import numpy as np

from .protocol import Protocol
from .scene import SceneState
from .spill_detector import SpillReport

CLASS_COLORS = {
    "scale": (200, 200, 60),
    "beaker": (60, 180, 255),
    "petri_dish": (255, 160, 60),
    "test_tube": (200, 120, 255),
    "pipette": (120, 255, 120),
}
MATERIAL_TINT = {"liquid": (255, 80, 0), "solid": (0, 200, 0)}
SPILL_RED = (0, 0, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(img, s, org, scale=0.55, color=(255, 255, 255), thick=1):
    cv2.putText(img, s, org, FONT, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, color, thick, cv2.LINE_AA)


def draw_scene(frame: np.ndarray, scene: SceneState, spill: SpillReport,
               show_allowed: bool = False) -> np.ndarray:
    out = frame.copy()

    # Allowed regions (debug view).
    if show_allowed:
        for m, allowed in spill.allowed_masks.items():
            tint = np.zeros_like(out)
            tint[allowed > 0] = MATERIAL_TINT.get(m, (255, 255, 255))
            out = cv2.addWeighted(out, 1.0, tint, 0.12, 0)

    # Material masks.
    for m, mask in scene.masks.items():
        tint = np.zeros_like(out)
        tint[mask > 0] = MATERIAL_TINT.get(m, (255, 255, 255))
        out = cv2.addWeighted(out, 1.0, tint, 0.45, 0)

    # Spill pixels + blobs.
    for m, smask in spill.spill_masks.items():
        out[smask > 0] = (0.35 * out[smask > 0] + 0.65 * np.array(SPILL_RED)).astype(np.uint8)
    for b in spill.blobs:
        x1, y1, x2, y2 = b.bbox
        cv2.rectangle(out, (x1, y1), (x2, y2), SPILL_RED, 2)
        _text(out, f"SPILL? {b.material} {b.area}px", (x1, max(14, y1 - 6)), 0.5, SPILL_RED, 1)

    # Vessel boxes.
    for d in scene.detections:
        color = CLASS_COLORS.get(d.cls_name, (255, 255, 255))
        x1, y1, x2, y2 = d.xyxy
        thick = 1 if d.remembered else 2
        if d.remembered:
            # dashed-ish: draw corners only
            L = 14
            for (px, py, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
                cv2.line(out, (px, py), (px + dx * L, py), color, 2)
                cv2.line(out, (px, py), (px, py + dy * L), color, 2)
        else:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick)
        label = f"{d.cls_name} {d.conf:.2f}" + (" (mem)" if d.remembered else "")
        _text(out, label, (x1, max(14, y1 - 6)), 0.5, color, 1)
    return out


def draw_panel(out: np.ndarray, proto: Protocol, spill: SpillReport, scene: SceneState,
               fps: float, weights_label: str, recent_events: list[str],
               baseline_status: str) -> np.ndarray:
    h, w = out.shape[:2]
    panel_w = 420
    canvas = np.zeros((h, w + panel_w, 3), dtype=np.uint8)
    canvas[:, :w] = out
    p = canvas[:, w:]
    p[:] = (28, 28, 28)
    x0, y = 14, 30

    _text(p, "SPILL DETECTION PoC", (x0, y), 0.7, (255, 255, 255), 2); y += 26
    _text(p, proto.name, (x0, y), 0.45, (180, 180, 180)); y += 18
    _text(p, f"detector: {weights_label}", (x0, y), 0.42, (150, 150, 150)); y += 16
    _text(p, f"{fps:4.1f} fps   frame {scene.frame_idx}   {baseline_status}", (x0, y), 0.42, (150, 150, 150)); y += 26

    # Status banner.
    if proto.state == Protocol.HALT:
        cv2.rectangle(p, (x0 - 4, y - 20), (panel_w - 10, y + 30), (0, 0, 180), -1)
        _text(p, "SPILL DETECTED - ARM HALTED", (x0, y + 4), 0.65, (255, 255, 255), 2)
        _text(p, proto.halt_reason[:48], (x0, y + 24), 0.42, (255, 220, 220))
        y += 48
    elif proto.state == Protocol.COMPLETE:
        cv2.rectangle(p, (x0 - 4, y - 20), (panel_w - 10, y + 10), (0, 140, 0), -1)
        _text(p, "PROTOCOL COMPLETE", (x0, y + 2), 0.65, (255, 255, 255), 2); y += 30
    elif proto.state == Protocol.TIMEOUT:
        cv2.rectangle(p, (x0 - 4, y - 20), (panel_w - 10, y + 10), (0, 120, 180), -1)
        _text(p, "STEP TIMED OUT - ARM PAUSED", (x0, y + 2), 0.6, (255, 255, 255), 2); y += 30
    elif spill.blobs:
        _text(p, f"material outside vessel: {spill.total_area}px (confirming...)", (x0, y), 0.45, (0, 120, 255)); y += 22
    else:
        _text(p, "monitoring: no spill", (x0, y), 0.5, (120, 255, 120)); y += 22
    y += 8

    # Step list.
    _text(p, "PROTOCOL", (x0, y), 0.5, (200, 200, 200)); y += 20
    for s in proto.steps:
        if s.index < proto.idx or proto.state == Protocol.COMPLETE:
            mark, color = "[x]", (120, 255, 120)
        elif s is proto.current and proto.state == Protocol.HALT:
            mark, color = "[!]", (80, 80, 255)
        elif s is proto.current:
            mark, color = "[>]", (0, 220, 255)
        else:
            mark, color = "[ ]", (140, 140, 140)
        _text(p, f"{mark} {s.index}. {s.name}", (x0, y), 0.5, color); y += 18
        if s is proto.current and proto.is_running:
            _text(p, f"     wait: {s.done_when}", (x0, y), 0.4, (170, 170, 170)); y += 14
            _text(p, f"     arm : {s.arm_command}", (x0, y), 0.4, (170, 170, 170)); y += 14
            frac = proto.progress_fraction(scene.timestamp)
            cv2.rectangle(p, (x0 + 6, y - 6), (x0 + 6 + 200, y + 2), (70, 70, 70), -1)
            cv2.rectangle(p, (x0 + 6, y - 6), (x0 + 6 + int(200 * frac), y + 2), (0, 220, 255), -1)
            y += 14
    nxt = proto.next_step
    y += 6
    _text(p, f"next planned: {nxt.name if nxt else '-'}", (x0, y), 0.45, (200, 200, 200)); y += 24

    # Scene facts the conditions are looking at.
    _text(p, "SCENE", (x0, y), 0.5, (200, 200, 200)); y += 18
    facts = [
        f"solid on scale : {scene.material_fraction_in('solid', 'scale'):.3f}",
        f"solid in dish  : {scene.material_fraction_in('solid', 'petri_dish'):.3f}",
        f"liquid in dish : {scene.material_fraction_in('liquid', 'petri_dish'):.3f}",
        f"pipette@beaker : {scene.vessels_overlap('pipette', 'beaker')}",
        "vessels: " + ", ".join(sorted({d.cls_name for d in scene.detections})) or "vessels: none",
    ]
    for f in facts:
        _text(p, f, (x0, y), 0.42, (170, 170, 170)); y += 15
    y += 8

    # Event ticker.
    _text(p, "EVENTS", (x0, y), 0.5, (200, 200, 200)); y += 18
    for e in recent_events[-7:]:
        _text(p, e[:52], (x0, y), 0.4, (190, 190, 190)); y += 15

    # Keys.
    _text(p, "q quit  r reset  b re-baseline  c ack spill  a allowed  s snap",
          (x0, h - 12), 0.38, (130, 130, 130))
    return canvas
