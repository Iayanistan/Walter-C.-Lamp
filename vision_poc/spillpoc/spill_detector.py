"""Spill = material found outside every region where that material is allowed to be.

    allowed(material) = union of dilated boxes of the vessels listed in materials.<m>.allowed_in
    spill_mask        = material_mask AND NOT allowed AND NOT baseline

A spill only fires once it has persisted for `persist_frames` consecutive frames, and
clears after `clear_frames` frames without one, so sensor noise doesn't stop the arm.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .scene import SceneState


@dataclass
class SpillBlob:
    material: str
    bbox: tuple[int, int, int, int]
    area: int


@dataclass
class SpillReport:
    active: bool = False              # a confirmed spill is present
    just_started: bool = False        # became active on this frame
    just_cleared: bool = False        # became inactive on this frame
    blobs: list[SpillBlob] = field(default_factory=list)
    spill_masks: dict[str, np.ndarray] = field(default_factory=dict)
    allowed_masks: dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def materials(self) -> list[str]:
        return sorted({b.material for b in self.blobs})

    @property
    def total_area(self) -> int:
        return sum(b.area for b in self.blobs)


class SpillDetector:
    def __init__(self, cfg: dict):
        s = cfg["spill"]
        self.margin = int(s.get("vessel_margin_px", 18))
        self.min_area = int(s.get("min_spill_area", 250))
        self.persist_frames = int(s.get("persist_frames", 5))
        self.clear_frames = int(s.get("clear_frames", 15))
        self.baseline_frames = int(s.get("baseline_frames", 20))
        self.allowed_in: dict[str, list[str]] = {
            m: list(v.get("allowed_in", [])) for m, v in cfg["materials"].items()
        }
        self.baseline: dict[str, np.ndarray] = {}
        self._baseline_acc: dict[str, np.ndarray] = {}
        self._baseline_n = 0
        self._hit_streak = 0
        self._miss_streak = 0
        self.active = False

    # ---- baseline -------------------------------------------------------
    def start_baseline(self):
        self.baseline.clear()
        self._baseline_acc.clear()
        self._baseline_n = 0

    @property
    def baseline_ready(self) -> bool:
        return self._baseline_n >= self.baseline_frames

    def feed_baseline(self, masks: dict[str, np.ndarray]) -> bool:
        """Accumulate 'empty bench' material masks. Returns True once enough frames are in."""
        if self.baseline_ready:
            return True
        for m, mask in masks.items():
            acc = self._baseline_acc.setdefault(m, np.zeros(mask.shape, dtype=np.uint16))
            acc += (mask > 0).astype(np.uint16)
        self._baseline_n += 1
        if self.baseline_ready:
            # Anything coloured in >= half the baseline frames is part of the bench, not a spill.
            half = max(1, self._baseline_n // 2)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            for m, acc in self._baseline_acc.items():
                base = np.where(acc >= half, 255, 0).astype(np.uint8)
                self.baseline[m] = cv2.dilate(base, k)
        return self.baseline_ready

    # ---- per-frame --------------------------------------------------------
    def allowed_mask(self, scene: SceneState, material: str) -> np.ndarray:
        h, w = scene.frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        allowed = set(self.allowed_in.get(material, []))
        for d in scene.detections:
            if d.cls_name in allowed:
                x1, y1, x2, y2 = d.xyxy
                cv2.rectangle(mask, (x1 - self.margin, y1 - self.margin),
                              (x2 + self.margin, y2 + self.margin), 255, -1)
        return mask

    def update(self, scene: SceneState) -> SpillReport:
        report = SpillReport()
        for material, mask in scene.masks.items():
            allowed = self.allowed_mask(scene, material)
            spill = cv2.bitwise_and(mask, cv2.bitwise_not(allowed))
            if material in self.baseline:
                spill = cv2.bitwise_and(spill, cv2.bitwise_not(self.baseline[material]))
            report.allowed_masks[material] = allowed
            report.spill_masks[material] = spill
            n, labels, stats, _ = cv2.connectedComponentsWithStats(spill, connectivity=8)
            for i in range(1, n):
                area = int(stats[i, cv2.CC_STAT_AREA])
                if area >= self.min_area:
                    x, y, bw, bh = (int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP]),
                                    int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT]))
                    report.blobs.append(SpillBlob(material, (x, y, x + bw, y + bh), area))

        hit = len(report.blobs) > 0
        if hit:
            self._hit_streak += 1
            self._miss_streak = 0
        else:
            self._miss_streak += 1
            self._hit_streak = 0

        was_active = self.active
        if not self.active and self._hit_streak >= self.persist_frames:
            self.active = True
        elif self.active and self._miss_streak >= self.clear_frames:
            self.active = False

        report.active = self.active
        report.just_started = self.active and not was_active
        report.just_cleared = was_active and not self.active
        return report

    def reset(self):
        self._hit_streak = self._miss_streak = 0
        self.active = False
