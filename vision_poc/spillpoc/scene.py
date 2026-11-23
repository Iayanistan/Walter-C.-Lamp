"""SceneState: everything the protocol and spill logic know about the current frame."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Detection:
    cls_name: str
    xyxy: tuple[int, int, int, int]
    conf: float
    remembered: bool = False  # True when this box is carried over from a previous frame

    @property
    def area(self) -> int:
        x1, y1, x2, y2 = self.xyxy
        return max(0, x2 - x1) * max(0, y2 - y1)

    def overlaps(self, other: "Detection", margin: int = 0) -> bool:
        ax1, ay1, ax2, ay2 = self.xyxy
        bx1, by1, bx2, by2 = other.xyxy
        return not (ax2 + margin < bx1 or bx2 + margin < ax1 or
                    ay2 + margin < by1 or by2 + margin < ay1)


@dataclass
class SceneState:
    frame_idx: int
    timestamp: float
    frame: np.ndarray
    detections: list[Detection] = field(default_factory=list)
    masks: dict[str, np.ndarray] = field(default_factory=dict)  # material name -> uint8 mask (0/255)

    def get(self, cls_name: str) -> Detection | None:
        """Highest-confidence detection of a class, or None."""
        best = None
        for d in self.detections:
            if d.cls_name == cls_name and (best is None or d.conf > best.conf):
                best = d
        return best

    def has(self, *cls_names: str) -> bool:
        return all(self.get(c) is not None for c in cls_names)

    def material_fraction_in(self, material: str, cls_name: str) -> float:
        """Fraction of a vessel's box covered by a material mask (0 if vessel absent)."""
        det = self.get(cls_name)
        mask = self.masks.get(material)
        if det is None or mask is None or det.area == 0:
            return 0.0
        x1, y1, x2, y2 = det.xyxy
        roi = mask[y1:y2, x1:x2]
        return float(np.count_nonzero(roi)) / float(det.area)

    def vessels_overlap(self, a: str, b: str, margin: int = 0) -> bool:
        da, db = self.get(a), self.get(b)
        return da is not None and db is not None and da.overlaps(db, margin)
