"""Colour-based segmentation of the materials (liquid, solid) via HSV thresholds."""

from __future__ import annotations

import cv2
import numpy as np


def hsv_in_range(hsv: np.ndarray, lower, upper) -> np.ndarray:
    """cv2.inRange with hue wrap-around support (e.g. red: lower H > upper H)."""
    lo, hi = np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
    if lo[0] <= hi[0]:
        return cv2.inRange(hsv, lo, hi)
    a = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]], dtype=np.uint8))
    b = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], dtype=np.uint8), hi)
    return cv2.bitwise_or(a, b)


class MaterialSegmenter:
    def __init__(self, cfg: dict):
        self.materials: dict[str, dict] = cfg["materials"]

    @staticmethod
    def _clean(mask: np.ndarray, open_k: int, close_k: int, min_area: int) -> np.ndarray:
        if open_k > 1:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if close_k > 1:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        if min_area > 0:
            n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            keep = np.zeros(n, dtype=bool)
            keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_area
            mask = np.where(keep[labels], 255, 0).astype(np.uint8)
        return mask

    def segment(self, frame: np.ndarray) -> dict[str, np.ndarray]:
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        out: dict[str, np.ndarray] = {}
        for name, m in self.materials.items():
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            for r in m.get("hsv_ranges", []):
                mask = cv2.bitwise_or(mask, hsv_in_range(hsv, r["lower"], r["upper"]))
            out[name] = self._clean(mask, int(m.get("open_kernel", 3)),
                                    int(m.get("close_kernel", 5)), int(m.get("min_blob_area", 0)))
        return out
