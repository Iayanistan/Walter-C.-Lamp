"""YOLO wrapper that returns our 5 vessel classes, with short-term memory for occlusions."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from ultralytics import YOLO

from . import resolve
from .scene import Detection


class VesselDetector:
    def __init__(self, cfg: dict):
        d = cfg["detector"]
        self.classes: list[str] = list(d["classes"])
        self.conf = float(d.get("conf", 0.35))
        self.iou = float(d.get("iou", 0.5))
        self.imgsz = int(d.get("imgsz", 640))
        self.device = d.get("device", "cpu")
        self.memory_frames = int(d.get("memory_frames", 15))

        weights = resolve(d["weights"])
        if weights.exists():
            self.model = YOLO(str(weights))
            self.name_map = {i: n for i, n in self.model.names.items()}
            self.using_fallback = False
        else:
            # COCO model until the custom detector is trained. Map a few COCO classes
            # onto ours so the rest of the pipeline can be exercised.
            self.model = YOLO(d.get("fallback_weights", "yolo11n.pt"))
            fb = d.get("coco_fallback_map", {})
            self.name_map = {i: fb.get(n) for i, n in self.model.names.items()}
            self.using_fallback = True

        # cls_name -> (Detection, frames_since_seen)
        self._memory: dict[str, tuple[Detection, int]] = {}

        # Warm-up so the first real frame isn't slow.
        self.model.predict(np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8),
                           imgsz=self.imgsz, device=self.device, verbose=False)

    @property
    def weights_label(self) -> str:
        return "COCO fallback (untrained)" if self.using_fallback else Path(str(self.model.ckpt_path)).name

    def detect(self, frame: np.ndarray) -> list[Detection]:
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
                                 device=self.device, verbose=False)[0]
        fresh: list[Detection] = []
        if res.boxes is not None and len(res.boxes):
            xyxy = res.boxes.xyxy.cpu().numpy().astype(int)
            confs = res.boxes.conf.cpu().numpy()
            clss = res.boxes.cls.cpu().numpy().astype(int)
            h, w = frame.shape[:2]
            for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
                name = self.name_map.get(int(k))
                if name not in self.classes:
                    continue
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w - 1, x2), min(h - 1, y2)
                fresh.append(Detection(name, (int(x1), int(y1), int(x2), int(y2)), float(c)))

        # Update memory with the best fresh box per class; age the rest.
        seen = set()
        for det in fresh:
            cur = self._memory.get(det.cls_name)
            if det.cls_name not in seen or (cur and det.conf > cur[0].conf):
                self._memory[det.cls_name] = (det, 0)
                seen.add(det.cls_name)
        out = list(fresh)
        for name, (det, age) in list(self._memory.items()):
            if name in seen:
                continue
            age += 1
            if age > self.memory_frames:
                del self._memory[name]
                continue
            self._memory[name] = (det, age)
            out.append(Detection(det.cls_name, det.xyxy, det.conf, remembered=True))
        return out

    def reset_memory(self):
        self._memory.clear()
