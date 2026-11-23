"""Append-only JSONL event log for a run, plus spill snapshots. Gives peers an audit trail."""

from __future__ import annotations

import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


class EventLog:
    def __init__(self, runs_dir: Path, keep_recent: int = 8):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(runs_dir) / stamp
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.run_dir / "events.jsonl"
        self._f = open(self.path, "a", encoding="utf-8")
        self.recent: deque[str] = deque(maxlen=keep_recent)
        self._t0 = time.time()

    def log(self, kind: str, **fields):
        rec = {"t": round(time.time() - self._t0, 3), "wall": datetime.now().isoformat(timespec="seconds"),
               "event": kind, **fields}
        self._f.write(json.dumps(rec) + "\n")
        self._f.flush()
        summary = " ".join(f"{k}={v}" for k, v in fields.items() if k in ("step", "material", "area", "reason", "arm"))
        self.recent.append(f"[{rec['t']:7.2f}s] {kind} {summary}".rstrip())
        print(self.recent[-1])

    def snapshot(self, frame: np.ndarray, tag: str) -> Path:
        p = self.run_dir / f"{tag}_{int((time.time() - self._t0) * 1000):08d}.png"
        cv2.imwrite(str(p), frame)
        return p

    def close(self):
        self._f.close()
