"""Uniform frame source for a webcam index or a video file."""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np


class FrameSource:
    def __init__(self, source: int | str, width: int = 1280, height: int = 720,
                 flip_horizontal: bool = False):
        self.is_file = isinstance(source, str) and not source.isdigit()
        src = source if self.is_file else int(source)
        # CAP_DSHOW opens noticeably faster on Windows for webcams.
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW) if not self.is_file else cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video source {source!r}")
        if not self.is_file:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.flip = flip_horizontal
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.frame_idx = 0

    def read(self) -> np.ndarray | None:
        ok, frame = self.cap.read()
        if not ok:
            return None
        self.frame_idx += 1
        return cv2.flip(frame, 1) if self.flip else frame

    def frames(self):
        """Generator yielding (frame_idx, timestamp_s, frame). Stops at end of file."""
        t0 = time.time()
        while True:
            frame = self.read()
            if frame is None:
                return
            ts = (self.frame_idx / self.fps) if self.is_file else (time.time() - t0)
            yield self.frame_idx, ts, frame

    def release(self):
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()


def open_source(cfg: dict, override: str | None = None) -> FrameSource:
    cam = cfg["camera"]
    source = override if override is not None else cam["source"]
    if isinstance(source, str) and source.isdigit():
        source = int(source)
    return FrameSource(source, cam.get("width", 1280), cam.get("height", 720),
                       cam.get("flip_horizontal", False))
