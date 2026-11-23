"""Live spill-detection demo.

    python scripts/run_demo.py                 # webcam from config.yaml
    python scripts/run_demo.py --source 1      # another camera index
    python scripts/run_demo.py --source clip.mp4 --record out.mp4

Keys: q quit | r reset protocol | b re-capture baseline | c acknowledge spill & resume
      a toggle allowed-region overlay | s save snapshot
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spillpoc import load_config, resolve  # noqa: E402
from spillpoc.camera import open_source  # noqa: E402
from spillpoc.events import EventLog  # noqa: E402
from spillpoc.material_segmenter import MaterialSegmenter  # noqa: E402
from spillpoc.overlay import draw_panel, draw_scene  # noqa: E402
from spillpoc.protocol import Protocol  # noqa: E402
from spillpoc.scene import SceneState  # noqa: E402
from spillpoc.spill_detector import SpillDetector  # noqa: E402
from spillpoc.vessel_detector import VesselDetector  # noqa: E402


class Pipeline:
    """One frame in -> annotated frame + events out. Shared by run_demo and evaluate."""

    def __init__(self, cfg: dict, log: EventLog | None = None):
        self.cfg = cfg
        self.detector = VesselDetector(cfg)
        self.segmenter = MaterialSegmenter(cfg)
        self.spills = SpillDetector(cfg)
        self.protocol = Protocol(cfg)
        self.log = log
        self.show_allowed = False
        self.spills.start_baseline()
        self._fps_t, self._fps = time.time(), 0.0

    def _emit(self, events, frame):
        for kind, fields in events:
            if self.log:
                self.log.log(kind, **fields)
                if kind == "spill_halt" and self.cfg["output"].get("save_spill_snapshots", True):
                    self.log.snapshot(frame, "spill")

    def process(self, frame_idx: int, ts: float, frame) -> tuple[SceneState, object]:
        dets = self.detector.detect(frame)
        masks = self.segmenter.segment(frame)
        scene = SceneState(frame_idx, ts, frame, dets, masks)

        if not self.spills.baseline_ready:
            # Build the "empty bench" mask first; protocol waits meanwhile.
            if self.spills.feed_baseline(masks):
                self._emit([("baseline_ready", {"frames": self.spills.baseline_frames})], frame)
            report = self.spills.update(scene)
            self.spills.reset()  # don't let baseline frames count toward a spill
            return scene, report

        report = self.spills.update(scene)
        events = self.protocol.update(scene, report)
        if report.just_cleared:
            events.append(("spill_cleared", {}))
        self._emit(events, frame)
        return scene, report

    def render(self, scene: SceneState, report) -> "cv2.Mat":
        now = time.time()
        dt = now - self._fps_t
        self._fps_t = now
        self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt) if dt > 0 else self._fps
        out = draw_scene(scene.frame, scene, report, self.show_allowed)
        status = "baseline OK" if self.spills.baseline_ready else \
            f"baseline {self.spills._baseline_n}/{self.spills.baseline_frames}"
        recent = list(self.log.recent) if self.log else []
        return draw_panel(out, self.protocol, report, scene, self._fps,
                          self.detector.weights_label, recent, status)

    # operator controls
    def reset(self, frame):
        self.protocol.reset()
        self.spills.reset()
        self.detector.reset_memory()
        self._emit([("reset", {})], frame)

    def rebaseline(self, frame):
        self.spills.start_baseline()
        self.spills.reset()
        self._emit([("baseline_restart", {})], frame)

    def acknowledge(self, frame):
        self.spills.reset()
        self._emit(self.protocol.acknowledge_spill(), frame)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--source", default=None, help="camera index or video path (overrides config)")
    ap.add_argument("--record", default=None, help="write the annotated output to this mp4")
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    log = EventLog(resolve(cfg["output"]["runs_dir"]))
    print(f"run dir: {log.run_dir}")
    pipe = Pipeline(cfg, log)
    print(f"detector: {pipe.detector.weights_label}")
    writer = None
    win = cfg["output"].get("window_name", "Spill Detection PoC")

    if not args.no_window:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    reason = "end of source"
    try:
        with open_source(cfg, args.source) as src:
            for idx, ts, frame in src.frames():
                scene, report = pipe.process(idx, ts, frame)
                view = pipe.render(scene, report)
                if args.record:
                    if writer is None:
                        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                                 src.fps if src.is_file else 20.0, (view.shape[1], view.shape[0]))
                    writer.write(view)
                if not args.no_window:
                    cv2.imshow(win, view)
                    k = cv2.waitKey(1) & 0xFF
                    if k in (ord("q"), 27):          # q or Esc
                        reason = "quit key"
                        break
                    elif k == ord("r"):
                        pipe.reset(frame)
                    elif k == ord("b"):
                        pipe.rebaseline(frame)
                    elif k == ord("c"):
                        pipe.acknowledge(frame)
                    elif k == ord("a"):
                        pipe.show_allowed = not pipe.show_allowed
                    elif k == ord("s"):
                        print("saved", log.snapshot(view, "snap"))
                    # The window's X button destroys the window but not this loop; without
                    # this check imshow just recreates it and the demo looks unclosable.
                    if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                        reason = "window closed"
                        break
    except KeyboardInterrupt:
        reason = "ctrl-c"
    finally:
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        for _ in range(5):      # Windows needs the event loop pumped to actually drop the window
            cv2.waitKey(1)
        log.log("run_end", state=pipe.protocol.state, reason=reason)
        log.close()
        print(f"stopped ({reason}); events in {log.run_dir}")


if __name__ == "__main__":
    main()
