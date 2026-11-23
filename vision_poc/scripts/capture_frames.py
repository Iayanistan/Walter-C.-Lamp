"""Grab still frames from the webcam for Roboflow auto-labeling of YOUR bench.

    python scripts/capture_frames.py                  # SPACE saves a frame, q quits
    python scripts/capture_frames.py --every 2        # also auto-save every 2 s
    python scripts/capture_frames.py --out data/raw/session2

Aim for ~100 frames. Vary: vessel positions, hand in/out of frame, pipette in/out,
materials present/absent, room lighting. Keep the camera where it will be for the demo.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from spillpoc import load_config, resolve  # noqa: E402
from spillpoc.camera import open_source  # noqa: E402

TIPS = [
    "move each vessel to a new spot",
    "put your hand over / next to the beaker",
    "hold the pipette over the dish",
    "hold the pipette over the beaker",
    "add solid to the scale and dish",
    "add liquid to the dish",
    "change room lighting (lamp on/off, blinds)",
    "remove one vessel from the scene",
    "tilt the camera slightly",
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--source", default=None)
    ap.add_argument("--out", default="data/raw")
    ap.add_argument("--every", type=float, default=0.0, help="auto-save interval in seconds (0 = manual only)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    out = resolve(args.out)
    out.mkdir(parents=True, exist_ok=True)
    n = 0
    last_auto = time.time()
    with open_source(cfg, args.source) as src:
        for _, _, frame in src.frames():
            now = time.time()
            auto = args.every > 0 and now - last_auto >= args.every
            view = frame.copy()
            cv2.putText(view, f"saved {n}   SPACE=save  q=quit   tip: {TIPS[n % len(TIPS)]}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("capture_frames", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" ") or auto:
                p = out / f"bench_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
                cv2.imwrite(str(p), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
                n += 1
                last_auto = now
                print("saved", p.name)
    cv2.destroyAllWindows()
    print(f"{n} frames in {out}")


if __name__ == "__main__":
    main()
