"""Interactive HSV tuner for the material colours. Writes the chosen range into config.yaml.

    python scripts/tune_colors.py --material liquid
    python scripts/tune_colors.py --material solid --source clip.mp4

Drag the trackbars until the mask (right) covers only the material. Press:
    s  save this range to config.yaml (replaces that material's hsv_ranges)
    p  pause/resume the video
    q  quit without saving
Tip: click on the material in the left window to print its HSV value.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from spillpoc import PROJECT_ROOT, load_config  # noqa: E402
from spillpoc.camera import open_source  # noqa: E402
from spillpoc.material_segmenter import hsv_in_range  # noqa: E402

WIN = "tune_colors"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--material", required=True, choices=["liquid", "solid"])
    ap.add_argument("--source", default=None)
    args = ap.parse_args()

    cfg_path = PROJECT_ROOT / "config.yaml"
    cfg = load_config(cfg_path)
    r0 = cfg["materials"][args.material]["hsv_ranges"][0]
    init = {"H lo": r0["lower"][0], "S lo": r0["lower"][1], "V lo": r0["lower"][2],
            "H hi": r0["upper"][0], "S hi": r0["upper"][1], "V hi": r0["upper"][2]}

    cv2.namedWindow(WIN)
    for k, v in init.items():
        cv2.createTrackbar(k, WIN, int(v), 179 if k.startswith("H") else 255, lambda _x: None)

    hsv_frame = [None]

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and hsv_frame[0] is not None and x < hsv_frame[0].shape[1]:
            print("HSV at click:", hsv_frame[0][y, x].tolist())

    cv2.setMouseCallback(WIN, on_mouse)

    paused = False
    frame = None
    with open_source(cfg, args.source) as src:
        while True:
            if not paused or frame is None:
                nxt = src.read()
                if nxt is None:
                    break
                frame = nxt
            small = cv2.resize(frame, None, fx=0.6, fy=0.6)
            hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
            hsv_frame[0] = hsv
            g = {k: cv2.getTrackbarPos(k, WIN) for k in init}
            lower = [g["H lo"], g["S lo"], g["V lo"]]
            upper = [g["H hi"], g["S hi"], g["V hi"]]
            mask = hsv_in_range(hsv, lower, upper)
            masked = cv2.bitwise_and(small, small, mask=mask)
            view = np.hstack([small, masked])
            cv2.putText(view, f"{args.material}: lower={lower} upper={upper}   s=save p=pause q=quit",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WIN, view)
            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            if k == ord("p"):
                paused = not paused
            if k == ord("s"):
                save_range(cfg_path, args.material, lower, upper)
                print(f"saved {args.material} range to config.yaml: lower={lower} upper={upper}")
    cv2.destroyAllWindows()


def save_range(cfg_path: Path, material: str, lower: list[int], upper: list[int]) -> None:
    """Replace the material's hsv_ranges block in config.yaml, keeping every comment intact."""
    text = cfg_path.read_text(encoding="utf-8")
    pattern = re.compile(
        rf"(^  {material}:\n(?:^    .*\n)*?)(^    hsv_ranges:\n(?:^      - .*\n)+)",
        re.MULTILINE)
    new_block = f"    hsv_ranges:\n      - {{lower: {lower}, upper: {upper}}}\n"
    text, n = pattern.subn(lambda m: m.group(1) + new_block, text, count=1)
    if n != 1:
        raise RuntimeError(f"could not find materials.{material}.hsv_ranges in {cfg_path}")
    cfg_path.write_text(text, encoding="utf-8")
    yaml.safe_load(text)  # sanity: still valid YAML


if __name__ == "__main__":
    main()
