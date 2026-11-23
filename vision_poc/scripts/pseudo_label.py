"""Fallback to Roboflow auto-label: let OUR trained model propose boxes on your bench frames,
and accept/discard whole frames with one keypress (no box drawing).

    python scripts/pseudo_label.py                       # frames from data/raw
    python scripts/pseudo_label.py --frames data/raw/session2 --conf 0.5

Keys:  a accept frame (boxes saved as labels)   d discard   b back one   q quit
Accepted frames go to data/dataset/train/{images,labels} with a "bench_" prefix, so a
subsequent  python scripts/train_vessels.py --resume-from models/vessels.pt --epochs 15
fine-tunes on them. Note: prepare_dataset.py rebuilds data/dataset from scratch and will
remove them, so run this AFTER prepare_dataset.py.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from spillpoc import load_config, resolve  # noqa: E402
from spillpoc.overlay import CLASS_COLORS  # noqa: E402
from spillpoc.vessel_detector import VesselDetector  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--frames", default="data/raw")
    ap.add_argument("--conf", type=float, default=0.4)
    args = ap.parse_args()

    cfg = load_config(args.config)
    cfg["detector"]["conf"] = args.conf
    cfg["detector"]["memory_frames"] = 0
    det = VesselDetector(cfg)
    if det.using_fallback:
        sys.exit("models/vessels.pt not found - train first (scripts/train_vessels.py)")
    classes = det.classes

    frames = sorted(p for p in resolve(args.frames).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not frames:
        sys.exit(f"no images in {args.frames} - run scripts/capture_frames.py first")
    out_img = resolve(cfg["dataset"]["out_dir"]) / "train" / "images"
    out_lbl = resolve(cfg["dataset"]["out_dir"]) / "train" / "labels"
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    i = accepted = 0
    while 0 <= i < len(frames):
        p = frames[i]
        frame = cv2.imread(str(p))
        dets = det.detect(frame)
        h, w = frame.shape[:2]
        view = frame.copy()
        for d in dets:
            x1, y1, x2, y2 = d.xyxy
            c = CLASS_COLORS.get(d.cls_name, (255, 255, 255))
            cv2.rectangle(view, (x1, y1), (x2, y2), c, 2)
            cv2.putText(view, f"{d.cls_name} {d.conf:.2f}", (x1, max(14, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2, cv2.LINE_AA)
        cv2.putText(view, f"[{i + 1}/{len(frames)}] {p.name}  {len(dets)} boxes   a=accept d=discard b=back q=quit  (accepted {accepted})",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("pseudo_label", view)
        k = cv2.waitKey(0) & 0xFF
        if k == ord("q"):
            break
        if k == ord("b"):
            i = max(0, i - 1)
            continue
        if k == ord("a") and dets:
            stem = f"bench_{p.stem}"
            shutil.copy2(p, out_img / (stem + p.suffix.lower()))
            lines = []
            for d in dets:
                x1, y1, x2, y2 = d.xyxy
                cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
                bw, bh = (x2 - x1) / w, (y2 - y1) / h
                lines.append(f"{classes.index(d.cls_name)} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
            (out_lbl / (stem + ".txt")).write_text("\n".join(lines) + "\n", encoding="utf-8")
            accepted += 1
        i += 1
    cv2.destroyAllWindows()
    print(f"accepted {accepted} frames into {out_img.parent}")


if __name__ == "__main__":
    main()
