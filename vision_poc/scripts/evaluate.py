"""Run the full pipeline over recorded clips and score spill detection.

Folder layout (the folder name is the expected outcome):
    data/clips/spill/     clips where a spill happens
    data/clips/clean/     clips with no spill

    python scripts/evaluate.py data/clips
    python scripts/evaluate.py data/clips --save-videos   # also writes annotated mp4s to runs/eval/

Prints per-clip result, detection latency (time from first material-outside-vessel
pixel to confirmed SPILL_HALT), a confusion table, and writes runs/eval/results.csv.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from spillpoc import load_config, resolve  # noqa: E402
from spillpoc.camera import FrameSource  # noqa: E402
from run_demo import Pipeline  # noqa: E402

VIDEO_EXT = (".mp4", ".mov", ".avi", ".mkv")


def run_clip(cfg: dict, path: Path, save_to: Path | None) -> dict:
    pipe = Pipeline(cfg, log=None)
    first_suspect = None
    halted_at = None
    halted_step = None
    final_state = None
    writer = None
    with FrameSource(str(path)) as src:
        for idx, ts, frame in src.frames():
            scene, report = pipe.process(idx, ts, frame)
            if report.blobs and first_suspect is None and pipe.spills.baseline_ready:
                first_suspect = ts
            if pipe.protocol.state == pipe.protocol.HALT and halted_at is None:
                halted_at = ts
                halted_step = pipe.protocol.halted_step.name if pipe.protocol.halted_step else "-"
            final_state = pipe.protocol.state
            if save_to is not None:
                view = pipe.render(scene, report)
                if writer is None:
                    writer = cv2.VideoWriter(str(save_to), cv2.VideoWriter_fourcc(*"mp4v"), src.fps,
                                             (view.shape[1], view.shape[0]))
                writer.write(view)
    if writer:
        writer.release()
    return {
        "clip": path.name,
        "detected_spill": halted_at is not None,
        "halt_time_s": round(halted_at, 2) if halted_at is not None else "",
        "latency_s": round(halted_at - first_suspect, 2) if halted_at is not None and first_suspect is not None else "",
        "halted_step": halted_step or "",
        "final_state": final_state,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clips_dir")
    ap.add_argument("--config", default=None)
    ap.add_argument("--save-videos", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    root = resolve(args.clips_dir)
    out_dir = resolve("runs/eval")
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for expected_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        expected = expected_dir.name.lower() == "spill"
        for clip in sorted(p for p in expected_dir.iterdir() if p.suffix.lower() in VIDEO_EXT):
            print(f"-> {expected_dir.name}/{clip.name}")
            save_to = out_dir / f"{expected_dir.name}_{clip.stem}_annotated.mp4" if args.save_videos else None
            r = run_clip(cfg, clip, save_to)
            r["expected_spill"] = expected
            r["correct"] = r["detected_spill"] == expected
            rows.append(r)
            print(f"   expected={expected} detected={r['detected_spill']} "
                  f"latency={r['latency_s']} step={r['halted_step']} -> {'OK' if r['correct'] else 'MISS'}")

    if not rows:
        sys.exit("no clips found (expected data/clips/spill/*.mp4 and data/clips/clean/*.mp4)")

    tp = sum(1 for r in rows if r["expected_spill"] and r["detected_spill"])
    fn = sum(1 for r in rows if r["expected_spill"] and not r["detected_spill"])
    fp = sum(1 for r in rows if not r["expected_spill"] and r["detected_spill"])
    tn = sum(1 for r in rows if not r["expected_spill"] and not r["detected_spill"])
    lat = [r["latency_s"] for r in rows if r["latency_s"] != ""]
    print("\n                 detected   not detected")
    print(f"  spill clips    {tp:>8}   {fn:>12}")
    print(f"  clean clips    {fp:>8}   {tn:>12}")
    print(f"\n  accuracy {(tp + tn) / len(rows):.2f}   recall {tp / max(1, tp + fn):.2f}   "
          f"false-alarm rate {fp / max(1, fp + tn):.2f}")
    if lat:
        print(f"  mean detection latency {sum(lat) / len(lat):.2f}s  (max {max(lat):.2f}s)")

    csv_path = out_dir / "results.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {csv_path}")


if __name__ == "__main__":
    main()
