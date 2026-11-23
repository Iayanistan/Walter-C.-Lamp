"""Fine-tune a YOLO detector for the 5 vessel classes on the GPU.

    python scripts/train_vessels.py                       # yolo11s, 50 epochs
    python scripts/train_vessels.py --model yolo11n.pt --epochs 30
    python scripts/train_vessels.py --resume-from models/vessels.pt --epochs 15   # bench fine-tune
    python scripts/train_vessels.py --resume --batch 8                            # continue an interrupted run

Writes best weights to models/vessels.pt. Falls back to a smaller batch if the 4 GB GPU
runs out of memory.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import torch
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from spillpoc import load_config, resolve  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--model", default="yolo11s.pt", help="starting checkpoint")
    ap.add_argument("--resume-from", default=None, help="start from these weights instead (e.g. models/vessels.pt)")
    ap.add_argument("--resume", action="store_true",
                    help="continue the interrupted run from its last.pt (only --batch/--imgsz can change)")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--name", default="vessels")
    ap.add_argument("--cpu", action="store_true", help="allow training without a GPU")
    args = ap.parse_args()

    cfg = load_config(args.config)
    data_yaml = resolve(cfg["dataset"]["out_dir"]) / "data.yaml"
    if not data_yaml.exists():
        sys.exit(f"{data_yaml} not found - run scripts/prepare_dataset.py first")
    out_weights = resolve(cfg["detector"]["weights"])
    # Per-epoch checkpoints go next to the dataset (outside OneDrive); only best.pt is copied back.
    train_runs_dir = resolve(cfg["dataset"]["out_dir"]).parent / "train_runs"

    if not torch.cuda.is_available():
        if not args.cpu:
            sys.exit("CUDA not available. If nvidia-smi says 'GPU access blocked by the operating system', "
                     "reset the graphics driver (Win+Ctrl+Shift+B) or reboot, then re-run. "
                     "Pass --cpu to train on the CPU anyway (very slow).")
        device = "cpu"
    else:
        device = 0
        print(f"GPU: {torch.cuda.get_device_name(0)}")

    start = args.resume_from or args.model
    batch = args.batch
    last_ckpt = train_runs_dir / args.name / "weights" / "last.pt"
    while True:
        try:
            if args.resume:
                if not last_ckpt.exists():
                    sys.exit(f"nothing to resume: {last_ckpt} missing")
                # Ultralytics restores every other setting from the checkpoint.
                results = YOLO(str(last_ckpt)).train(resume=True, batch=batch, imgsz=args.imgsz, device=device)
            else:
                results = YOLO(start).train(
                    data=str(data_yaml), epochs=args.epochs, imgsz=args.imgsz, batch=batch,
                    device=device, workers=args.workers, amp=True, cache="ram", project=str(train_runs_dir),
                    name=args.name, exist_ok=True, patience=15, plots=True,
                    # Modest augmentation: the bench camera is fixed, so no heavy geometric changes.
                    fliplr=0.5, degrees=5.0, hsv_h=0.01, hsv_s=0.5, hsv_v=0.4, mosaic=1.0, close_mosaic=10,
                )
            break
        except torch.cuda.OutOfMemoryError:
            torch.cuda.empty_cache()
            if batch <= 4:
                raise
            batch //= 2
            print(f"\nCUDA OOM - retrying with batch={batch}\n")

    best = Path(results.save_dir) / "weights" / "best.pt"
    out_weights.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, out_weights)
    print(f"\nSaved {out_weights}")

    # Per-class report on the validation split.
    metrics = YOLO(str(out_weights)).val(data=str(data_yaml), imgsz=args.imgsz, device=device, plots=False, verbose=False)
    names = metrics.names
    print(f"\nmAP50 overall: {metrics.box.map50:.3f}   mAP50-95: {metrics.box.map:.3f}")
    for i, ap50 in zip(metrics.box.ap_class_index, metrics.box.ap50):
        print(f"  {names[int(i)]:<12} mAP50 {ap50:.3f}")


if __name__ == "__main__":
    main()
