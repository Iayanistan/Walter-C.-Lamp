"""Download the Roboflow dataset versions listed in config.yaml, remap their classes onto
our 5 vessel classes, and merge them into one YOLO dataset at data/dataset/.

    python scripts/prepare_dataset.py            # download + build
    python scripts/prepare_dataset.py --no-download   # rebuild from data/downloads only

Downloads use, in order of preference:
  1. a signed export link in data/downloads/links.json  ({"<project>": "https://app.roboflow.com/ds/..."}),
     as returned by Roboflow's versions_export (no account key needed), or
  2. ROBOFLOW_API_KEY from .env (see .env.example) via the roboflow SDK.
Images with no remaining labels are dropped (except a small quota kept as background negatives).
"""

from __future__ import annotations

import argparse
import io
import json
import os
import random
import shutil
import sys
import zipfile
from collections import Counter
from pathlib import Path

import requests
import yaml
from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from spillpoc import PROJECT_ROOT, load_config, resolve  # noqa: E402

SPLITS = ("train", "valid", "test")


def download_signed(url: str, dest: Path) -> Path:
    dest.mkdir(parents=True, exist_ok=True)
    for child in dest.iterdir():
        shutil.rmtree(child) if child.is_dir() else child.unlink()
    r = requests.get(url, stream=True, timeout=120)
    r.raise_for_status()
    total = int(r.headers.get("content-length", 0))
    buf, done = io.BytesIO(), 0
    for chunk in r.iter_content(1 << 20):
        buf.write(chunk)
        done += len(chunk)
        if total:
            print(f"\r  downloading {done / 1e6:6.1f}/{total / 1e6:.1f} MB", end="", flush=True)
    print()
    with zipfile.ZipFile(buf) as z:
        z.extractall(dest)
    return dest


def download(ws: str, project: str, version: int, dest: Path, links: dict[str, str]) -> Path:
    if project in links:
        return download_signed(links[project], dest)
    from roboflow import Roboflow

    key = os.environ.get("ROBOFLOW_API_KEY")
    if not key:
        sys.exit(f"No signed link for {project} in data/downloads/links.json and ROBOFLOW_API_KEY not set "
                 "(copy .env.example to .env and add your private key).")
    dest.mkdir(parents=True, exist_ok=True)
    rf = Roboflow(api_key=key)
    ds = rf.workspace(ws).project(project).version(version).download("yolov11", location=str(dest), overwrite=True)
    return Path(ds.location)


def read_source_names(src_dir: Path) -> list[str]:
    with open(src_dir / "data.yaml", "r", encoding="utf-8") as f:
        names = yaml.safe_load(f)["names"]
    return list(names.values()) if isinstance(names, dict) else list(names)


def convert_source(src_dir: Path, class_map: dict[str, str], target_classes: list[str],
                   out_dir: Path, tag: str, counts: Counter, negatives_budget: int = 0) -> tuple[int, int, int]:
    src_names = read_source_names(src_dir)
    idx_map: dict[int, int] = {}
    for i, n in enumerate(src_names):
        if n in class_map:
            idx_map[i] = target_classes.index(class_map[n])
    missing = set(class_map) - set(src_names)
    if missing:
        print(f"  warning: class_map names not in source data.yaml: {sorted(missing)}")

    kept = dropped = negatives = 0
    for split in SPLITS:
        img_dir, lbl_dir = src_dir / split / "images", src_dir / split / "labels"
        if not img_dir.exists():
            continue
        # Sources with no test split: their valid images stay valid; that's fine for us.
        (out_dir / split / "images").mkdir(parents=True, exist_ok=True)
        (out_dir / split / "labels").mkdir(parents=True, exist_ok=True)
        for img in img_dir.iterdir():
            if img.suffix.lower() not in (".jpg", ".jpeg", ".png"):
                continue
            lbl = lbl_dir / (img.stem + ".txt")
            lines_out = []
            if lbl.exists():
                for line in lbl.read_text(encoding="utf-8").splitlines():
                    parts = line.split()
                    if len(parts) < 5:
                        continue
                    new_idx = idx_map.get(int(float(parts[0])))
                    if new_idx is None:
                        continue
                    lines_out.append(" ".join([str(new_idx)] + parts[1:5]))
                    counts[(split, target_classes[new_idx])] += 1
            new_stem = f"{tag}_{img.stem}"
            if not lines_out:
                # Keep a few label-free images as background negatives (train split only):
                # other glassware the model must learn NOT to call a beaker/dish/etc.
                if split == "train" and negatives < negatives_budget:
                    shutil.copy2(img, out_dir / split / "images" / (new_stem + img.suffix.lower()))
                    (out_dir / split / "labels" / (new_stem + ".txt")).write_text("", encoding="utf-8")
                    negatives += 1
                else:
                    dropped += 1
                continue
            shutil.copy2(img, out_dir / split / "images" / (new_stem + img.suffix.lower()))
            (out_dir / split / "labels" / (new_stem + ".txt")).write_text("\n".join(lines_out) + "\n", encoding="utf-8")
            kept += 1
    return kept, dropped, negatives


def reshuffle(out_dir: Path, seed: int, ratios=(0.8, 0.1, 0.1)) -> None:
    """Pool every image and re-split 80/10/10.

    Each Roboflow source ships its own split, so merging them can leave a class trained on
    one source's images but validated on another's (test tubes did exactly that). Pooling
    first means train and valid see the same mixture.
    """
    pool = []
    for split in SPLITS:
        img_dir = out_dir / split / "images"
        if img_dir.exists():
            pool += [(p, out_dir / split / "labels" / (p.stem + ".txt")) for p in img_dir.iterdir()
                     if p.suffix.lower() in (".jpg", ".jpeg", ".png")]
    random.Random(seed).shuffle(pool)
    n = len(pool)
    bounds = (int(n * ratios[0]), int(n * (ratios[0] + ratios[1])))
    targets = {}
    for i, pair in enumerate(pool):
        targets[pair] = SPLITS[0] if i < bounds[0] else (SPLITS[1] if i < bounds[1] else SPLITS[2])
    moved = 0
    for (img, lbl), split in targets.items():
        if img.parent.parent.name == split:
            continue
        img.replace(out_dir / split / "images" / img.name)
        if lbl.exists():
            lbl.replace(out_dir / split / "labels" / lbl.name)
        moved += 1
    print(f"\nRe-split {n} images across sources (seed {seed}); moved {moved}")


def recount(out_dir: Path, target_classes: list[str]) -> Counter:
    counts: Counter = Counter()
    for split in SPLITS:
        lbl_dir = out_dir / split / "labels"
        if not lbl_dir.exists():
            continue
        for lbl in lbl_dir.glob("*.txt"):
            for line in lbl.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    counts[(split, target_classes[int(line.split()[0])])] += 1
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-download", action="store_true", help="reuse what is already in data/downloads")
    ap.add_argument("--no-reshuffle", action="store_true",
                    help="keep each source's own train/valid/test split instead of pooling and re-splitting")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--negatives", type=int, default=100,
                    help="background images (no target class) to keep per source in train (default 100)")
    args = ap.parse_args()

    load_dotenv(PROJECT_ROOT / ".env")
    cfg = load_config(args.config)
    ds_cfg = cfg["dataset"]
    target_classes: list[str] = list(cfg["detector"]["classes"])
    out_dir = resolve(ds_cfg["out_dir"])
    dl_dir = resolve(ds_cfg["download_dir"])

    # Clear contents rather than the folder itself: OneDrive keeps a handle on directories.
    out_dir.mkdir(parents=True, exist_ok=True)
    for child in out_dir.iterdir():
        shutil.rmtree(child) if child.is_dir() else child.unlink()

    links_path = dl_dir / "links.json"
    links: dict[str, str] = json.loads(links_path.read_text(encoding="utf-8")) if links_path.exists() else {}

    counts: Counter = Counter()
    for i, src in enumerate(ds_cfg["sources"]):
        project, version = src["project"], int(src["version"])
        dest = dl_dir / f"{project}-v{version}"
        print(f"\n[{i + 1}/{len(ds_cfg['sources'])}] {project} v{version}")
        if args.no_download:
            if not (dest / "data.yaml").exists():
                print("  skipped (not downloaded)")
                continue
        else:
            dest = download(ds_cfg["workspace"], project, version, dest, links)
        kept, dropped, neg = convert_source(dest, src["class_map"], target_classes, out_dir, f"s{i}", counts,
                                            negatives_budget=args.negatives)
        print(f"  kept {kept} labelled images + {neg} background negatives, dropped {dropped}")

    if not args.no_reshuffle:
        reshuffle(out_dir, args.seed)
        counts = recount(out_dir, target_classes)

    data_yaml = {
        "path": str(out_dir).replace("\\", "/"),
        "train": "train/images",
        "val": "valid/images",
        "test": "test/images" if (out_dir / "test" / "images").exists() else None,
        "nc": len(target_classes),
        "names": target_classes,
    }
    data_yaml = {k: v for k, v in data_yaml.items() if v is not None}
    with open(out_dir / "data.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(data_yaml, f, sort_keys=False)

    print("\nInstances per class:")
    print(f"  {'class':<12}" + "".join(f"{s:>8}" for s in SPLITS) + f"{'total':>8}")
    for c in target_classes:
        row = [counts[(s, c)] for s in SPLITS]
        flag = "  <-- thin" if sum(row) < 300 else ""
        print(f"  {c:<12}" + "".join(f"{n:>8}" for n in row) + f"{sum(row):>8}{flag}")
    n_imgs = {s: len(list((out_dir / s / "images").glob("*"))) for s in SPLITS if (out_dir / s / "images").exists()}
    print(f"\nImages: {n_imgs}\nWrote {out_dir / 'data.yaml'}")


if __name__ == "__main__":
    main()
