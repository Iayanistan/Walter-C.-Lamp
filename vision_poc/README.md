# Spill-Detection Computer Vision — Proof of Concept

MDS410 capstone (iodine-testing robotic arm). This PoC demonstrates that **one camera feed** can both
**drive a lab protocol step by step** and **detect spills** — material ending up anywhere other than its
intended vessel — during any of those steps, halting the (simulated) arm when it happens.

## How it works

```
webcam ──► YOLO vessel detector ──► boxes: scale, beaker, petri_dish, test_tube, pipette
       └─► HSV colour segmenter ──► masks: liquid, solid
                                          │
             allowed(material) = dilated boxes of vessels that material may be in (+ pipette for liquid)
             spill_mask        = material ∧ ¬allowed ∧ ¬baseline(empty bench)
             SPILL fires after N consecutive frames  ──►  protocol → SPILL_HALT, ARM.emergency_stop()
                                          │
             protocol state machine: each step waits until its vision condition holds for 2 s,
             then "sends" the next arm command (printed/logged) and moves on.
```

Protocol (config.yaml → `protocol.steps`):

| # | Step | Advances when the camera sees… | Simulated arm command |
|---|------|-------------------------------|-----------------------|
| 0 | SETUP | scale + beaker + petri dish all visible | `ARM.home()` |
| 1 | WEIGH_SOLID | solid on the scale | `ARM.dispense_solid(target='scale')` |
| 2 | TRANSFER_SOLID | solid in the petri dish | `ARM.transfer(from='scale', to='petri_dish')` |
| 3 | DRAW_LIQUID | pipette overlapping the beaker | `ARM.pipette.aspirate(from='beaker')` |
| 4 | DISPENSE_LIQUID | liquid in the petri dish | `ARM.pipette.dispense(to='petri_dish')` |
| 5 | VERIFY | solid **and** liquid in the dish | `CAMERA.verify_result()` → COMPLETE |

Every run writes `runs/<timestamp>/events.jsonl` plus a PNG snapshot of each spill.

## Setup (once)

```powershell
# 1. CUDA PyTorch + dependencies (RTX A1000). Verifies torch.cuda.is_available() at the end.
powershell -ExecutionPolicy Bypass -File scripts\setup_gpu.ps1

# 2. (Only if re-downloading datasets without signed links) copy .env.example -> .env and
#    paste your private Roboflow key (app.roboflow.com -> Settings -> API Keys). .env is git-ignored.
```

**Data lives outside OneDrive.** `config.yaml` points `dataset.out_dir` / `download_dir` at
`C:/Users/<you>/spillpoc_data/` because OneDrive sync throttles image reads to ~1 MB/s during
training. Change those two paths if you move machines.

## Training the vessel detector (no manual labeling)

The detector is trained on public Roboflow Universe datasets forked into your workspace
(see `dataset.sources` in `config.yaml`; licences: MIT and CC BY 4.0).

```powershell
python scripts\prepare_dataset.py        # download exports, remap 30+ classes -> our 5, merge
python scripts\train_vessels.py          # yolo11s, 50 epochs on the GPU -> models/vessels.pt
```

`prepare_dataset.py` downloads from signed export links in `<download_dir>/links.json` (produced by
Roboflow's *versions_export*; no account key needed) and falls back to the SDK + `.env` key.
It prints instances per class (anything under ~300 is flagged) and keeps ~100 label-free images per
source as background negatives so the model learns that other glassware is *not* a beaker/dish.

Sources ship their own train/valid splits, which left one class trained on one dataset and
validated on another (test_tube scored 0.27 mAP50 purely from that mismatch). `prepare_dataset.py`
therefore **pools every image and re-splits 80/10/10** by default (`--no-reshuffle` to disable).

Current merged dataset: 1,940 train / 242 valid / 243 test images; per-class instances —
scale 489, beaker 618, petri_dish 1069, test_tube 512, pipette 656.

### Trained model (`models/vessels.pt`, yolo11s, 60 epochs, RTX A1000)

| class | mAP50 | note |
|---|---|---|
| scale | 0.995 | |
| beaker | 0.959 | correctly rejects Erlenmeyer flasks |
| test_tube | 0.813 | was 0.269 before the re-split |
| pipette | 0.750 | bulb pipettes and plastic droppers |
| petri_dish | 0.338 | see below |
| **overall** | **0.771** | 10.9 ms inference per frame on the GPU |

The petri_dish figure is misleading: 78% of its validation instances are top-down **agar culture
plates** from one source, which the model finds hard (recall 0.44) and which look nothing like a
bench dish. On bench-style dishes — the demo case — recall is **0.78–0.83**. The bench-adaptation
step below is what closes the remaining gap on your own dish.

### Adapting to *your* bench (recommended, ~20 min, still no box drawing)

1. `python scripts\capture_frames.py` — take ~100 frames of your actual setup (follow the on-screen tips).
2. Upload them to the forked project and run Roboflow **Auto Label** with the five class prompts
   (or ask Claude to do it via the Roboflow MCP), then approve/reject in the Roboflow UI.
3. Generate a new version, bump `version:` in `config.yaml`, re-run `prepare_dataset.py`, then
   `python scripts\train_vessels.py --resume-from models\vessels.pt --epochs 15`.

Fallback without Roboflow: `python scripts\pseudo_label.py` lets the trained model propose boxes on
your frames; press `a` to accept a frame or `d` to discard, then fine-tune as above.

## Calibrating the material colours

```powershell
python scripts\tune_colors.py --material liquid     # drag sliders, press s to save into config.yaml
python scripts\tune_colors.py --material solid
```

Use a liquid colour far from skin tones (blue/green food dye works best). Iodine brown overlaps with
skin hue; if you must use it, keep the saturation floor high and test with your hand in frame.

## Running the demo

```powershell
python scripts\run_demo.py                          # webcam
python scripts\run_demo.py --source clip.mp4        # recorded clip
python scripts\run_demo.py --record demo_out.mp4    # save the annotated view
```

Keys (click the **video window** first so it has keyboard focus): `q` or `Esc` quit · `r` reset
protocol · `b` re-capture baseline · `c` acknowledge spill & resume · `a` show allowed regions ·
`s` snapshot. Closing the window with its X button, or Ctrl+C in the terminal, also exits cleanly —
the run always prints `stopped (<reason>)` and finishes writing its event log.

Until `models/vessels.pt` exists the demo runs on a COCO model with a rough class mapping
(cup→beaker, bowl→petri dish…) so the pipeline can be exercised early; the HUD says
"COCO fallback (untrained)".

## Evaluating on recorded clips

```
data/clips/spill/*.mp4   ← clips where a spill happens
data/clips/clean/*.mp4   ← clips without a spill
```

```powershell
python scripts\evaluate.py data\clips --save-videos
```

Prints a confusion table, recall / false-alarm rate, and mean detection latency; writes `runs/eval/results.csv`.

## Demo checklist for peers

1. Clean run → steps 0→5 in order, `PROTOCOL COMPLETE`, zero spill events.
2. Drip liquid on the bench during DISPENSE_LIQUID → `SPILL_HALT` within ~0.5 s, snapshot saved, log names the step.
3. Grains outside the dish during TRANSFER_SOLID → `SPILL_HALT` with material `solid`.
4. Carry a full pipette across the bench → **no** spill (the pipette is an allowed region).
5. Cover the beaker with your hand for a second → **no** false spill (box memory).

## Dataset attribution

- Projektas, *Lab Assistant Ver.2*, Roboflow Universe, MIT — https://universe.roboflow.com/projektas-1tsnj/lab-assistant-ver.2
- Object detection, *Laboratory Material Detector*, CC BY 4.0 — https://universe.roboflow.com/object-detection-mylvp/laboratory-material-detector
- AIHCMC, *ChemistryLab*, CC BY 4.0 — https://universe.roboflow.com/aihcmc/chemistrylab
- Microbio, *Petri-Dish*, CC BY 4.0 — https://universe.roboflow.com/microbio-xyalg/petri-dish-f96wr
