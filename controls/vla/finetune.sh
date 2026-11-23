#!/usr/bin/env bash
# Fine-tune GR00T N1.7 on Walter demonstrations. Run from the Isaac-GR00T checkout.
#   DATASET=~/walter_data/flask_to_hotplate OUT=~/walter_ckpt/run1 bash /path/to/controls/vla/finetune.sh
# Needs a 40 GB+ GPU (H100/L40/A6000 class) and Hugging Face access to nvidia/Cosmos-Reason2-2B.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
DATASET="${DATASET:?set DATASET to a recorded dataset directory (or several, space separated)}"
OUT="${OUT:-/tmp/walter_finetune}"
STEPS="${STEPS:-5000}"
NUM_GPUS="${NUM_GPUS:-1}"

uv run torchrun --nproc_per_node="$NUM_GPUS" --master_port=29500 \
  gr00t/experiment/launch_finetune.py \
  --base-model-path nvidia/GR00T-N1.7-3B \
  --dataset-path $DATASET \
  --embodiment-tag NEW_EMBODIMENT \
  --modality-config-path "$HERE/walter_modality_config.py" \
  --num-gpus "$NUM_GPUS" \
  --output-dir "$OUT" \
  --max-steps "$STEPS" \
  --save-steps 1000 --save-total-limit 5 \
  --global-batch-size 32 \
  --color-jitter-params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
  --dataloader-num-workers 4

echo "Open-loop check (predicted vs recorded actions):"
echo "  uv run python gr00t/eval/open_loop_eval.py --dataset-path <one dataset> --embodiment-tag NEW_EMBODIMENT \\"
echo "      --model-path $OUT/checkpoint-$STEPS --traj-ids 0 --execution-horizon 16"
