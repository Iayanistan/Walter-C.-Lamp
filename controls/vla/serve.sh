#!/usr/bin/env bash
# Serve a fine-tuned checkpoint for the ROS 2 bridge (walter_bringup vla.launch.py). Run from the Isaac-GR00T checkout.
#   CKPT=~/walter_ckpt/run1/checkpoint-5000 bash serve.sh
# Replay mode (no model, replays recorded actions; checks the plumbing end to end):
#   REPLAY=~/walter_data/flask_to_hotplate bash serve.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PORT="${PORT:-5555}"
if [[ -n "${REPLAY:-}" ]]; then
  exec uv run python gr00t/eval/run_gr00t_server.py --embodiment-tag NEW_EMBODIMENT \
    --dataset-path "$REPLAY" --modality-config-path "$HERE/walter_modality_config.py" \
    --execution-horizon 8 --host 0.0.0.0 --port "$PORT"
fi
exec uv run python gr00t/eval/run_gr00t_server.py --embodiment-tag NEW_EMBODIMENT \
  --model-path "${CKPT:?set CKPT to a checkpoint directory}" --device cuda:0 --host 0.0.0.0 --port "$PORT"
