# GR00T side (policy training and serving)

GR00T N1.7 runs in its own Python environment (Isaac-GR00T's `uv` venv, Python 3.12, PyTorch),
usually on the GPU machine. The ROS 2 side never imports it. The two talk over ZMQ on port 5555,
using the same protocol as `gr00t/eval/run_gr00t_server.py`.

| File | What it is |
|---|---|
| `walter_modality_config.py` | Tells GR00T how to read Walter datasets: cameras `scene` + `wrist`, state/action `single_arm` (6 joints) + `gripper` (1), 16-step action chunks, relative joint actions. |
| `finetune.sh` | `launch_finetune.py` with the Walter config on top of `nvidia/GR00T-N1.7-3B`. |
| `serve.sh` | Starts the policy server for a checkpoint, or in replay mode for a dataset. |

## One-time setup (GPU machine)

```bash
git clone https://github.com/NVIDIA/Isaac-GR00T && cd Isaac-GR00T
uv sync --python 3.12
uv run huggingface-cli login     # needs access to the gated nvidia/Cosmos-Reason2-2B
```

## Loop

1. Record demos with `episode_recorder` (Isaac Sim first, then the real cell). One dataset folder
   per skill is easiest: `flask_to_hotplate`, `flask_to_cooling`, `flask_to_dispenser`, `pour`.
2. `DATASET="~/walter_data/flask_to_hotplate ~/walter_data/pour" OUT=~/walter_ckpt/run1 bash finetune.sh`
3. Open-loop eval (command printed at the end of finetune.sh). Predictions should track the recorded curves on a training episode before you try closed loop.
4. `CKPT=~/walter_ckpt/run1/checkpoint-5000 bash serve.sh`
5. On the ROS machine: `ros2 launch walter_bringup vla.launch.py host:=<gpu machine ip>`

## Things to confirm on the first real checkpoint

- **Absolute or delta outputs.** The config trains relative joint actions. The bridge assumes the
  server returns absolute joint targets (`actions_are_deltas: false` in `walter_bringup/config/vla.yaml`).
  Run `serve.sh` in REPLAY mode against a recorded dataset first: if the arm retraces the demo, the
  setting is right. If it drifts by roughly the starting pose, flip it to `true`.
- **Latency.** The bridge logs `last_inference_ms` in feedback. Above about 150 ms, lower
  `execution_horizon` gains little; use TensorRT (`scripts/deployment/`) or overlap inference with execution.
- **Data volume.** NVIDIA's SO-100 example fine-tunes on a handful of episodes as a smoke test.
  Plan on 50 to 200 demos per skill for something that works off the training poses.
