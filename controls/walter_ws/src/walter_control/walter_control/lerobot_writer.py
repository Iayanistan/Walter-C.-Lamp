"""Writes demonstrations in the GR00T flavour of the LeRobot v2 dataset format.

Layout (Isaac-GR00T getting_started/data_preparation.md):
    meta/info.json, meta/episodes.jsonl, meta/tasks.jsonl, meta/modality.json
    data/chunk-000/episode_000000.parquet
    videos/chunk-000/observation.images.<camera>/episode_000000.mp4   (H.264, yuv420p)

GR00T computes meta/stats.json and meta/relative_stats.json itself at fine-tune time.
No ROS imports; the episode_recorder node feeds this, and so could an Isaac Sim script.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

CHUNK_SIZE = 1000


@dataclass
class Layout:
    """Names and sizes of the concatenated state/action vectors; becomes meta/modality.json."""
    state: dict[str, int] = field(default_factory=lambda: {"single_arm": 6, "gripper": 1})
    action: dict[str, int] = field(default_factory=lambda: {"single_arm": 6, "gripper": 1})
    cameras: tuple[str, ...] = ("scene", "wrist")
    annotation: str = "human.task_description"

    @staticmethod
    def _ranges(parts: dict[str, int]) -> dict:
        out, start = {}, 0
        for name, size in parts.items():
            out[name] = {"start": start, "end": start + size}
            start += size
        return out

    def modality(self) -> dict:
        return {
            "state": self._ranges(self.state),
            "action": self._ranges(self.action),
            "video": {cam: {"original_key": f"observation.images.{cam}"} for cam in self.cameras},
            "annotation": {self.annotation: {}},
        }


class LeRobotWriter:
    def __init__(self, root: str | Path, fps: float, layout: Layout | None = None,
                 robot_type: str = "abb_irb120", joint_names: list[str] | None = None):
        self.root = Path(root)
        self.fps = fps
        self.layout = layout or Layout()
        self.robot_type = robot_type
        self.state_dim = sum(self.layout.state.values())
        self.action_dim = sum(self.layout.action.values())
        self.names = joint_names or [f"q{i}" for i in range(self.state_dim)]
        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        self.tasks: list[str] = self._read_jsonl("tasks.jsonl", "task")
        self.episodes = [json.loads(line) for line in self._lines("episodes.jsonl")]
        self.total_frames = sum(e["length"] for e in self.episodes)
        self.image_shapes: dict[str, tuple[int, int]] = {}
        (self.root / "meta" / "modality.json").write_text(json.dumps(self.layout.modality(), indent=2))

    def _lines(self, name):
        p = self.root / "meta" / name
        return [line for line in p.read_text().splitlines() if line.strip()] if p.exists() else []

    def _read_jsonl(self, name, key):
        return [json.loads(line)[key] for line in self._lines(name)]

    def task_index(self, task: str) -> int:
        if task not in self.tasks:
            self.tasks.append(task)
        return self.tasks.index(task)

    def write_episode(self, task: str, states: np.ndarray, actions: np.ndarray,
                      images: dict[str, list[np.ndarray]]) -> int:
        """states (N, state_dim), actions (N, action_dim), images {camera: N frames of (H, W, 3) uint8}."""
        states = np.asarray(states, dtype=np.float32)
        actions = np.asarray(actions, dtype=np.float32)
        n = len(states)
        if n == 0 or actions.shape != (n, self.action_dim) or states.shape != (n, self.state_dim):
            raise ValueError(f"expected ({n}, {self.state_dim}) states and ({n}, {self.action_dim}) actions")
        for cam in self.layout.cameras:
            if len(images.get(cam, [])) != n:
                raise ValueError(f"camera {cam} has {len(images.get(cam, []))} frames, expected {n}")

        ep = len(self.episodes)
        chunk = ep // CHUNK_SIZE
        tidx = self.task_index(task)
        table = pa.table({
            "observation.state": pa.array(states.tolist(), type=pa.list_(pa.float32())),
            "action": pa.array(actions.tolist(), type=pa.list_(pa.float32())),
            "timestamp": pa.array(np.arange(n, dtype=np.float32) / self.fps),
            "frame_index": pa.array(np.arange(n, dtype=np.int64)),
            "episode_index": pa.array(np.full(n, ep, dtype=np.int64)),
            "index": pa.array(np.arange(self.total_frames, self.total_frames + n, dtype=np.int64)),
            "task_index": pa.array(np.full(n, tidx, dtype=np.int64)),
            f"annotation.{self.layout.annotation}": pa.array(np.full(n, tidx, dtype=np.int64)),
            "next.reward": pa.array(np.zeros(n, dtype=np.float32)),
            "next.done": pa.array([False] * (n - 1) + [True]),
        })
        data_path = self.root / f"data/chunk-{chunk:03d}/episode_{ep:06d}.parquet"
        data_path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, data_path)

        for cam in self.layout.cameras:
            frames = images[cam]
            h, w = frames[0].shape[:2]
            self.image_shapes[cam] = (h, w)
            out = self.root / f"videos/chunk-{chunk:03d}/observation.images.{cam}/episode_{ep:06d}.mp4"
            out.parent.mkdir(parents=True, exist_ok=True)
            encode_h264(frames, out, self.fps)

        self.episodes.append({"episode_index": ep, "tasks": [task], "length": n})
        self.total_frames += n
        self._write_meta()
        return ep

    def _write_meta(self):
        meta = self.root / "meta"
        (meta / "tasks.jsonl").write_text("".join(json.dumps({"task_index": i, "task": t}) + "\n"
                                                  for i, t in enumerate(self.tasks)))
        (meta / "episodes.jsonl").write_text("".join(json.dumps(e) + "\n" for e in self.episodes))
        features = {
            "observation.state": {"dtype": "float32", "shape": [self.state_dim], "names": self.names[: self.state_dim]},
            "action": {"dtype": "float32", "shape": [self.action_dim], "names": self.names[: self.action_dim]},
            "timestamp": {"dtype": "float32", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
            f"annotation.{self.layout.annotation}": {"dtype": "int64", "shape": [1]},
            "next.reward": {"dtype": "float32", "shape": [1]},
            "next.done": {"dtype": "bool", "shape": [1]},
        }
        for cam, (h, w) in self.image_shapes.items():
            features[f"observation.images.{cam}"] = {
                "dtype": "video", "shape": [h, w, 3], "names": ["height", "width", "channel"],
                "video_info": {"video.fps": self.fps, "video.codec": "h264", "video.pix_fmt": "yuv420p",
                               "video.is_depth_map": False, "has_audio": False},
            }
        n_eps = len(self.episodes)
        info = {
            "codebase_version": "v2.1",
            "robot_type": self.robot_type,
            "total_episodes": n_eps,
            "total_frames": self.total_frames,
            "total_tasks": len(self.tasks),
            "total_videos": n_eps * len(self.layout.cameras),
            "total_chunks": (n_eps - 1) // CHUNK_SIZE + 1 if n_eps else 0,
            "chunks_size": CHUNK_SIZE,
            "fps": self.fps,
            "splits": {"train": f"0:{n_eps}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": features,
        }
        (meta / "info.json").write_text(json.dumps(info, indent=2))


def encode_h264(frames: list[np.ndarray], path: Path, fps: float):
    """Pipes RGB frames into ffmpeg (libx264, yuv420p). GR00T's torchcodec loader expects H.264."""
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg not found; install it (apt install ffmpeg) to record video")
    h, w = frames[0].shape[:2]
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
           "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-g", "2", str(path)]
    proc = subprocess.run(cmd, input=b"".join(np.ascontiguousarray(f, dtype=np.uint8).tobytes() for f in frames),
                          capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg failed: {proc.stderr.decode()[-500:]}")
