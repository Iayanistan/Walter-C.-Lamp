import json
import shutil

import numpy as np
import pyarrow.parquet as pq
import pytest

from walter_control.lerobot_writer import LeRobotWriter


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="needs ffmpeg")
def test_two_episodes(tmp_path):
    w = LeRobotWriter(tmp_path, fps=15)
    for n, task in [(10, "pour the flask into the funnel"), (7, "place the flask on the hot plate")]:
        frames = [np.full((48, 64, 3), i * 10, np.uint8) for i in range(n)]
        w.write_episode(task, np.zeros((n, 7)), np.ones((n, 7)), {"scene": frames, "wrist": frames})

    info = json.loads((tmp_path / "meta/info.json").read_text())
    assert info["total_episodes"] == 2 and info["total_frames"] == 17 and info["total_tasks"] == 2
    modality = json.loads((tmp_path / "meta/modality.json").read_text())
    assert modality["state"]["gripper"] == {"start": 6, "end": 7}
    assert modality["video"]["wrist"]["original_key"] == "observation.images.wrist"

    t = pq.read_table(tmp_path / "data/chunk-000/episode_000001.parquet").to_pydict()
    assert t["index"][0] == 10 and t["annotation.human.task_description"][0] == 1 and t["next.done"][-1]
    assert (tmp_path / "videos/chunk-000/observation.images.scene/episode_000001.mp4").stat().st_size > 0

    # Reopening appends instead of overwriting.
    w2 = LeRobotWriter(tmp_path, fps=15)
    assert len(w2.episodes) == 2 and w2.total_frames == 17
