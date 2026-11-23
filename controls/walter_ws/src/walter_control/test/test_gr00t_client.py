"""Round-trip against a stand-in server that speaks the Isaac-GR00T PolicyServer protocol."""

import threading

import numpy as np
import zmq

from walter_control.gr00t_client import Gr00tClient, from_bytes, to_bytes
from walter_control.observation import build_observation, image_to_rgb, resize_nearest, split_action


def fake_server(port, ready, horizon=16):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://127.0.0.1:{port}")
    ready.set()
    while True:
        req = from_bytes(sock.recv())
        ep = req.get("endpoint")
        if ep == "ping":
            sock.send(to_bytes({"status": "ok"}))
        elif ep == "reset":
            sock.send(to_bytes({}))
        elif ep == "get_action":
            obs = req["data"]["observation"]
            state = obs["state"]["single_arm"]          # (1, 1, 6)
            assert obs["video"]["scene"].dtype == np.uint8
            assert obs["language"]["annotation.human.task_description"] == [["pour"]]
            arm = np.repeat(state[:, :1], horizon, axis=1) + 0.01 * np.arange(1, horizon + 1)[None, :, None]
            grip = np.zeros((1, horizon, 1), dtype=np.float32)
            sock.send(to_bytes([{"single_arm": arm.astype(np.float32), "gripper": grip}, {}]))
        elif ep == "kill":
            sock.send(to_bytes({}))
            break
    sock.close(linger=0)
    ctx.term()


def test_round_trip():
    port = 55731
    ready = threading.Event()
    th = threading.Thread(target=fake_server, args=(port, ready), daemon=True)
    th.start()
    ready.wait(2)
    client = Gr00tClient("127.0.0.1", port, timeout_ms=2000)
    assert client.ping()
    q = np.array([0, 0, 0, 0, 1.57, 0])
    img = np.zeros((240, 320, 3), np.uint8)
    obs = build_observation({"scene": img, "wrist": img}, {"single_arm": q, "gripper": np.array([0.04])},
                            "pour", "annotation.human.task_description")
    action, info = client.get_action(obs)
    arm, grip = split_action(action, "single_arm", "gripper", q, deltas=False)
    assert arm.shape == (16, 6) and grip.shape == (16,)
    np.testing.assert_allclose(arm[0], q + 0.01, atol=1e-6)
    client.call("kill")
    client.close()
    th.join(2)


def test_timeout_raises():
    client = Gr00tClient("127.0.0.1", 55732, timeout_ms=200)
    assert not client.ping()
    client.close()


def test_image_conversion():
    bgr = np.zeros((2, 3, 3), np.uint8)
    bgr[..., 0] = 255  # blue in BGR
    rgb = image_to_rgb("bgr8", 2, 3, 9, bgr.tobytes())
    assert rgb[0, 0].tolist() == [0, 0, 255]
    assert resize_nearest(np.zeros((480, 640, 3), np.uint8), 320, 240).shape == (240, 320, 3)
