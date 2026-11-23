"""Stand-in for the GR00T policy server, for dry runs without a GPU.

Speaks the same ZMQ protocol as Isaac-GR00T's run_gr00t_server.py and returns a small, slow sweep
of joint_1 around the current state (and holds the gripper), so the whole ROS side
(bridge -> safety filter -> controller) can be exercised on mock hardware.

    ros2 run walter_control fake_policy_server --port 5555
"""

from __future__ import annotations

import argparse

import numpy as np
import zmq

from .gr00t_client import from_bytes, to_bytes


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--horizon", type=int, default=16)
    ap.add_argument("--step", type=float, default=0.01, help="rad per action step on joint_1")
    args = ap.parse_args(argv)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://*:{args.port}")
    print(f"fake policy server on tcp://*:{args.port}")
    direction, calls = 1.0, 0
    while True:
        req = from_bytes(sock.recv())
        endpoint = req.get("endpoint", "get_action")
        if endpoint == "get_action":
            obs = req["data"]["observation"]
            arm_state = np.asarray(obs["state"]["single_arm"], dtype=np.float32)[:, -1]  # (B, 6)
            grip_state = np.asarray(obs["state"]["gripper"], dtype=np.float32)[:, -1]
            calls += 1
            if calls % 4 == 0:
                direction = -direction
            steps = np.arange(1, args.horizon + 1, dtype=np.float32)[None, :, None]
            arm = np.repeat(arm_state[:, None], args.horizon, axis=1)
            arm[..., 0:1] += direction * args.step * steps[..., :1]
            grip = np.repeat(grip_state[:, None], args.horizon, axis=1)
            reply = [{"single_arm": arm.astype(np.float32), "gripper": grip.astype(np.float32)}, {}]
        elif endpoint == "kill":
            sock.send(to_bytes({}))
            break
        else:  # ping, reset, anything else
            reply = {"status": "ok"}
        sock.send(to_bytes(reply))
    sock.close(linger=0)
    ctx.term()


if __name__ == "__main__":
    main()
