"""Minimal client for the Isaac-GR00T policy server (gr00t/eval/run_gr00t_server.py).

Speaks the same ZMQ REQ/REP + msgpack-numpy protocol as gr00t.policy.server_client.PolicyClient
(Isaac-GR00T main, N1.7), so the ROS 2 side only needs pyzmq, msgpack and msgpack-numpy instead of
the whole GR00T/PyTorch stack. If the upstream wire format changes, swap this for
`from gr00t.policy.server_client import PolicyClient`, which has the same get_action() signature.
"""

from __future__ import annotations

import io
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq


def _encode(obj):
    if isinstance(obj, np.ndarray) and obj.dtype.kind == "O":
        raise TypeError("object-dtype arrays are refused by the GR00T server; use a numeric dtype")
    return mnp.encode(obj)


def _decode(obj):
    if isinstance(obj, dict):
        marker = obj.get("__ndarray_class__", obj.get(b"__ndarray_class__"))
        if marker:
            return np.load(io.BytesIO(obj.get("as_npy", obj.get(b"as_npy"))), allow_pickle=False)
        nd, kind = obj.get(b"nd", obj.get("nd")), obj.get(b"kind", obj.get("kind"))
        if nd and kind in (b"O", "O"):
            raise ValueError("refusing to decode a pickled object array")
    return mnp.decode(obj)


def to_bytes(data: Any) -> bytes:
    return msgpack.packb(data, default=_encode)


def from_bytes(data: bytes) -> Any:
    return msgpack.unpackb(data, object_hook=_decode, raw=False)


class Gr00tClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 5555, timeout_ms: int = 2000,
                 api_token: str | None = None):
        self.host, self.port, self.timeout_ms, self.api_token = host, port, timeout_ms, api_token
        self._ctx = zmq.Context()
        self._sock = None
        self._connect()

    def _connect(self):
        if self._sock is not None:
            self._sock.close(linger=0)
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self._sock.connect(f"tcp://{self.host}:{self.port}")

    def call(self, endpoint: str, data: dict | None = None) -> Any:
        request: dict = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        if self.api_token:
            request["api_token"] = self.api_token
        try:
            self._sock.send(to_bytes(request))
            reply = self._sock.recv()
        except zmq.error.Again:
            # A REQ socket that timed out is stuck waiting for a reply; start a fresh one.
            self._connect()
            raise TimeoutError(f"GR00T server at {self.host}:{self.port} did not answer in {self.timeout_ms} ms")
        response = from_bytes(reply)
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"GR00T server error: {response['error']}")
        return response

    def ping(self) -> bool:
        try:
            self.call("ping")
            return True
        except (TimeoutError, zmq.error.ZMQError):
            return False

    def get_action(self, observation: dict, options: dict | None = None) -> tuple[dict, dict]:
        action, info = self.call("get_action", {"observation": observation, "options": options})
        return action, info

    def reset(self, options: dict | None = None):
        return self.call("reset", {"options": options})

    def close(self):
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None
        self._ctx.term()
