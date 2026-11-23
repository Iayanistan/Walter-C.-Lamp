"""Forward spill-detection PoC events to the Walter supervisor over UDP.

Drop this file into the PoC's spillpoc/ package and add two lines to EventLog
(spillpoc/events.py), so a camera spill halts the real or simulated arm:

    from .spill_udp import forward          # top of events.py
    forward(rec)                            # at the end of EventLog.log(), after self._f.flush()

Set WALTER_SUPERVISOR=<ros machine ip>:50555 in the environment (default 127.0.0.1:50555).
Fire-and-forget: if nothing is listening, the PoC carries on as before.
"""

import json
import os
import socket

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_host, _port = os.environ.get("WALTER_SUPERVISOR", "127.0.0.1:50555").rsplit(":", 1)


def forward(record: dict) -> None:
    try:
        _sock.sendto(json.dumps(record).encode(), (_host, int(_port)))
    except OSError:
        pass
