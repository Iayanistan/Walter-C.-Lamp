"""Run a procedure exported from the procedure builder's YAML tab on the FR3 helpers.

    python -m walter_fr3.run my_procedure.yaml                 # dry run
    python -m walter_fr3.run my_procedure.yaml --ip 192.168.58.2

Each step `- block: {params}` becomes `walter.block(**params)`. A `# Start: flask_1 at rack_1`
comment, or a `start:` list, tells the helpers what sits where when the run begins.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml

from .helpers import HELPERS, ProcedureError, Walter
from .session import parser, Session


def load(path: str | Path) -> dict:
    text = Path(path).read_text()
    proc = yaml.safe_load(text) or {}
    if "start" not in proc:
        m = re.search(r"^# Start: (.+)$", text, re.M)
        proc["start"] = [dict(zip(("item", "at"), part.strip().split(" at "))) for part in m.group(1).split(",")] if m else []
    return proc


def check(proc: dict):
    steps = proc.get("steps") or []
    if not steps:
        raise ProcedureError("procedure has no steps")
    for i, entry in enumerate(steps, 1):
        if not isinstance(entry, dict) or len(entry) != 1:
            raise ProcedureError(f"step {i}: write each step as '- block_name: {{params}}'")
        name = next(iter(entry))
        if name not in HELPERS:
            raise ProcedureError(f"step {i}: unknown block '{name}' (known: {', '.join(HELPERS)})")


def run_procedure(w: Walter, proc: dict):
    check(proc)
    for o in proc.get("start") or []:
        w.items[o["at"]] = o["item"]
    for i, entry in enumerate(proc["steps"], 1):
        (name, params), = entry.items()
        print(f"\n[{i:02d}] {name} {params or ''}")
        try:
            w.call(name, params or {})
        except ProcedureError as e:
            raise ProcedureError(f"step {i} ({name}): {e}") from None


def main(argv=None):
    ap = parser(__doc__.splitlines()[0])
    ap.add_argument("procedure", help="YAML file from the builder's YAML tab")
    args = ap.parse_args(argv)
    proc = load(args.procedure)
    sys.exit(Session(args).run(lambda w: run_procedure(w, proc)))


if __name__ == "__main__":
    main()
