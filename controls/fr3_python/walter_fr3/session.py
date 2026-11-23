"""Command-line plumbing shared by d4607.py, run.py and scripts exported from the procedure builder.

    python my_procedure.py                      dry run, virtual clock, operator steps auto-confirmed
    python my_procedure.py --ask                dry run, but stop at every operator step
    python my_procedure.py --realtime           dry run at real speed
    python my_procedure.py --ip 192.168.58.2    the real FR3 (or FAIRINO's SimMachine)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Callable

from .arm import ArmError, DryRunArm, FairinoArm, Halted
from .helpers import DEFAULT_STATIONS, Operator, ProcedureError, RunLog, Walter, listen_for_spills


def parser(description: str | None = None) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("--ip", help="FR3 controller address. Leave out for a dry run with no robot.")
    ap.add_argument("--stations", default=str(DEFAULT_STATIONS), help="stations YAML with taught poses")
    ap.add_argument("--realtime", action="store_true", help="dry run at real speed instead of a virtual clock")
    ap.add_argument("--ask", action="store_true", help="dry run: wait for a person at operator steps")
    ap.add_argument("--max-vel", type=float, default=30.0, help="cap on MoveJ speed, percent of rated")
    ap.add_argument("--log-dir", default="runs", help="where run logs and reports go")
    ap.add_argument("--spill-port", type=int, default=0, help="UDP port for spill-PoC halts (50555), 0 = off")
    ap.add_argument("--quiet", action="store_true", help="dry run: do not print every arm move")
    return ap


class Session:
    def __init__(self, args):
        self.args = args
        self.dry = not args.ip
        self.dir = Path(args.log_dir) / time.strftime("%Y%m%d_%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        if self.dry:
            arm = DryRunArm(realtime=args.realtime, verbose=not args.quiet)
            op = Operator(auto=not args.ask)
        else:
            st = __import__("yaml").safe_load(Path(args.stations).read_text()).get("gripper", {})
            arm = FairinoArm(args.ip, gripper_do=int(st.get("tool_do", 0)), close_level=int(st.get("close_level", 1)))
            op = Operator(auto=False)
        if args.spill_port:
            listen_for_spills(arm, args.spill_port)
        self.walter = Walter(arm, args.stations, op, RunLog(self.dir / "events.jsonl"), max_vel_pct=args.max_vel)

    def run(self, procedure: Callable[[Walter], object]) -> int:
        w = self.walter
        print(f"== {'DRY RUN (no robot)' if self.dry else 'FR3 at ' + self.args.ip} == log: {self.dir}")
        try:
            procedure(w)
        except Halted as e:
            print(f"\nHALTED: {e}. The arm was stopped where it is; clear the cell before restarting.")
            w.log("halted", reason=str(e))
            return 2
        except (ProcedureError, ArmError) as e:
            w.arm.halt(str(e))
            print(f"\nSTOPPED: {e}")
            w.log("error", reason=str(e))
            return 1
        except KeyboardInterrupt:
            w.arm.halt("Ctrl-C")
            print("\nStopped by Ctrl-C.")
            return 130
        finally:
            w.arm.close()
        if w.run_log.deviations:
            print(f"\nFinished with {len(w.run_log.deviations)} timed-step deviation(s); see the log.")
        else:
            print("\nFinished.")
        return 0


def open_walter(argv=None, description: str | None = None) -> Session:
    return Session(parser(description).parse_args(argv))


def main_for(procedure: Callable[[Walter], object], description: str | None = None):
    sys.exit(open_walter(description=description).run(procedure))
