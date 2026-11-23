# walter_fr3: Python helpers for the FAIRINO FR3 and the full ASTM D4607 run

Plain Python, no ROS. Needs `numpy` and `pyyaml`; the real arm also needs FAIRINO's Python SDK
(`from fairino import Robot`).

```
cd controls/fr3_python
python -m walter_fr3.d4607                     # whole D4607 run, dry: no robot, virtual clock
python -m walter_fr3.d4607 --ask               # dry run that stops at every operator step
python -m walter_fr3.d4607 --ip 192.168.58.2   # the real FR3 (or FAIRINO's SimMachine)
python -m walter_fr3.run my_procedure.yaml     # run a procedure from the builder's YAML tab
python -m walter_fr3.teach --ip 192.168.58.2 --pose grasp   # print the current pose for the stations file
python -m pytest tests                         # 9 tests, includes a builder-vs-helpers sync check
```

Every run writes `runs/<time>/events.jsonl`, and `d4607_report.json` when a report is made.

| File | What |
|---|---|
| `walter_fr3/helpers.py` | The helper functions (class `Walter`). One helper = one block in the procedure builder, same name, same parameters. |
| `walter_fr3/d4607.py` | The whole standard as a sequence of helper calls, three flasks. Its docstring lists who does each section. |
| `walter_fr3/calc.py` | Section 11: X/M, C, log-log fit, iodine number, and the 11.4 dosage estimate. |
| `walter_fr3/arm.py` | `DryRunArm` and `FairinoArm` (SDK: MoveJ, ServoJ, SetToolDO). Joint-limit and speed checks on every move. |
| `walter_fr3/stations_fr3_d4607.yaml` | Stations and taught poses. **All poses are placeholders: teach them before moving glassware.** |
| `walter_fr3/run.py`, `session.py` | Run builder YAML; shared command-line options (`--ip`, `--ask`, `--realtime`, `--max-vel`, `--spill-port 50555`). |

## Helpers

`home` `move_to` `pick` `place` `hold_under` `dispense` `pour` `swirl` `shake` `boil` `wait` `cool`
`verify` `operator` `record` `report`

```python
from walter_fr3.session import main_for

def procedure(w):
    w.items["rack_1"] = "flask_1"        # what is on the bench at the start
    w.pick("rack_1")
    w.swirl(seconds=10)
    w.place("hotplate")
    w.boil(seconds=30, tolerance_s=2)    # times the boil, lifts the flask off on time
    w.place("cool_pad")
    w.home()

main_for(procedure)
```

## What the arm does and does not do in D4607

The arm: swirl to wet the carbon (10.4), move flasks on and off the hot plate with the 30 ± 2 s boil
timed (10.4), hold each flask for the iodine (10.5), shake 30 ± 1 s (10.5), pour into the funnel
(10.5), swirl each filtrate (10.6). Python: dosage estimate (11.4), calculation and report (11, 12).

The operator: making and standardizing solutions (8, 9), grinding and drying (10.1, 10.2), weighing
(10.3), pipetting HCl and filtrate, stoppers on and off, the pipet rinse, and the titration (10.6).
One gripper cannot work a stopper on a flask it is holding, and nothing in the cell reads a balance
or a titration endpoint.

## Not yet done

- `FairinoArm` follows FAIRINO's SDK reference and passes against a fake SDK in the tests, but has
  never talked to a controller. First run: SimMachine, then the real arm with `--max-vel 10` and water.
- No camera here: `verify` asks the operator unless you pass `Walter(checker=...)`.
- Do not run this and the ROS 2 `fairino_hardware` driver against the controller at the same time.
