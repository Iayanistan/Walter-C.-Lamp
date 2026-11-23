# Procedure builder (blocks = FR3 Python helpers)

Open `index.html` in any browser. No server, no ROS, no arm.

- Scratch-style editor: drag blocks from the shelf into one snap-together script, drag them to reorder, drag back to the shelf to delete (with undo). Edit a block's white boxes and dropdowns right on the block; click a block to see all its fields in the right-hand panel; the bench plan and dry-run controls sit at the top of that column.
- Compose procedures from 16 blocks. Each block is one helper in `../fr3_python/walter_fr3/helpers.py` with the same name and parameters (a test in `../fr3_python/tests` fails if they drift). Stations mirror `walter_fr3/stations_fr3_d4607.yaml`; you can add your own.
- `d4607_three_flasks` is the whole standard, the same run as `python -m walter_fr3.d4607`.
- The Python tab exports the procedure as a script of helper calls; the YAML tab exports the same thing for `python -m walter_fr3.run file.yaml`. Both dry-run with no robot and drive the FR3 with `--ip`.
- Checks run in the browser: required parameters, gripper and object state step by step, tagged stations, poses, timed windows, recovery coverage. MoveIt reachability is not checked.
- Draft -> Sim passed (dry run is an animation on a top-down bench plan) -> Approved (named sign-off). Any edit goes back to draft. "Run on arm" is disabled.
- The ROS 2 runner (`ros2 run walter_skills run`) reads the same YAML but only knows the original ten blocks (not swirl, dispense, boil, cool, record, report).
- Procedures live in the browser's localStorage only.

Wiring it up later: swap the in-browser `analyze()` for `/api/compile` and `/api/skills` from `walter_app/server.py` (or the FastAPI backend in the guide), and enable Run on arm against `/api/run`.
