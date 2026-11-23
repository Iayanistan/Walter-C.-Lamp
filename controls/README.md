# Walter controls: FAIRINO FR3 (and ABB IRB 120) + ROS 2 Jazzy + GR00T N1.7

Framework for driving a bought arm with ROS 2 and NVIDIA GR00T, aimed at ASTM D4607 sections 10.4
to 10.5. The final arm is a **FAIRINO FR3** (`robot:=fr3`), which runs the full sequence. The ABB
IRB 120 with RobotStudio (`robot:=irb120`, the default) is used only for basic commands.

FAIRINO's FRCap is a plug-in system for the robot's WebApp (Node.js + Vue pages running in the
controller); it cannot host ROS 2 or GR00T. The FR3 is driven from ROS 2 through FAIR-INNOVATION's
own ros2_control plugin (`fairino_hardware` in github.com/FAIR-INNOVATION/frcobot_ros2), and
GR00T stays on the GPU machine behind our bridge, exactly as with the IRB 120.

| Path | What |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Whole-system architecture for the FR3 (updated 2026-11-04): FAIRINO vs our software, each subsystem, safety layers, Plan B, known gaps. Diagram: [docs/architecture_fr3.png](docs/architecture_fr3.png). The old IRB 120 version is in docs/ARCHITECTURE_irb120_2026-10-20.md. |
| [docs/WORKFLOW.md](docs/WORKFLOW.md) | Phase-by-phase plan (mock, Isaac, sim data + fine-tune, RobotStudio, real arm) with exit checks, plus an arm purchase checklist. |
| [walter_ws/](walter_ws/) | The ROS 2 Jazzy workspace (7 packages). Start with its README. |
| [sim/isaac/](sim/isaac/) | Isaac Sim 5.x scene script wired to ros2_control and the cameras. |
| [fr3_python/](fr3_python/) | Plain-Python FR3 helpers (FAIRINO SDK, no ROS) and the full three-flask ASTM D4607 run; the procedure builder's blocks are these helpers. Dry-run tested only. |
| [vla/](vla/) | GR00T modality config, fine-tune and serve scripts. |
| [integrations/spill_udp.py](integrations/spill_udp.py) | Two-line hook so the spill-detection PoC can halt the arm. |

Verified so far: the workspace builds on Jazzy, all unit tests pass, and on mock hardware the
policy loop (with a stand-in GR00T server), the D4607 task runner, a skill procedure with an AprilTag
placement failure recovered by the policy (fake tags), gamepad teleop recording through MoveIt
Servo, a procedure built and run from the browser app, the spill halt and demo
recording all ran end to end. The ABB EGM driver compiles on Jazzy and loads the IRB 120
description up to the point where it waits for a controller. Not yet run: the Isaac Sim script,
a real GR00T checkpoint, and anything against RobotStudio or a real IRC5.

FR3 (2026-10-26): the FR3 model, FK (checked against the URDF), controllers, safety limits, MoveIt
config and placeholder station poses are in, and the same skill procedure (including the
misplaced-funnel recovery) ran on mock hardware with `robot:=fr3`. Not yet built or run:
FAIR-INNOVATION's `fairino_hardware` plugin, the FR3 SimMachine, or a real FR3.
