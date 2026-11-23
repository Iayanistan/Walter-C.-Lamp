# Walter controls architecture (ABB IRB 120 + ROS 2 Jazzy + GR00T N1.7)

## Decisions

| Choice | Pick | Why |
|---|---|---|
| ROS 2 distro | **Jazzy** (Ubuntu 24.04, LTS to May 2029) | Humble ends May 2027, mid-project. Isaac Sim 5.x ships Jazzy support. GR00T N1.7 uses Python 3.12, the same as Jazzy. |
| Arm driver | **PickNik `abb_ros2`** (`abb_hardware_interface`) | ros2_control plugin that streams joint positions over EGM at 250 Hz and reads controller info over RWS. It is the maintained ROS 2 path for ABB arms. |
| Simulator | **Isaac Sim 5.x** via `topic_based_ros2_control` | The sim becomes a fourth "hardware" option. Nothing above ros2_control knows whether it is driving Isaac or the IRC5. |
| VLA | **GR00T N1.7** (`nvidia/GR00T-N1.7-3B`, `NEW_EMBODIMENT` fine-tune) | Current GA release, Apache 2.0. Runs as a separate ZMQ policy server, so PyTorch never enters the ROS process. |

## How ROS 2 fits with the ABB software

ROS 2 does not replace the ABB controller. The IRC5 keeps running RobotWare: servo loops, brakes,
the e-stop chain, the FlexPendant, and its own joint and speed limits. ROS 2 runs on a Linux PC
next to it and plays the role of a motion source.

```
 Linux PC (ROS 2 Jazzy)                                         IRC5 Compact (RobotWare 6.x)
 ┌──────────────────────────────────────────────┐              ┌──────────────────────────────┐
 │ task_runner ── MoveIt ──┐                     │              │ RAPID: EGMActJoint/EGMRunJoint│
 │ vla_bridge ─ safety ────┼─> joint_trajectory_ │   EGM/UDP    │ hands joint position control  │
 │              filter     │   controller        │<──250 Hz────>│ to the PC while it runs       │
 │ supervisor (modes, halt)│        │            │              │                               │
 │                         └────────┘            │   RWS/HTTP   │ servo, brakes, e-stop, limits │
 │ ros2_control: abb_hardware_interface ─────────┼─────────────>│ config, state, I/O            │
 └──────────────────────────────────────────────┘              └──────────────────────────────┘
```

- **EGM** (Externally Guided Motion, RobotWare option **689-1**) is the streaming channel. A small
  RAPID program on the controller starts an EGM session; from then on the PC sends joint targets
  every 4 ms and gets joint feedback back. The `robot_studio_resources` folder in `abb_ros2` has an
  example RAPID module and RobotStudio station to start from.
- **RWS** (Robot Web Services, HTTP) is used at start-up to read the robot's configuration, and can
  set digital I/O, which is one way to drive a gripper. `abb_ros2` supports RWS 1.0 only, which
  means **RobotWare 6.x on an IRC5**. On an OmniCore controller (RobotWare 7, RWS 2.0) launch with
  `configure_via_rws:=false` so the driver takes joint info from the URDF instead.
- If the controller lacks EGM, the fallback is a RAPID socket server that executes whole MoveIt
  trajectories. That works for the scripted steps but cannot follow GR00T chunks smoothly, so
  **EGM is a must-have when buying the arm**.

## One description, four back ends

`walter_cell.urdf.xacro hardware:=...` picks the ros2_control plugin. Controllers, MoveIt, the
safety filter, the GR00T bridge and the recorder are identical across all of them.

| `hardware` | Plugin | Use |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | Laptop dry runs and CI. Commands echo back as state. |
| `isaac` | `topic_based_ros2_control/TopicBasedSystem` | Isaac Sim digital twin, demo generation, closed-loop policy tests. |
| `abb` + RobotStudio IP | `abb_hardware_interface/ABBSystemHardware` | RobotStudio virtual controller on a Windows PC (same EGM/RWS path as the real arm). |
| `abb` + IRC5 IP | same | The real IRB 120. |

## Control modes

The supervisor owns one mode at a time and publishes it latched on `/walter/mode`.

| Mode | Who moves the arm | Path to the controller |
|---|---|---|
| IDLE | nobody | none |
| SCRIPTED | task runner or MoveIt | FollowJointTrajectory action, straight to `joint_trajectory_controller` |
| VLA | GR00T bridge | chunk → safety filter → `joint_trajectory_controller/joint_trajectory` topic |
| HALTED | nobody (latched) | supervisor sends a hold point; only `~/reset` leaves it |

HALTED is entered from `/supervisor/halt`, a `true` on `/walter/halt`, or a `spill_halt` event
from the spill-detection camera over UDP (`integrations/spill_udp.py`).

## The GR00T loop

```
 cameras (scene, wrist) ─┐
 /joint_states ──────────┼─> vla_bridge ──ZMQ──> GR00T policy server (GPU)
 instruction (action goal)┘      │  <── 16-step chunk (joint targets + gripper), ~15 Hz steps
                                 │ execute first 8 steps, then re-query
                                 ├─> /walter/vla/chunk ─> safety_filter ─> joint_trajectory_controller
                                 └─> gripper_position_controller (step by step)
```

- **Observation:** `video.scene`, `video.wrist` (320×240 RGB), `state.single_arm` (6 joint
  positions), `state.gripper` (opening), plus the language instruction.
- **Action:** `single_arm` (6 joint targets) and `gripper` (opening) for 16 future steps at 15 Hz.
  The modality config trains joint actions as deltas from the current state.
- **Safety filter** (`chunk_guard.py`): clamps to joint limits minus 0.05 rad, rejects a chunk that
  starts more than 0.15 rad from the measured pose or takes the TCP outside a box over the bench,
  and slows (never speeds up) any segment over 25% of the IRB 120's rated joint speeds. A rejected
  chunk sends a hold. Silence from the policy for over 1 s in VLA mode also sends a hold.
- **Process split:** the bridge only needs `pyzmq` and `msgpack-numpy`. The server can run on the
  same PC (needs a big GPU) or a separate GPU box on the LAN.

## Why scripted and learned steps are mixed

D4607 has hard timing (boil 30 ± 2 s, shake 30 ± 1 s) and "no delay" filtering, plus
manipulation that depends on where the glassware actually is. The task runner keeps the clock
and the deterministic motions, and calls GR00T only for perception-heavy skills:

| D4607 | Step | Done by |
|---|---|---|
| 10.4 | flask from rack to hot plate | GR00T `flask_to_hotplate` |
| 10.4 | boil 30 ± 2 s | task runner timer (logged, tolerance checked) |
| 10.4 | hot plate to cooling pad | GR00T `flask_to_cooling` |
| 10.5 | to the iodine dispenser | GR00T `flask_to_dispenser` |
| 10.5 | dispense 100 mL, stopper | operator for now |
| 10.5 | shake 30 ± 1 s | scripted J5 oscillation, ±10° at 2.5 Hz |
| 10.5 | pour into the funnel | operator unstoppers, GR00T `pour` |

Each run writes `~/.ros/walter_runs/<stamp>/events.jsonl`, in the same shape as the spill PoC's
log, with each step's duration and whether it met its D4607 tolerance.

## Safety layers (outermost wins)

1. IRC5 hardware e-stop, enabling device, and (if bought) SafeMove zones. Not software you own.
2. IRC5 axis limits and speed supervision.
3. Supervisor HALTED latch (software stop, also fed by the spill camera).
4. Safety filter on every GR00T chunk.
5. MoveIt collision checking on scripted moves (bench and robot only so far; add the hot plate,
   rack and funnels as collision objects once they have fixed positions).

Layers 3 to 5 are development aids. Run first trials with water, at reduced speed on the
FlexPendant, with the cell guarded.

## If a different arm is bought

Only `walter_description` changes: replace the IRB 120 macro and the joint list in
`walter.ros2_control.xacro`, and update the limits in `safety.yaml`, `joint_limits.yaml` and
`kinematics.py`. The new arm needs a ros2_control driver with a position interface it can follow
at 100 Hz or more. UR (`ur_robot_driver`), Franka FR3, Kinova Gen3 and UFactory xArm all have one.

## Known gaps and risks

- `abb_ros2` only ships a robot description for the IRB 1200. This workspace brings its own
  IRB 120 description and only uses the driver's hardware interface. Checked on 2026-10-20: the
  driver (rolling branch) compiles against Jazzy, loads this description, configures EGM on port
  6511 and waits for a controller. Nothing past that has been tested without RobotStudio or an IRC5.
- `walter_isaac_scene.py` is written against the Isaac Sim 5.0 API but has not been run.
- The SRDF collision matrix is hand-written. Re-run the MoveIt Setup Assistant once the gripper is real.
- The gripper is a placeholder (prismatic, 0 to 40 mm per finger) and is mock on the real arm.
- Pose values in `d4607.yaml` are placeholders that pass the workspace check. Teach real ones.
- The IRB 120 reaches 580 mm, slightly less than the 630 mm spec from the build-it-yourself BOM.
  Lay out the hood stations within about 550 mm of the base.
- The IRB 120 is IP30 in its standard version. Fume-hood HCl and iodine vapour call for a
  protective jacket or the clean-room/IP67 variant.
