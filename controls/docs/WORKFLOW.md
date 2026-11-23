# Integration workflow: from laptop to a real D4607 run

Each phase reuses the same ROS 2 stack and only changes `hardware:=` or where the policy comes
from. Do not skip a phase's exit check: each one isolates a different failure source (software,
sim, controller networking, real-world perception).

```
 Phase 1 mock ─> 2 Isaac twin ─> 3 sim data + GR00T fine-tune ─> 4 RobotStudio (EGM)
                                                                       │
                     6 real data + closed loop  <──  5 real IRB 120 commissioning
```

## Phase 0: machines

| Machine | OS / software | Job |
|---|---|---|
| Control PC | Ubuntu 24.04, ROS 2 Jazzy, Isaac Sim 5.x, RTX GPU | Runs the whole ROS stack, Isaac Sim, and can serve GR00T if the GPU has about 16 GB free. |
| Training GPU | Linux, CUDA, 40 GB+ (H100/L40/A6000) or a cloud node | GR00T fine-tuning. Can also serve the policy over the LAN. |
| Windows PC | RobotStudio | Virtual IRC5 for phase 4, and programming the real controller. |
| IRC5 Compact | RobotWare 6.x with EGM (689-1) | The real arm, phase 5 onwards. |

The spill-detection PoC currently runs on Windows. It can stay there; it reaches the supervisor
over UDP (`integrations/spill_udp.py`).

## Phase 1: mock hardware (done, rerun on your machine)

```bash
cd ~/walter_ws && colcon build && source install/setup.bash && colcon test
ros2 launch walter_bringup bringup.launch.py
```
Then the dry-run commands in `walter_ws/README.md` (fake policy server, test-pattern camera, task runner).

**Exit check:** 22 tests pass. RViz shows the cell. A `RunPolicy` goal moves the arm and
`/walter/safety/status` counts forwarded chunks. The task runner finishes and its `events.jsonl`
marks the boil and shake steps `within_tolerance: true`. A UDP `spill_halt` puts the mode in HALTED.

## Phase 2: Isaac Sim digital twin

1. Lay out the real hood: rack, hot plate, cooling pad, dispenser, funnels, measured from the
   arm base. Put those numbers into `walter_cell.urdf.xacro` (bench, cameras), `walter_isaac_scene.py`
   (objects) and the MoveIt planning scene.
2. Follow `sim/isaac/README.md` to start the scene, then `bringup.launch.py hardware:=isaac`.
3. Teach the `poses:` in `d4607.yaml` by jogging in RViz (MoveIt motion planning panel) and copying `/joint_states`.

**Exit check:** the task runner completes `d4607.yaml` in sim with every `vla` step swapped for
a scripted move (or skipped). Both camera topics publish at about 15 Hz. `ros2 topic hz /clock` is steady.

## Phase 3: sim demonstrations and the first GR00T fine-tune

1. One dataset per skill: `flask_to_hotplate`, `flask_to_cooling`, `flask_to_dispenser`, `pour`.
2. Generate demos. The cheapest is scripted pick-and-place in sim with randomised flask positions
   (new `--seed` per batch) and the `episode_recorder` running. Teleop through MoveIt Servo gives
   more natural motion; add it once the scripted pipeline works. Aim for 50 to 200 episodes per skill.
3. Copy the datasets to the training GPU and run `vla/finetune.sh`.
4. Open-loop eval on a training episode (command printed by the script). Predictions should follow
   the recorded curves.
5. Serve with `vla/serve.sh`, then run the task file in Isaac with the real `vla` steps.
   First run `serve.sh` in **REPLAY** mode on a recorded dataset: if the arm retraces the demo,
   the plumbing and `actions_are_deltas` are right.

**Exit check:** each skill succeeds in at least 8 of 10 sim trials from randomised starts, with
zero rejected chunks in `/walter/safety/status`.

## Phase 4: RobotStudio virtual controller (before touching the arm)

1. Build the driver: `cd src && vcs import < ../walter.repos`, `rosdep install ...`, `colcon build`.
2. In RobotStudio, create an IRB 120 + IRC5 station with RobotWare 6.x and the EGM option.
   Start from the RAPID EGM module and network setup in `abb_ros2/docs` (`RobotStudioSetup.md`,
   `NetworkingConfiguration.md`). Set the EGM UDP device to the control PC's IP, port 6511.
3. `ros2 launch walter_bringup bringup.launch.py hardware:=abb rws_ip:=<windows pc ip>`
4. Start the RAPID task. The driver logs the EGM connection; then run the task file with scripted steps.

**Exit check:** scripted task completes on the virtual controller with the same timing log as in
sim. A GR00T skill runs against the virtual controller with Isaac still providing the camera images
(this proves the policy loop over EGM before any real motion).

## Phase 5: real IRB 120 commissioning

1. Network: a dedicated NIC to the IRC5 WAN port with a static IP. EGM is UDP at 250 Hz, so no
   Wi-Fi and no shared switch with camera traffic. Check that the PC firewall allows UDP 6511 in.
2. Copy the RobotStudio RAPID/EGM config to the real controller. Set EGM position limits and
   speed limits on the controller side as a second net.
3. Fit the gripper, update the description (mass, TCP, finger geometry) and the gripper
   ros2_control block, and calibrate both cameras (intrinsics, plus the scene camera pose relative
   to `bench`).
4. Bring up with `hardware:=abb rws_ip:=<irc5 ip>` at reduced speed on the FlexPendant. Run
   scripted moves only, then the shake step empty, then with a water-filled flask.

**Exit check:** full scripted D4607 sequence with water, all timing tolerances met, spill camera
halt verified by deliberately spilling water.

## Phase 6: real data and closed-loop GR00T

1. Record 20 to 50 real demos per skill (teleop) with the same recorder.
2. Fine-tune on sim + real together (`DATASET="<sim dirs> <real dirs>"`).
3. Run each skill with water, then the full hybrid task with water, then with reagents under
   supervision.

**Exit check:** three consecutive full D4607 10.3 to 10.5 runs with water and no operator
intervention beyond the planned operator steps.

## Arm purchase checklist (things this stack depends on)

- [ ] IRC5 (Compact is fine for the IRB 120) with **RobotWare 6.x**. An OmniCore controller needs `configure_via_rws:=false`.
- [ ] **EGM option 689-1** licensed. Without it there is no 250 Hz streaming.
- [ ] Spare digital outputs or a fieldbus for the gripper.
- [ ] FlexPendant and enabling device included.
- [ ] Optional: SafeMove for certified speed and zone limits.
- [ ] Protection for fume-hood use (jacket, or the clean-room/IP67 variant).
