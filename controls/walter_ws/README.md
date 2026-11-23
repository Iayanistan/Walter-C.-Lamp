# walter_ws: ROS 2 Jazzy workspace for the Walter cell

Ubuntu 24.04 + ROS 2 Jazzy. Two arms, picked with `robot:=`:

* `irb120` (default): ABB IRB 120 + RobotStudio, used only to show basic control.
* `fr3`: FAIRINO FR3, the final arm that runs the full D4607 sequence. Real-arm driver is
  FAIR-INNOVATION's `fairino_hardware` (`hardware:=fairino`); see `walter_fr3.repos`.

Station poses live in `stations.yaml` (IRB 120) and `stations_fr3.yaml` (FR3); point
`WALTER_STATIONS` at the one for the arm in use. Everything else (skills, procedures, task runner,
safety filter, GR00T bridge, app) is shared. See
[../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md) for how the pieces fit and
[../docs/WORKFLOW.md](../docs/WORKFLOW.md) for the order to bring them up.

| Package | What it holds |
|---|---|
| `walter_description` | Cell URDF: bench, IRB 120 (ROS-Industrial kinematics and meshes), placeholder gripper, TCP, scene and wrist cameras. `robot` picks IRB 120 or FR3; `hardware` picks mock, Isaac, ABB or Fairino ros2_control. |
| `walter_bringup` | `bringup.launch.py` (robot, controllers, MoveIt, safety filter, supervisor), `vla.launch.py` (GR00T bridge), controller, safety and VLA configs. |
| `walter_moveit_config` | MoveIt 2 config: `arm` group base_link to tcp, `home`/`ready` poses, KDL, OMPL. |
| `walter_interfaces` | `SafetyStatus` msg, `SetMode`, `StartEpisode` and `CheckPlacement` srvs, `RunPolicy` action. |
| `walter_control` | Supervisor, safety filter, GR00T bridge, D4607 task runner, episode recorder, gamepad teleop (MoveIt Servo), IRC5 I/O gripper bridge, plus a fake policy server and test-pattern camera for dry runs. |
| `walter_app` | Browser procedure builder (`ros2 run walter_app server`, http://localhost:8080): compose skills, check, save, run, confirm, halt. |
| `walter_skills` | Skill library (pick, place, pour, shake...), `stations.yaml`, procedure compiler/runner, `teach` tool, AprilTag `placement_checker`, and `fake_tags` for dry runs. |

## Install

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-moveit ros-jazzy-topic-based-ros2-control ros-jazzy-xacro python3-vcstool ffmpeg
pip install --break-system-packages msgpack-numpy   # no apt package; or use a venv with --system-site-packages

mkdir -p ~/walter_ws && cp -r <this folder>/* ~/walter_ws/ && cd ~/walter_ws
# Only for the real IRB 120 / RobotStudio:
(cd src && vcs import < ../walter.repos)
# For the FR3 (any robot:=fr3 run needs fairino_description; read the notes in the file first):
(cd src && vcs import < ../walter_fr3.repos)
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash
colcon test && colcon test-result --verbose
```

## Dry run with no robot and no GPU

```bash
ros2 launch walter_bringup bringup.launch.py                 # mock hardware, MoveIt, RViz
# or the FR3 model: robot:=fr3, with WALTER_STATIONS=src/walter_skills/config/stations_fr3.yaml
ros2 run walter_control fake_policy_server                   # stands in for GR00T
ros2 run walter_control test_pattern_camera                  # stands in for the cameras
ros2 launch walter_bringup vla.launch.py
ros2 action send_goal /walter/run_policy walter_interfaces/action/RunPolicy "{instruction: 'test', max_duration: 6.0}"
ros2 run walter_control task_runner                          # runs config/d4607.yaml; confirm operator steps with
ros2 service call /task_runner/confirm std_srvs/srv/Trigger
```

## Useful topics and services

| Name | Type | Notes |
|---|---|---|
| `/walter/mode` | std_msgs/String (latched) | IDLE, SCRIPTED, VLA, HALTED |
| `/supervisor/set_mode` | walter_interfaces/srv/SetMode | refused while HALTED |
| `/supervisor/halt`, `/supervisor/reset` | std_srvs/Trigger | software stop and un-latch |
| `/walter/halt` | std_msgs/Bool | any node can publish true to halt |
| udp/50555 | JSON | spill events from the camera PoC (`{"event": "spill_halt", ...}`) |
| `/walter/run_policy` | walter_interfaces/action/RunPolicy | run GR00T with an instruction |
| `/walter/vla/chunk` | trajectory_msgs/JointTrajectory | bridge to safety filter |
| `/walter/safety/status` | walter_interfaces/msg/SafetyStatus | forwarded / rejected / slowed chunks |
| `/placement_checker/check` | walter_interfaces/srv/CheckPlacement | is a station's equipment on its marks (AprilTags) |
| `/episode_recorder/start`, `stop`, `discard` | StartEpisode / Trigger | demo recording to GR00T LeRobot format |
