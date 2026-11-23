# Isaac Sim for the Walter cell

Isaac Sim plays the part of the robot. ros2_control talks to it through
`topic_based_ros2_control`, so MoveIt, the safety filter, the GR00T bridge and the recorder run
exactly as they will on the real IRB 120.

```
ros2_control (TopicBasedSystem) --/isaac_joint_commands-->   Isaac Sim articulation (arm)
                                --/isaac_gripper_commands--> Isaac Sim articulation (gripper)
                                <--/isaac_joint_states-----  Isaac Sim
GR00T bridge / recorder         <--/scene_camera, /wrist_camera image_raw-- Isaac Sim cameras
everything (use_sim_time)       <--/clock------------------  Isaac Sim
```

## Versions

Isaac Sim 5.x on Ubuntu 24.04 with ROS 2 Jazzy (Isaac Sim 5 supports Humble and Jazzy). An RTX
GPU is required. A 24 GB card is comfortable for the sim; GR00T fine-tuning wants a separate
40 GB+ GPU or a cloud node.

## Run

```bash
# 1. Expand the cell for Isaac (absolute mesh paths, isaac ros2_control block)
source ~/walter_ws/install/setup.bash
xacro $(ros2 pkg prefix walter_description)/share/walter_description/urdf/walter_cell.urdf.xacro hardware:=isaac \
  | sed "s#package://walter_description#$(ros2 pkg prefix walter_description)/share/walter_description#g" \
  > /tmp/walter_isaac.urdf

# 2. Start the scene (Isaac Sim's python.sh; source ROS 2 Jazzy first or use Isaac's bundled libs)
~/isaacsim/python.sh walter_isaac_scene.py --urdf /tmp/walter_isaac.urdf --seed 1

# 3. Bring up the controls stack against the sim
ros2 launch walter_bringup bringup.launch.py hardware:=isaac
```

**FAIRINO FR3** (the final arm): add `robot:=fr3` to the xacro and launch commands, and resolve the
FAIRINO meshes too (needs `fairino_description` from `walter_fr3.repos` built in the workspace):

```bash
xacro $(ros2 pkg prefix walter_description)/share/walter_description/urdf/walter_cell.urdf.xacro robot:=fr3 hardware:=isaac \
  | sed -e "s#package://walter_description#$(ros2 pkg prefix walter_description)/share/walter_description#g" \
        -e "s#package://fairino_description#$(ros2 pkg prefix fairino_description)/share/fairino_description#g" \
  > /tmp/walter_isaac_fr3.urdf
~/isaacsim/python.sh walter_isaac_scene.py --urdf /tmp/walter_isaac_fr3.urdf --seed 1
ros2 launch walter_bringup bringup.launch.py robot:=fr3 hardware:=isaac
```

`walter_isaac_scene.py` was written against the Isaac Sim 5.0 API but has not been run yet.
On the first launch, check the console for renamed node types or import-config fields; everything
else in the stack has been run against mock hardware.

## Making training data in sim

1. Start the scene with a new `--seed` per batch, so the flask position changes.
2. `ros2 run walter_control episode_recorder --ros-args -p dataset_dir:=~/walter_data/flask_to_hotplate -p use_sim_time:=true`
3. Drive the skill: a scripted task file run by `task_runner` (cheap, many episodes) or teleop
   through MoveIt Servo with a SpaceMouse (slower, more natural motion). Bracket each demo with
   `~/start` and `~/stop` on the recorder.
4. Mix in real-cell demos later. Sim-only fine-tunes usually need some real data to close the
   visual gap; the scene camera matches the spill-detection webcam view to make that easier.

Domain randomisation worth adding next: flask position (already done), lighting, bench texture,
camera pose jitter of about 1 cm and 2°, and flask fill level.

## Error injection and simulated tags

- `--misplace-mm 20` starts the funnel stand 20 mm off its marks in a random direction (set by `--seed`).
  Use it to record and test GR00T recovery demonstrations.
- The scene publishes ground-truth TF frames `tag_0` (bench reference), `tag_2` (hot plate) and
  `tag_4` (funnel stand), so `ros2 run walter_skills placement_checker --ros-args -p use_sim_time:=true`
  works against the sim without a camera. Teach their expected poses once with
  `ros2 run walter_skills teach --station funnel --tag` while the scene runs with no misplacement.
