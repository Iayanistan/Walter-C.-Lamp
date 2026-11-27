"""Isaac Sim scene for the Walter cell, wired to ROS 2 for walter_bringup hardware:=isaac.

Run with Isaac Sim's Python (Isaac Sim 5.x, ROS 2 Jazzy sourced or the bundled Jazzy libs):

    ./python.sh walter_isaac_scene.py --usd ~/walter_sim/usd/fr3wms_ag145.usd [--headless] [--seed 3]   # Isaac Sim 6.x
    ./python.sh walter_isaac_scene.py --urdf /tmp/walter_isaac.urdf --gripper-link gripper_body              # old 5.x path

What it builds:
  * the cell URDF (bench + IRB 120 + placeholder gripper), position-driven joints
  * a stand-in 250 mL flask and a hot plate, with the flask pose randomised per run
  * a light funnel stand; --misplace-mm N shifts it N mm in a random direction (error injection
    for recording GR00T recovery demonstrations)
  * simulated AprilTags: ground-truth TF frames tag_0 (bench reference), tag_2 (hot plate) and
    tag_4 (funnel stand), named like apriltag_ros' frames, so placement_checker works in sim
  * scene and wrist RGB cameras -> /scene_camera/image_raw, /wrist_camera/image_raw (15 Hz)
  * /clock, /isaac_joint_states (all joints), /isaac_joint_commands (arm), /isaac_gripper_commands

ISAAC SIM 6.x NOTES (checked against the 6.0 migration guide, not run): isaacsim.core.api/prims/utils are
deprecated but still work; ROS2PublishJointState.targetPrim, ROS2PublishTransformTree.targetPrims and
ROS2CameraHelper.frameSkipCount are deprecated but still work (expect warnings). The URDF import commands
below are only used with --urdf; on 6.x use --usd. Isaac Sim 6 runs Python 3.12.

STATUS: written against the Isaac Sim 5.0 Python API and its ROS 2 bridge node names, NOT yet run
(no Isaac Sim in the environment it was written in). Expect to fix a name or two on first launch;
the OmniGraph wiring is the same as Isaac Sim's "ROS 2 Joint Control" tutorial.
"""

import argparse

parser = argparse.ArgumentParser()
src = parser.add_mutually_exclusive_group(required=True)
src.add_argument("--usd", help="saved robot USD (FR3 WMS + AG-105-145 imported from fr3wms_ag145_cell.urdf); "
                 "the recommended path on Isaac Sim 6.x, see the tutorial Step 1.3")
src.add_argument("--urdf", help="legacy Isaac Sim 5.x path: import a URDF with the old URDF commands")
parser.add_argument("--robot-prim", default="/World/fr3",
                    help="with --usd: where to reference the robot (its articulation root must be this prim or "
                         "--articulation-root)")
parser.add_argument("--articulation-root", default=None,
                    help="prim with ArticulationRootAPI if it is not --robot-prim (check the Property panel)")
parser.add_argument("--gripper-link", default="gripper_base_link",
                    help="prim path of the link the wrist camera hangs from, relative to --robot-prim (or to the "
                         "articulation root with --urdf); check the Stage panel, newer importers may nest links "
                         "deeper. gripper_body for the old placeholder cell")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--camera-hz", type=float, default=15.0)
parser.add_argument("--misplace-mm", type=float, default=0.0,
                    help="shift the funnel stand this far from its marks, in a random direction")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402  (must come before any other omni import)

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder, FixedCuboid  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
if args.urdf:
    enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()

BENCH_TOP = 0.9  # must match bench_z in walter_cell.urdf.xacro
rng = np.random.default_rng(args.seed)

# 250 Hz physics to match the controller rate; rendering at 30 Hz keeps cameras cheap.
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 250.0, rendering_dt=1.0 / 30.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# --- robot ------------------------------------------------------------------------------------
if args.usd:
    # Isaac Sim 6.x: import fr3wms_ag145_cell.urdf once in the GUI (Base Type: Source), set drives and
    # colliders, save as USD, then reference it here. The bench mount is baked into that URDF.
    from isaacsim.core.utils.stage import add_reference_to_stage  # deprecated in 6.x but still works
    add_reference_to_stage(usd_path=args.usd, prim_path=args.robot_prim)
    robot_path = args.articulation_root or args.robot_prim
else:
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False      # keep tcp / camera frames
    cfg.fix_base = True
    cfg.make_default_prim = False
    cfg.create_physics_scene = False    # World already made one
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    # Stiff position drives so joints track the commanded positions from ros2_control.
    cfg.default_drive_type = 1          # UrdfJointTargetType.JOINT_DRIVE_POSITION
    cfg.default_drive_strength = 1.0e5
    cfg.default_position_drive_damping = 1.0e3
    _, robot_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=args.urdf, import_config=cfg,
                                              get_articulation_root=True)
print(f"robot articulation at {robot_path}")
if args.usd:
    # The FR3 URDF has no bench (the old walter_cell xacro did), so add a bench top under the arm.
    # 1.2 x 0.9 m, extending toward +x in world, where the 180-deg-yawed arm works.
    world.scene.add(FixedCuboid(prim_path="/World/bench", name="bench",
                                position=np.array([0.35, 0.0, BENCH_TOP - 0.02]),
                                scale=np.array([1.2, 0.9, 0.04]), color=np.array([0.55, 0.45, 0.35])))

# --- task objects (stand-ins; swap for real glassware USDs when you have them) --------------
hotplate = world.scene.add(FixedCuboid(
    prim_path="/World/hotplate", name="hotplate", position=np.array([0.10, 0.0, BENCH_TOP + 0.04]),
    scale=np.array([0.18, 0.18, 0.08]), color=np.array([0.8, 0.8, 0.8])))
flask_xy = np.array([0.0, 0.22]) + rng.uniform(-0.03, 0.03, size=2)  # rack slot 1 +/- 3 cm
flask = world.scene.add(DynamicCylinder(
    prim_path="/World/flask", name="flask", radius=0.042, height=0.14, mass=0.28,
    position=np.array([flask_xy[0], flask_xy[1], BENCH_TOP + 0.07]), color=np.array([0.7, 0.85, 0.95])))
funnel_xy = np.array([-0.05, -0.25])                                  # its taped marks; match your layout
if args.misplace_mm:
    heading = rng.uniform(0.0, 2.0 * np.pi)
    funnel_xy = funnel_xy + args.misplace_mm / 1000.0 * np.array([np.cos(heading), np.sin(heading)])
    print(f"funnel stand misplaced by {args.misplace_mm:.0f} mm at {np.degrees(heading):.0f} deg")
funnel_stand = world.scene.add(DynamicCuboid(
    prim_path="/World/funnel_stand", name="funnel_stand", mass=0.3,
    position=np.array([funnel_xy[0], funnel_xy[1], BENCH_TOP + 0.06]),
    scale=np.array([0.08, 0.08, 0.12]), color=np.array([0.3, 0.5, 0.3])))

# --- cameras ---------------------------------------------------------------------------------
def make_camera(path, translate, rotate_xyz_deg):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(18.0)
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz_deg))
    return path


# Scene camera: front of the bench looking back at the work area (same view as the spill PoC webcam).
scene_cam = make_camera("/World/scene_camera", (0.95, 0.0, BENCH_TOP + 0.6), (55.0, 0.0, 90.0))
# Wrist camera: child of the gripper body so it moves with the tool. USD cameras look down -Z.
wrist_cam = make_camera(f"{args.robot_prim if args.usd else robot_path}/{args.gripper_link}/wrist_camera", (0.045, 0.0, 0.03), (0.0, 180.0, 0.0))

# --- ROS 2 graph -----------------------------------------------------------------------------
keys = og.Controller.Keys
ctx_targets = ["PubJointState", "SubArm", "SubGripper", "PubClock"]
og.Controller.edit(
    {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PubJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubArm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArmController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("SubGripper", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("GripperController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("PubJointState.inputs:topicName", "isaac_joint_states"),
            ("PubJointState.inputs:targetPrim", robot_path),
            ("SubArm.inputs:topicName", "isaac_joint_commands"),
            ("SubGripper.inputs:topicName", "isaac_gripper_commands"),
            ("ArmController.inputs:targetPrim", robot_path),
            ("GripperController.inputs:targetPrim", robot_path),
        ],
        keys.CONNECT: [
            *[("Tick.outputs:tick", f"{n}.inputs:execIn") for n in
              ("PubClock", "PubJointState", "SubArm", "ArmController", "SubGripper", "GripperController")],
            ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubJointState.inputs:timeStamp"),
            *[("Context.outputs:context", f"{n}.inputs:context") for n in ctx_targets],
            *[(f"{sub}.outputs:{field}", f"{ctl}.inputs:{field}")
              for sub, ctl in (("SubArm", "ArmController"), ("SubGripper", "GripperController"))
              for field in ("jointNames", "positionCommand", "velocityCommand", "effortCommand")],
        ],
    },
)

# Cameras on their own graph so they can tick slower than physics.
for name, cam_path, topic, frame in (("Scene", scene_cam, "scene_camera/image_raw", "scene_camera_optical_frame"),
                                     ("Wrist", wrist_cam, "wrist_camera/image_raw", "wrist_camera_optical_frame")):
    og.Controller.edit(
        {"graph_path": f"/World/{name}CameraGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", cam_path),
                ("RenderProduct.inputs:width", 640),
                ("RenderProduct.inputs:height", 480),
                ("CameraHelper.inputs:topicName", topic),
                ("CameraHelper.inputs:frameId", frame),
                ("CameraHelper.inputs:type", "rgb"),
                # Publish every Nth rendered frame to hit roughly camera_hz.
                ("CameraHelper.inputs:frameSkipCount", max(0, int(round(30.0 / args.camera_hz)) - 1)),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),
                ("Context.outputs:context", "CameraHelper.inputs:context"),
            ],
        },
    )

# --- simulated AprilTags ----------------------------------------------------------------------
# Ground truth instead of detection: an Xform named tag_<id> on each tagged object, published as TF
# relative to the bench reference tag_0. placement_checker cannot tell these from real detections.
def add_tag(parent_path, tag_id, translate):
    path = f"{parent_path}/tag_{tag_id}"
    UsdGeom.Xform.Define(stage, path).AddTranslateOp().Set(Gf.Vec3d(*translate))
    return path


ref_tag = add_tag("/World", 0, (0.25, 0.30, BENCH_TOP))
sim_tags = [add_tag("/World/hotplate", 2, (0.0, 0.0, 0.5)),       # local coords of the scaled cube: top face
            add_tag("/World/funnel_stand", 4, (0.0, 0.0, 0.5))]
og.Controller.edit(
    {"graph_path": "/World/SimTagGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            ("PubTF.inputs:parentPrim", ref_tag),
            ("PubTF.inputs:targetPrims", sim_tags),
        ],
        keys.CONNECT: [
            ("Tick.outputs:tick", "PubTF.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
            ("Context.outputs:context", "PubTF.inputs:context"),
        ],
    },
)

world.reset()
world.play()
print("Walter scene running. Now: ros2 launch walter_bringup bringup.launch.py hardware:=isaac")
while simulation_app.is_running():
    world.step(render=True)
simulation_app.close()
