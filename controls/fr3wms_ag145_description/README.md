# fr3wms_ag145_description

FAIRINO FR3 WMS + DH-Robotics AG-105-145 gripper, by Iayan.

| File | Use |
| --- | --- |
| `urdf/fr3wms_ag145.urdf` | ROS (RViz, MoveIt): `package://fr3wms_ag145_description/...` mesh paths |
| `urdf/fr3wms_ag145_relative_paths.urdf` | Same robot, `../meshes/...` paths |
| `urdf/fr3wms_ag145_cell.urdf` | Relative paths plus a `world` joint mounting the arm on the bench (z 0.90 m, yaw 180 deg). Import this into Isaac Sim with Base Type: Source. |

Joints `j1..j6`; gripper `finger_joint` (0 = open 145 mm, 0.932 rad = closed) with 13 mimic joints; TCP link `gripper_tcp`.

**Meshes are not in the repo yet.** Add `meshes/fr3wms/{visual,collision}` and `meshes/ag145/{visual,collision}` next to `urdf/`.
