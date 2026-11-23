"""Regenerate urdf/fairino_fr3_macro.xacro from fairino_description/urdf/fairino3_v6.urdf.

    python3 gen_fr3.py <frcobot_ros2>/fairino_description/urdf/fairino3_v6.urdf ../urdf/fairino_fr3_macro.xacro
"""
import re, sys, xml.etree.ElementTree as ET
src = sys.argv[1]
root = ET.parse(src).getroot()
names = {"base_link": "base_link", "shoulder_link": "link_1", "upperarm_link": "link_2", "forearm_link": "link_3",
         "wrist1_link": "link_4", "wrist2_link": "link_5", "wrist3_link": "link_6"}
jn = {f"j{i}": f"joint_{i}" for i in range(1, 7)}
out = ['<?xml version="1.0"?>',
 '<!-- FAIRINO FR3 (v6) kinematics and inertials, transcribed from fairino_description/urdf/fairino3_v6.urdf',
 '     (FAIR-INNOVATION/frcobot_ros2). Links and joints are renamed to the Walter convention',
 '     (base_link, link_1..link_6, joint_1..joint_6, tool0) so nothing above the description changes.',
 '     Meshes are NOT copied: they load from the fairino_description package, which must be in the',
 '     workspace (see the FR3 tutorial). Regenerate with gen_fr3.py if Fairino updates the URDF. -->',
 '<robot xmlns:xacro="http://ros.org/wiki/xacro">',
 '  <xacro:macro name="fairino_fr3" params="prefix">']
def f(v): return " ".join(v.split())
for link in root.findall("link"):
    n = names[link.get("name")]
    out.append(f'    <link name="${{prefix}}{n}">')
    ine = link.find("inertial")
    if ine is not None:
        o = ine.find("origin"); m = ine.find("mass").get("value"); I = ine.find("inertia").attrib
        out.append(f'      <inertial><origin xyz="{f(o.get("xyz"))}" rpy="{f(o.get("rpy"))}"/><mass value="{m}"/>'
                   f'<inertia ixx="{I["ixx"]}" ixy="{I["ixy"]}" ixz="{I["ixz"]}" iyy="{I["iyy"]}" iyz="{I["iyz"]}" izz="{I["izz"]}"/></inertial>')
    for tag in ("visual", "collision"):
        el = link.find(tag)
        if el is None: continue
        mesh = el.find("geometry/mesh").get("filename")
        mat = '<material name="fairino_white"><color rgba="0.898 0.918 0.929 1"/></material>' if tag == "visual" else ""
        out.append(f'      <{tag}><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{mesh}"/></geometry>{mat}</{tag}>')
    out.append('    </link>')
for j in root.findall("joint"):
    o = j.find("origin"); lim = j.find("limit")
    out += [f'    <joint name="${{prefix}}{jn[j.get("name")]}" type="{j.get("type")}">',
            f'      <origin xyz="{f(o.get("xyz"))}" rpy="{f(o.get("rpy"))}"/>',
            f'      <parent link="${{prefix}}{names[j.find("parent").get("link")]}"/>',
            f'      <child link="${{prefix}}{names[j.find("child").get("link")]}"/>',
            f'      <axis xyz="{f(j.find("axis").get("xyz"))}"/>',
            f'      <limit lower="{lim.get("lower")}" upper="{lim.get("upper")}" effort="{lim.get("effort")}" velocity="{lim.get("velocity")}"/>',
            '    </joint>']
out += ['    <!-- tool0 = the flange face (wrist3 frame, z out of the flange). Check it against the TCP',
        '         pose the FAIRINO WebApp reports with the tool set to 0 before trusting it. -->',
        '    <link name="${prefix}tool0"/>',
        '    <joint name="${prefix}link_6-tool0" type="fixed">',
        '      <parent link="${prefix}link_6"/>',
        '      <child link="${prefix}tool0"/>',
        '      <origin xyz="0 0 0" rpy="0 0 0"/>',
        '    </joint>',
        '  </xacro:macro>', '</robot>', '']
open(sys.argv[2], "w").write("\n".join(out))
