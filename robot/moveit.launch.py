"""MoveIt 2 for the SO-100: move_group + so100_trajectory_bridge.py (+ RViz with the MotionPlanning panel).
Start it after robot_model.launch.py (arm:=sim for practice, arm:=none with the real so100_driver.py):
    pixi run ros2 launch robot\\moveit.launch.py             (Windows)
    ros2 launch ./moveit.launch.py                           (Ubuntu)
    ... rviz:=false    no RViz window (arm_control.py --moveit and the mission do not need it)
Config files are in moveit/. MoveIt plans around the Kobuki, decks, LiDAR and camera, so a MEASURE change in
kobuki_x2.urdf is picked up here too.
"""
import copy
import os
import pathlib
import sys
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "moveit")


def load(name):
    with open(os.path.join(CONFIG, name), encoding="utf-8") as f:
        return yaml.safe_load(f)


def robot_description():
    """kobuki_x2.urdf with collision shapes: every visual is also a collision shape, except the textured Kobuki
    body (a plain cylinder is enough) and the wheels (inside the body)."""
    with open(os.path.join(HERE, "kobuki_x2.urdf"), encoding="utf-8") as f:
        text = f.read().replace("MESH_URI", pathlib.Path(HERE, "meshes").as_uri())
    root = ET.fromstring(text)
    for link in root.findall("link"):
        name = link.get("name")
        if name.startswith("wheel_"):
            continue
        if name == "base_link":
            collision = ET.SubElement(link, "collision")
            ET.SubElement(collision, "origin", xyz="0 0 0.058", rpy="0 0 0")
            ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder", radius="0.176", length="0.095")
            continue
        for visual in link.findall("visual"):
            collision = ET.SubElement(link, "collision")
            for part in visual:
                if part.tag in ("origin", "geometry"):
                    collision.append(copy.deepcopy(part))
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    with open(os.path.join(CONFIG, "so100.srdf"), encoding="utf-8") as f:
        srdf = f.read()
    model = {"robot_description": robot_description(),
             "robot_description_semantic": srdf,
             "robot_description_kinematics": load("kinematics.yaml"),
             "robot_description_planning": load("joint_limits.yaml")}
    planning = {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl", "ompl": load("ompl_planning.yaml")}
    scene = {"publish_planning_scene": True, "publish_geometry_updates": True, "publish_state_updates": True,
             "publish_transforms_updates": True, "publish_robot_description_semantic": True}
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="open RViz with the MotionPlanning panel"),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[model, planning, load("moveit_controllers.yaml"), scene]),
        ExecuteProcess(cmd=[sys.executable, "-u", os.path.join(HERE, "so100_trajectory_bridge.py")], output="screen"),
        Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", os.path.join(CONFIG, "moveit.rviz")],
             parameters=[model, planning], condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
