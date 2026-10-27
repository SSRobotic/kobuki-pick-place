"""Publish kobuki_x2.urdf (robot model + base_link->laser TF + SO-100 arm) for RViz.
Replaces the separate static_transform_publisher window.
    pixi run ros2 launch robot\\robot_model.launch.py            (Windows)
    ros2 launch ./robot_model.launch.py                          (Ubuntu)
    ... arm:=none   when a real SO-100 driver publishes /joint_states
"""
import os
import pathlib
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "kobuki_x2.urdf"), encoding="utf-8") as f:
        description = f.read()
    # CAD meshes live in meshes/ next to this file; RViz needs an absolute file:// URI
    description = description.replace("MESH_URI", pathlib.Path(here, "meshes").as_uri())
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="rest",
                              description="rest = publish the folded SO-100 pose; none = a real arm driver publishes /joint_states"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        ExecuteProcess(cmd=[sys.executable, os.path.join(here, "arm_rest_publisher.py")], output="screen",
                       condition=LaunchConfigurationEquals("arm", "rest")),
    ])
