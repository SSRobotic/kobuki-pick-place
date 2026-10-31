"""Foxglove bridge: watch and drive the robot from a browser on another computer or a phone.
Run it on the laptop on the robot, then open Foxglove and connect to ws://<laptop IP>:8765
    pixi run ros2 launch robot\\foxglove.launch.py            (Windows)
    ros2 launch ./foxglove.launch.py                          (Ubuntu)
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The stock bridge only serves package:// meshes; kobuki_x2.urdf points at file://.../meshes/
# (see robot_model.launch.py), so allow those too or the 3D panel shows no robot body.
MESHES = r"(?:[-\w%.]+/)*[-\w%.]+\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
ASSET_ALLOWLIST = ["^package://" + MESHES, "^file:///.*/meshes/" + MESHES]


def generate_launch_description():
    env = {}
    if os.name == "nt":  # RoboStack puts foxglove.dll + foxglove_cpp_shared.dll in Library\lib, not on PATH
        lib = os.path.join(get_package_prefix("foxglove_bridge"), "lib")
        env["PATH"] = lib + os.pathsep + os.environ.get("PATH", "")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8765"),
        Node(package="foxglove_bridge", executable="foxglove_bridge", output="screen", additional_env=env,
             parameters=[{"port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                          "address": "0.0.0.0",
                          "asset_uri_allowlist": ASSET_ALLOWLIST,
                          "capabilities": ["clientPublish", "parameters", "parametersSubscribe",
                                           "services", "connectionGraph", "assets"]}]),
    ])
