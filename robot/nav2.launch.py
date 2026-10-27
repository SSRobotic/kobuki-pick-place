"""Nav2 for Kobuki + X2: map_server + AMCL (localization) and planner/controller/behaviors/bt_navigator.
Needs the robot running first (fake_robot.py or the real drivers) and robot_model.launch.py.
    pixi run ros2 launch robot\\nav2.launch.py                       (Windows)
    ros2 launch nav2.launch.py                                      (Ubuntu, inside robot/)
    ... nav2.launch.py map:=C:\\path\\to\\my_map.yaml                 (use another map)
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCALIZATION = ["map_server", "amcl"]
NAVIGATION = ["planner_server", "controller_server", "behavior_server", "bt_navigator"]


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    params = os.path.join(here, "nav2_params.yaml")
    map_yaml = LaunchConfiguration("map")

    def server(package, executable, name, extra=None):
        return Node(package=package, executable=executable, name=name, output="screen",
                    parameters=[params] + (extra or []))

    def manager(name, nodes):
        if os.name == "nt":  # lifecycle_manager crashes on Windows (RoboStack): use lifecycle_up.py
            return ExecuteProcess(cmd=[sys.executable, "-u", os.path.join(here, "lifecycle_up.py")] + nodes,
                                  name=name, output="screen")
        return Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name=name,
                    output="screen", parameters=[{"autostart": True, "node_names": nodes}])

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(here, "maps", "practice_map.yaml"),
                              description="map .yaml saved by map_saver_cli"),
        server("nav2_map_server", "map_server", "map_server", [{"yaml_filename": map_yaml}]),
        server("nav2_amcl", "amcl", "amcl"),
        manager("lifecycle_manager_localization", LOCALIZATION),
        server("nav2_planner", "planner_server", "planner_server"),
        server("nav2_controller", "controller_server", "controller_server"),
        server("nav2_behaviors", "behavior_server", "behavior_server"),
        server("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        manager("lifecycle_manager_navigation", NAVIGATION),
    ])
