#!/usr/bin/env bash
# Opens every terminal needed for mapping (Ubuntu + ROS 2).
#   cd ~/kobuki-pick-place/robot && ./start_mapping.sh
# Stop a node with Ctrl+C in its own terminal.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[ -f "$ROS_SETUP" ] || ROS_SETUP="$(ls /opt/ros/*/setup.bash | head -1)"
echo "Using $ROS_SETUP"

open_node() {  # $1 = title, $2 = command
  gnome-terminal --title="$1" -- bash -c "source '$ROS_SETUP'; cd '$DIR'; echo '== $1 =='; $2; echo; echo '[stopped] press Enter to close'; read"
}

open_node "1 LiDAR"  "python3 lidar_ros2.py --mirror"
open_node "2 Kobuki" "python3 kobuki_ros2.py --enable-motion"
sleep 4
open_node "3 Robot model" "ros2 launch ./robot_model.launch.py"
sleep 2
open_node "4 SLAM" "ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file slam_params.yaml"
open_node "5 RViz" "rviz2 -d mapping.rviz"
open_node "6 TELEOP (click here)" "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35"

echo "Started. When the map is complete:"
echo "  mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p save_map_timeout:=10.0"
