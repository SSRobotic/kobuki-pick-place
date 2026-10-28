# Opens every window needed for mapping. Run from PowerShell:
#   cd <project folder> ; .\robot\start_mapping.ps1
# Close a node with Ctrl+C in its own window.
$root = Split-Path -Parent $PSScriptRoot   # project folder (one level above robot\)


function Open-Node($title, $cmd) {
    $full = "`$host.UI.RawUI.WindowTitle = '$title'; cd $root; $cmd"
    Start-Process powershell -ArgumentList @('-NoExit', '-Command', $full)
}

Open-Node "1 LiDAR"  "pixi run python robot\lidar_ros2.py --mirror"
Open-Node "2 Kobuki" "pixi run python robot\kobuki_ros2.py --enable-motion"
Start-Sleep -Seconds 4
# Robot model + base_link->laser TF (LiDAR height/yaw live in robot\kobuki_x2.urdf, joint "laser_joint")
Open-Node "3 Robot model" "pixi run ros2 launch robot\robot_model.launch.py"
Start-Sleep -Seconds 2
Open-Node "4 SLAM" "pixi run ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file robot\slam_params.yaml"

if (Test-Path "$root\robot\mapping.rviz") {
    Open-Node "5 RViz" "pixi run rviz2 -d robot\mapping.rviz"
} else {
    Open-Node "5 RViz" "pixi run rviz2"
}
Open-Node "6 TELEOP (click here, English keyboard)" "pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35"

Write-Host "Started. When the map looks complete run:"
Write-Host "  pixi run ros2 run nav2_map_server map_saver_cli -f robot\maps\my_map --ros-args -p save_map_timeout:=10.0"
