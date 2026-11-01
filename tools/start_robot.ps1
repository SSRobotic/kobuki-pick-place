# Open every program for the real run in its own PowerShell window (titles 0-8), in the right order.
#   powershell -ExecutionPolicy Bypass -File tools\start_robot.ps1                      (map $HOME\maps\my_map.yaml)
#   powershell -ExecutionPolicy Bypass -File tools\start_robot.ps1 -Map $HOME\maps\my_map2.yaml
# Colours come from $HOME\hsv.txt (hsv_tuner.py's -p line) if it exists. Close a window = stop that program.
# Afterwards: RViz 2D Pose Estimate -> robot\nav_check.py -> (new map or moved boxes: robot\save_stations.py)
# -> robot\mission_nav2.py
param([string]$Map = "$HOME\maps\my_map.yaml")

$root = Split-Path -Parent $PSScriptRoot
$profile_xml = Join-Path $root "robot\fastdds_local.xml"
$hsv = "-p cube_hsv_low:=[0,176,111] -p cube_hsv_high:=[13,255,255] -p marker_hsv_low:=[126,41,60] -p marker_hsv_high:=[146,255,255]"
if (Test-Path "$HOME\hsv.txt") { $hsv = (Get-Content "$HOME\hsv.txt" -Raw).Trim() }
if (-not (Test-Path $Map)) { Write-Host "map not found: $Map" -ForegroundColor Red; exit 1 }

function Open-Window($title, $command, $wait) {
    $c = "`$host.UI.RawUI.WindowTitle = '$title'; Set-Location '$root'; " +
         "`$env:FASTRTPS_DEFAULT_PROFILES_FILE = '$profile_xml'; $command"
    Start-Process powershell -ArgumentList "-NoExit -ExecutionPolicy Bypass -Command `"$c`""
    Write-Host "opened $title" -ForegroundColor Green
    Start-Sleep -Seconds $wait
}

Open-Window "0 keep awake" "& '$root\tools\keep_awake.ps1'" 1
Open-Window "1 LiDAR" "pixi run python robot\lidar_ros2.py --mirror" 3
Open-Window "2 Kobuki" "pixi run python robot\kobuki_ros2.py --enable-motion" 3
Open-Window "3 robot model" "pixi run ros2 launch robot\robot_model.launch.py arm:=none" 3
Open-Window "4 arm" "pixi run python robot\so100_driver.py" 3
Open-Window "5 camera" "pixi run python robot\camera_node.py" 3
Open-Window "6 detector" "pixi run python robot\cube_detector.py --ros-args -p surface_height:=0.09 $hsv" 3
Open-Window "7 Nav2" "pixi run ros2 launch robot\nav2.launch.py map:=$Map" 12
Open-Window "8 RViz" "pixi run rviz2 -d robot\mission.rviz" 1

Write-Host ""
Write-Host "All open. Next (in a new PowerShell, 2 setup lines first):" -ForegroundColor Yellow
Write-Host "  RViz: 2D Pose Estimate on the real robot -> pixi run python robot\nav_check.py"
Write-Host "  new map / boxes moved: pixi run python robot\save_stations.py"
Write-Host "  run: pixi run python robot\mission_nav2.py"
