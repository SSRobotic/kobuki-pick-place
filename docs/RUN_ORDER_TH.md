# ลำดับรันหุ่นจริง — เปิดสวิตช์จนกลับจุดเริ่ม

ต้องทำไว้ก่อนแล้ว (ครั้งเดียว ดู [REAL_DAY_TH.md](REAL_DAY_TH.md)): แผนที่ `my_map` · `robot\stations.yaml` จาก `save_stations.py` · ค่าสีจาก `hsv_tuner.py` · คาลิเบรตแขน · (Ubuntu: ดู [UBUNTU_TH.md](UBUNTU_TH.md) หัวข้อ E1–E4)

## 0. เปิดเครื่อง

1. วางหุ่นที่ Initial หันทางเดิม · แขนท่า REST
2. เปิดสวิตช์ Kobuki · กดปุ่มแดงแบตแขนให้ไฟติด
3. เสียบ USB: Kobuki · LiDAR · บอร์ดเซอร์โวแขน · กล้อง
4. ลูกบาศก์บนกล่อง 9 ซม. ที่ Station 1 · แผ่นจุดวางบนกล่อง 9 ซม. ที่ Station 2

## ทุกหน้าต่างเริ่มด้วย 2 บรรทัดนี้

```powershell
cd D:\ROS2_Windows_Humble
$env:FASTRTPS_DEFAULT_PROFILES_FILE = "$PWD\robot\fastdds_local.xml"
```

## 1–12. เปิดทีละหน้าต่าง ตามลำดับ (เปิดค้างไว้ทั้งหมด)

**1 LiDAR**
```powershell
pixi run python robot\lidar_ros2.py --mirror
```

**2 Kobuki**
```powershell
pixi run python robot\kobuki_ros2.py --enable-motion
```

**3 โมเดลหุ่น**
```powershell
pixi run ros2 launch robot\robot_model.launch.py arm:=none
```

**4 แขน**
```powershell
pixi run python robot\so100_driver.py
```

**5 กล้อง**
```powershell
pixi run python robot\camera_node.py
```

**6 ตัวหาวัตถุ** (ใส่ค่าสีของตัวเองแทน `[...]`)
```powershell
pixi run python robot\cube_detector.py --ros-args -p surface_height:=0.09 -p cube_hsv_low:=[...] -p cube_hsv_high:=[...] -p marker_hsv_low:=[...] -p marker_hsv_high:=[...]
```

**7 Nav2**
```powershell
pixi run ros2 launch robot\nav2.launch.py map:=$HOME\maps\my_map.yaml
```

**8 RViz**
```powershell
pixi run rviz2 -d robot\mission.rviz
```

**12 MoveIt**
```powershell
pixi run ros2 launch robot\moveit.launch.py rviz:=false
```

**10 เสียงหุ่น**
```powershell
pixi run python robot\announcer.py
```

**9 ภารกิจ** (เปิดสุดท้าย)
```powershell
pixi run python robot\mission_nav2.py --moveit
```

จบที่ `Mission finished: succeeded` · หุ่นกลับ Initial

## หยุดฉุกเฉิน

`Ctrl+C` หน้าต่าง 9 แล้วหน้าต่าง 2
