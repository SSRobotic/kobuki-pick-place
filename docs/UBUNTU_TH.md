# รันหุ่นจริงบน Ubuntu (VM) — ตั้งแต่ git clone จนหุ่นกลับจุดเริ่ม

Ubuntu **22.04** + ROS 2 **Humble** (apt ปกติ ไม่ใช้ pixi) · VM ที่เป็น Ubuntu รุ่นอื่นใช้ Humble จาก apt ไม่ได้
⚠ ทดสอบกับหุ่นจริงแล้วบน Windows · บน Ubuntu ยังไม่เคยต่อหุ่นจริง (โค้ดชุดเดียวกัน)

## A. ตั้งค่า VM (ครั้งเดียว)

- CPU ≥ 4 core · RAM ≥ 8 GB · ดิสก์ว่าง ≥ 15 GB
- **VMware:** VM → Settings → Display → ติ๊ก *Accelerate 3D graphics* · USB Controller → *USB 3.1*
- **VirtualBox:** Settings → Display → *Enable 3D Acceleration* · USB → *USB 3.0 (xHCI)*

## B. ติดตั้ง (ครั้งเดียว)

ถ้า VM มี ROS 2 Humble อยู่แล้ว (มี `/opt/ros/humble`) ข้ามไปบรรทัด `sudo apt install -y ros-humble-navigation2 ...` เลย

```bash
sudo apt update && sudo apt install -y curl git software-properties-common
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-humble-desktop
```

แพ็กเกจที่โปรเจกต์ใช้:
```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit ros-humble-teleop-twist-keyboard python3-serial python3-opencv python3-numpy python3-yaml alsa-utils
```

ตั้งค่าเครื่อง + โหลดโค้ด:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo usermod -aG dialout $USER
cd ~
git clone https://github.com/SSRobotic/kobuki-pick-place.git
```
แล้ว **logout/login (หรือรีสตาร์ท VM) 1 ครั้ง** ให้ `dialout` มีผล (ใช้พอร์ต USB ได้)

อัปเดตโค้ดรอบหลัง: `cd ~/kobuki-pick-place && git pull`

## C. ต่อ USB เข้า VM (ทุกครั้งที่เสียบ)

เสียบ Kobuki, LiDAR, บอร์ดแขน, กล้อง ที่เครื่องเพื่อน แล้วโยนเข้า VM:
- **VMware:** เมนู VM → Removable Devices → เลือกทีละตัว → *Connect (Disconnect from Host)*
- **VirtualBox:** เมนู Devices → USB → ติ๊กทีละตัว

| อุปกรณ์ | ชื่อในเมนู |
|---|---|
| Kobuki | `FTDI` / `Future Technology Devices` / `iKobuki` |
| LiDAR | `Silicon Labs CP210x` |
| แขน | `QinHeng` / `USB-Enhanced-SERIAL CH343` |
| กล้อง | `PrimeSense` / `ASUS Xtion` (หรือเว็บแคม) |

เช็ค:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
✅ `ttyUSB0` `ttyUSB1` (Kobuki, LiDAR) + `ttyACM0` (แขน) · โปรแกรมหาพอร์ตเองจากรหัส USB

## D. ทุกหน้าต่าง terminal เริ่มด้วย 2 บรรทัดนี้

```bash
cd ~/kobuki-pick-place/robot
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_local.xml
```

## E. ทำครั้งเดียวต่อสนาม: แผนที่ + ค่าสี + ตำแหน่ง Station

เปิดหน้าต่าง 1–3 ของ F ก่อน แล้วเปิดอีก 3 หน้าต่าง:
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file slam_params.yaml
```
```bash
rviz2 -d mapping.rviz
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35
```
ขับรอบห้องจนกำแพงคม แล้วเซฟ:
```bash
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p save_map_timeout:=10.0
```
ปิด 3 หน้าต่างนี้ · หาค่าสี: `python3 hsv_tuner.py` · ตำแหน่ง Station: `nano mission_nav2.py` แก้ `STATIONS` ตาม [REAL_DAY_TH.md](REAL_DAY_TH.md) ขั้น 8

## F. เปิดสวิตช์ → หุ่นกลับจุดเริ่ม

0. หุ่นที่ Initial หันทางเดิม · แขนท่า REST · สวิตช์ Kobuki · ปุ่มแดงแบตแขน · ต่อ USB เข้า VM (C) · ลูกบาศก์/จุดวางบนกล่อง 9 ซม.

เปิดทีละหน้าต่าง (แต่ละหน้าต่างเริ่มด้วย D) เปิดค้างไว้ทั้งหมด:

**1 LiDAR**
```bash
python3 lidar_ros2.py --mirror
```

**2 Kobuki**
```bash
python3 kobuki_ros2.py --enable-motion
```

**3 โมเดลหุ่น**
```bash
ros2 launch ./robot_model.launch.py arm:=none
```

**4 แขน**
```bash
python3 so100_driver.py
```

**5 กล้อง** (Xtion บน Linux ยังไม่เคยลอง: ถ้าไม่ขึ้นใช้เว็บแคมแบบนี้)
```bash
python3 camera_node.py --ros-args -p backend:=opencv -p device:=0 -p hfov_deg:=60
```

**6 ตัวหาวัตถุ** (ค่าสีของตัวเองแทน `[...]`)
```bash
python3 cube_detector.py --ros-args -p surface_height:=0.09 -p cube_hsv_low:=[...] -p cube_hsv_high:=[...] -p marker_hsv_low:=[...] -p marker_hsv_high:=[...]
```

**7 Nav2**
```bash
ros2 launch ./nav2.launch.py map:=$HOME/maps/my_map.yaml
```

**8 RViz**
```bash
rviz2 -d mission.rviz
```

**12 MoveIt**
```bash
ros2 launch ./moveit.launch.py rviz:=false
```

**10 เสียงหุ่น**
```bash
python3 announcer.py
```

**9 ภารกิจ** (เปิดสุดท้าย)
```bash
python3 mission_nav2.py --moveit
```

จบที่ `Mission finished: succeeded` · หุ่นกลับ Initial · หยุดฉุกเฉิน: `Ctrl+C` หน้าต่าง 9 แล้วหน้าต่าง 2

## ถ้าติด

| อาการ | แก้ |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` (หรือเปิด terminal ใหม่หลังทำ B) |
| `No unique Kobuki adapter` / `servo board not found` | ตัวนั้นยังไม่ได้ Connect เข้า VM (C) · `ls /dev/ttyUSB* /dev/ttyACM*` ต้องเห็น |
| `Permission denied: '/dev/ttyUSB0'` | ยังไม่ได้ `usermod -aG dialout` หรือยังไม่ได้ logout/login หลังจากนั้น |
| `package 'moveit_ros_move_group' not found` / `nav2_...` | ลงแพ็กเกจใน B ไม่ครบ: รันบรรทัด `sudo apt install -y ros-humble-navigation2 ...` อีกครั้ง |
| RViz ดำ / ช้ามาก | เปิด 3D acceleration (A) · ยังไม่ได้: `export LIBGL_ALWAYS_SOFTWARE=1` ก่อนเปิด RViz |
| แขน `servos not answering` | แบตแขนดับ: กดปุ่มแดง · แล้ว Connect บอร์ดแขนเข้า VM ใหม่ |
