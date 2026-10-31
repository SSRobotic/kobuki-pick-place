# รันหุ่นจริงบน Ubuntu (VM) — ตั้งแต่ git clone จนหุ่นกลับจุดเริ่ม

Ubuntu **22.04** + ROS 2 **Humble** (apt ปกติ ไม่ใช้ pixi) · VM ที่เป็น Ubuntu รุ่นอื่นใช้ Humble จาก apt ไม่ได้
⚠ ทดสอบกับหุ่นจริงแล้วบน Windows · บน Ubuntu ยังไม่เคยต่อหุ่นจริง (โค้ดชุดเดียวกัน)

## A. ตั้งค่า VM (ครั้งเดียว)

- CPU ≥ 4 core · RAM ≥ 8 GB · ดิสก์ว่าง ≥ 15 GB
- **VMware:** VM → Settings → Display → ติ๊ก *Accelerate 3D graphics* · USB Controller → USB compatibility *USB 2.0*
- **VirtualBox:** Settings → Display → *Enable 3D Acceleration* · USB → *USB 2.0 (EHCI)*
- ต้องเป็น **USB 2.0**: กล้อง Xtion มักไม่ส่งภาพเมื่อผ่าน USB 3 ของ VM (อุปกรณ์อื่นใช้ 2.0 ได้หมด)

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
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit ros-humble-teleop-twist-keyboard ros-humble-openni2-camera python3-serial python3-opencv python3-numpy python3-yaml alsa-utils
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

กล้อง (Xtion ไม่ใช่ `ttyUSB`):
```bash
lsusb | grep -i -e 1d27 -e primesense
```
✅ มีบรรทัด `1d27:0601` (หรือ `1d27:0600`) · ไม่มี = ยังไม่ได้ Connect เข้า VM หรือ USB ไม่ใช่ 2.0 (A)

## D. ทุกหน้าต่าง terminal เริ่มด้วย 2 บรรทัดนี้

```bash
cd ~/kobuki-pick-place/robot
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_local.xml
```

## E. ทำครั้งเดียวต่อสนาม: แผนที่ + ค่าสี + ตำแหน่ง Station

**วางหุ่นที่ Initial Location หันไปทาง Station 1 ก่อนเริ่ม** (จุดเริ่มทำแผนที่ = จุด (0,0) ของแผนที่ = จุดที่ Nav2 คิดว่าหุ่นยืนอยู่ตอนเปิด)

เปิดหน้าต่าง 1, 2 ของ F และโมเดลหุ่นแบบ **ไม่มีไดรเวอร์แขน** (ไม่งั้นแขนใน RViz เป็นสีขาวกองอยู่ที่พื้น):
```bash
ros2 launch ./robot_model.launch.py arm:=rest
```
แล้วเปิดอีก 3 หน้าต่าง:
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
**ปิด 3 หน้าต่างนี้ทุกครั้งก่อนเปิด Nav2** (ถ้า slam_toolbox ยังเปิดค้าง มันกับ AMCL จะแย่งกันขยับแผนที่ หุ่นเดินมั่ว) · หาค่าสี: `python3 hsv_tuner.py` · ตำแหน่ง Station: `nano mission_nav2.py` แก้ `STATIONS` ตาม [REAL_DAY_TH.md](REAL_DAY_TH.md) ขั้น 8

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

**3 โมเดลหุ่น** (`arm:=none` = แขนจริงจากหน้าต่าง 4 เป็นคนส่งท่าแขน · ถ้าไม่ได้ต่อแขน ใช้ `arm:=rest`)
```bash
ros2 launch ./robot_model.launch.py arm:=none
```

**4 แขน**
```bash
python3 so100_driver.py
```

**5 กล้อง** Xtion (บน Ubuntu ใช้ไดรเวอร์ของ ROS ตัวนี้ · `camera_node.py` แบบ `device:=0` ใช้กับ Xtion ไม่ได้ เพราะ Xtion ไม่ใช่เว็บแคม)
```bash
ros2 run openni2_camera openni2_camera_driver --ros-args -r rgb/image_raw:=/camera/color/image_raw -r rgb/camera_info:=/camera/color/camera_info -p rgb_frame_id:=camera_color_optical_frame -p depth_registration:=false -p use_device_time:=false
```
เช็คในหน้าต่างใหม่ (D ก่อน):
```bash
ros2 topic hz /camera/color/image_raw
```
✅ ประมาณ `30` · ภาพจะเริ่มส่งเมื่อมีคนดู (คำสั่งนี้ / ตัวหาวัตถุ) · `No devices connected` = Xtion ยังไม่เข้า VM (C)

ไม่มี Xtion ใช้เว็บแคมแทน:
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

**บอก Nav2 ว่าหุ่นอยู่ไหน (ทุกครั้ง ก่อนสั่งเดิน):**
1. ใน RViz กด **2D Pose Estimate** → คลิกตรงที่หุ่นยืนจริงบนแผนที่ → ลากไปทางที่หุ่นหัน → ปล่อย
2. ดูจุดแดง (LiDAR) ต้องทับเส้นดำ (กำแพง) · ไม่ทับ = ทำข้อ 1 ใหม่
3. หน้าต่างใหม่ (D ก่อน):
```bash
python3 nav_check.py
```
✅ `ผ่านหมด` · ❌ ทำตามที่มันบอก แล้วรันใหม่ · ผ่านแล้วจะลอง **2D Goal Pose** ใน RViz ก็ได้

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
| แขนใน RViz เป็นสีขาว กองอยู่ที่พื้น (`No transform`) | ไม่มีใครส่งท่าแขน: เปิดหน้าต่าง 4 (`so100_driver.py`) ให้ขึ้น `SO-100 ready` · ถ้าไม่ได้ใช้แขน ปิดหน้าต่าง 3 แล้วเปิดใหม่เป็น `arm:=rest` |
| สั่ง 2D Goal Pose แล้วหุ่นเดินมั่ว / หมุนวน / ถอยหลัง | `python3 nav_check.py` แล้วแก้ ❌ · ที่เจอบ่อย: ไม่ได้กด 2D Pose Estimate (Nav2 คิดว่าหุ่นยังอยู่ที่ (0,0) ของแผนที่) · slam_toolbox ยังเปิดค้าง · VM ช้า (`/scan` < 2 Hz: ปิด RViz ตัวอื่น เพิ่ม CPU) · จุดแดงทับกำแพงแค่ตอนหยุด พอหุ่นเดินแล้วเลื่อน = TF LiDAR ผิด: `python3 lidar_calib.py` |
| กล้อง: `No devices connected` / `ros2 topic hz` เงียบ | Xtion ยังไม่ Connect เข้า VM · USB ของ VM ต้องเป็น 2.0 (A) · ถอด/Connect ใหม่หลังลง `ros-humble-openni2-camera` · หาไม่เจอจริงๆ ใช้เว็บแคม (หน้าต่าง 5) |
| กล้อง: `Package 'openni2_camera' not found` | ลงแพ็กเกจ B ไม่ครบ: รันบรรทัด `sudo apt install -y ros-humble-navigation2 ...` อีกครั้ง |
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` (หรือเปิด terminal ใหม่หลังทำ B) |
| `No unique Kobuki adapter` / `servo board not found` | ตัวนั้นยังไม่ได้ Connect เข้า VM (C) · `ls /dev/ttyUSB* /dev/ttyACM*` ต้องเห็น |
| `Permission denied: '/dev/ttyUSB0'` | ยังไม่ได้ `usermod -aG dialout` หรือยังไม่ได้ logout/login หลังจากนั้น |
| `package 'moveit_ros_move_group' not found` / `nav2_...` | ลงแพ็กเกจใน B ไม่ครบ: รันบรรทัด `sudo apt install -y ros-humble-navigation2 ...` อีกครั้ง |
| RViz ดำ / ช้ามาก | เปิด 3D acceleration (A) · ยังไม่ได้: `export LIBGL_ALWAYS_SOFTWARE=1` ก่อนเปิด RViz |
| แขน `servos not answering` | แบตแขนดับ: กดปุ่มแดง · แล้ว Connect บอร์ดแขนเข้า VM ใหม่ |
