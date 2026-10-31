# คู่มือหุ่นจริง — Ubuntu (ไม่ใช้ pixi) · ฉบับเดียวจบ

Kobuki + LiDAR X2 + แขน SO-100 + กล้อง Xtion: ออกจาก Initial → หยิบลูกบาศก์ที่ Station 1 → วางที่ Station 2 → กลับ Initial
Ubuntu **22.04** + ROS 2 **Humble** จาก apt (เครื่องจริงหรือ VM) · อ่านจากบนลงล่าง · **A–B ทำครั้งแรกครั้งเดียว · C ทำครั้งเดียวต่อสนาม · D ทำทุกรอบที่รันหุ่น** · ใช้ Windows ให้ดู [GUIDE_WINDOWS_PIXI.md](GUIDE_WINDOWS_PIXI.md)

---

## กติกา (อ่านก่อน)

**ทุกหน้าต่าง terminal ใหม่ พิมพ์ 2 บรรทัดนี้ก่อนเสมอ** (ไม่ใส่บรรทัดที่ 2 = MoveIt ไม่ได้ท่าแขน แขนไม่ขยับ):
```bash
cd ~/kobuki-pick-place/robot
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_local.xml
```
- 1 คำสั่งที่รันค้าง = 1 หน้าต่าง เปิดค้างไว้ หยุดด้วย `Ctrl+C`
- คำสั่งสั้นที่รันแล้วจบเอง ใช้หน้าต่าง **"สำรอง"** อันเดียวทั้งวัน

---

## A. ติดตั้ง (ครั้งแรกครั้งเดียว)

**ถ้าเป็น VM** — ตั้งก่อนเปิด VM: CPU ≥ 4 · RAM ≥ 8 GB · เปิด 3D acceleration · USB เป็น **USB 2.0** (Xtion ไม่ส่งภาพผ่าน USB 3 ของ VM)

1. ROS 2 Humble (มี `/opt/ros/humble` แล้ว = ข้ามไปข้อ 2):
   ```bash
   sudo apt update && sudo apt install -y curl git software-properties-common
   sudo add-apt-repository -y universe
   export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
   curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
   sudo dpkg -i /tmp/ros2-apt-source.deb
   sudo apt update && sudo apt install -y ros-humble-desktop
   ```
2. แพ็กเกจที่โปรเจกต์ใช้:
   ```bash
   sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit ros-humble-teleop-twist-keyboard ros-humble-openni2-camera python3-serial python3-opencv python3-numpy python3-yaml python3-tk alsa-utils
   ```
3. ตั้งเครื่อง + โหลดโค้ด:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   sudo usermod -aG dialout $USER
   cd ~ && git clone https://github.com/SSRobotic/kobuki-pick-place.git
   ```
   แล้ว **logout/login (หรือรีสตาร์ท) 1 ครั้ง**

อัปเดตโค้ดรอบหลัง: `cd ~/kobuki-pick-place && git pull`

---

## B. ประกอบ + เสียบสาย + ตั้งแขน (ครั้งแรกครั้งเดียว / ถอดประกอบใหม่)

### B1. ประกอบ

| ชิ้น | ตรงไหน |
|---|---|
| หน้า / หลัง | หน้า = กันชน · หลัง = แผงพอร์ต USB |
| แขน SO-100 | แผ่นขาวกลางชั้นกลาง **ฐานหันออกหน้า** |
| กล้อง Xtion | ชั้นล่าง ขอบหน้า **หันออกหน้า** (ใต้ปากคีบ) |
| LiDAR | ขายึดขาว ชั้นกลาง หน้าซ้าย |
| แบตแขน PINSEN | ชั้นกลาง ท้ายหุ่น หลังฐานแขน |

สายไม่ผ่านระดับเลเซอร์ (~25 ซม. จากพื้น) และไม่บังกล้อง · ตำแหน่งที่วัดไว้แล้วอยู่ใน `robot/kobuki_x2.urdf` (ย้ายชิ้นไหน = วัดใหม่แก้ `xyz` ของ joint นั้น)

### B2. เสียบสาย

1. เสียบแบต PINSEN เข้าบอร์ดเซอร์โว กดปุ่มแดงให้ไฟติด **ก่อน** เสียบ USB บอร์ด · เปิดสวิตช์ Kobuki
2. เสียบ USB ทั้ง 4 ตัว · **VM:** โยนเข้า VM ทีละตัว (VMware: VM → Removable Devices → *Connect* · VirtualBox: Devices → USB → ติ๊ก)

| อุปกรณ์ | ชื่อในเมนู VM |
|---|---|
| Kobuki | `FTDI` / `Future Technology Devices` |
| LiDAR | `Silicon Labs CP210x` |
| แขน | `QinHeng` / `USB-Enhanced-SERIAL CH343` |
| กล้อง | `PrimeSense` / `ASUS Xtion` |

เช็ค:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
lsusb | grep -i -e 1d27 -e primesense
```
✅ `ttyUSB0` `ttyUSB1` `ttyACM0` + บรรทัด `1d27:0601` (กล้อง)

### B3. คาลิเบรตแขน (มีไฟล์ `robot/so100_calibration.yaml` แล้ว + ไม่ได้ถอดแขน = ข้าม)

หน้าต่างสำรอง:
```bash
python3 so100_calibrate_app.py
```
**Torque OFF** (ประคองแขน) → จัดทีละท่า กด **บันทึกท่านี้** → **บันทึกไฟล์** → วางลูกบาศก์ระหว่างนิ้ว กด **วัดจุดคีบ** → ปิดแอป

| ท่า | จัดยังไง |
|---|---|
| REST | ฐานหันออกหน้า พับสุด มือห้อยเหนือกล้อง ปากหุบ |
| HOME | แขนบนตั้งตรง แขนล่างยื่นออกหน้าขนานพื้น มือชี้ลง **ปากอ้า ~60°** |
| PAN_90 | หมุนฐาน 90° ให้แขนชี้ไป **ซ้าย** ของหุ่น |

### B4. เช็คแขน

จัดแขนท่า REST · หน้าต่างแยก (เปิดค้าง) `python3 so100_driver.py` รอ `SO-100 ready` · แล้วหน้าต่างสำรอง ทีละบรรทัด (มือพร้อมดึงปลั๊กแบตแขน):
```bash
python3 arm_control.py carry
python3 arm_control.py joints 0.5 -1.57 1.57 1.57 0.8
python3 arm_control.py pick 0.31 0.0 0.11
python3 arm_control.py place 0.31 0.0 0.09
python3 arm_control.py rest
```
✅ ทุกบรรทัดจบ `done` · `joints 0.5 ...` แขนต้องหัน**ซ้าย** (หันขวา = คาลิเบรตใหม่) · `pick` ลูกบาศก์บนกล่อง 9 ซม. กลางลูกบาศก์ห่างกลางหุ่น 31 ซม. ต้องคีบขึ้นมาได้ · เสร็จแล้ว `Ctrl+C` ไดรเวอร์

---

## C. ทำครั้งเดียวต่อสนาม (ย้ายกล่อง = ทำ C3 ใหม่อย่างเดียว)

### C1. แผนที่

**วางหุ่นที่ Initial หันไปทาง Station 1** (มาร์คเทป: จุดนี้ = (0,0) ของแผนที่) · เปิดค้าง 5 หน้าต่าง:
```bash
python3 lidar_ros2.py --mirror
```
```bash
python3 kobuki_ros2.py --enable-motion
```
```bash
ros2 launch ./robot_model.launch.py arm:=rest
```
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file slam_params.yaml
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35
```
และ RViz: `rviz2 -d mapping.rviz`

ขับ (คลิกหน้าต่าง teleop): `i` หน้า · `,` ถอย · `j`/`l` หมุน · `k` หยุด · วนช้า ๆ ผ่านทั้ง 2 Station แล้วกลับ Initial · กำแพงต้องเป็นเส้นเดียวคม → หน้าต่างสำรอง:
```bash
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p save_map_timeout:=10.0
```
✅ `Map saved successfully` → **ปิดทุกหน้าต่างของ C1** (slam ค้าง = ตอนนำทางหุ่นเดินมั่ว)

(ย้าย LiDAR มาใหม่และแผนที่ซ้อนเบลอ: ระหว่างเปิด LiDAR + Kobuki + teleop รัน `python3 lidar_calib.py` ทำตามที่บอก แล้วใส่มุมที่ได้ใน `laser_joint` ของ URDF)

### C2. ค่าสี

วางลูกบาศก์ + แผ่นจุดวางให้กล้องเห็น ใต้ไฟห้องจริง · เปิดค้าง กล้อง (คำสั่งหน้าต่าง 6 ในตาราง D) · หน้าต่างสำรอง:
```bash
python3 hsv_tuner.py
```
กด `1` คลิกลูกบาศก์ · `2` คลิกแผ่นจุดวาง · `q` → ได้บรรทัด `-p cube_hsv_low:=[...] ...` **ก๊อปเก็บ** ด้วย `nano ~/hsv.txt` (วาง → `Ctrl+O` Enter `Ctrl+X`) · ปิดกล้อง

### C3. ตำแหน่ง Station

1. วางกล่อง 9 ซม. + ลูกบาศก์ที่ Station 1 · กล่อง + แผ่นจุดวางที่ Station 2 ตรงที่ใช้จริง (มาร์คเทป)
2. หุ่นที่ Initial · เปิดค้าง: LiDAR, Kobuki, โมเดล (`arm:=rest`) จาก C1 และ
   ```bash
   ros2 launch ./nav2.launch.py map:=$HOME/maps/my_map.yaml
   ```
   ```bash
   rviz2 -d mission.rviz
   ```
3. RViz: **2D Pose Estimate** คลิกตรงหุ่นจริง ลากไปทางที่หุ่นหัน · จุดแดงต้องทับกำแพง · หน้าต่างสำรอง `python3 nav_check.py` ต้อง `ผ่านหมด`
4. หน้าต่างสำรอง:
   ```bash
   python3 save_stations.py
   ```
   กด **Publish Point** ก่อนคลิกทุกครั้ง คลิก 4 ครั้ง: กลางลูกบาศก์ → พื้นฝั่งที่หุ่นจะวิ่งเข้าหา (โล่ง ห่าง ~1 ม.) → กลางแผ่นจุดวาง → พื้นฝั่งที่หุ่นจะวิ่งเข้าหา
   ✅ `เซฟแล้ว: .../robot/stations.yaml` (⚠ ชนกำแพง = Enter แล้วคลิกฝั่งอื่น)
5. ซ้อมวิ่ง: `python3 mission_nav2.py --nav-only`
   ✅ จอดเอา**ด้านหน้าหุ่น**เข้าหากล่อง ทั้ง 2 Station แล้วกลับ Initial · `Mission finished: succeeded`

### C4. เก็บไฟล์

```bash
mkdir -p ~/backup && cp ~/maps/my_map.* ~/hsv.txt stations.yaml so100_calibration.yaml ~/backup/
```

---

## D. ทุกรอบ: เปิดสวิตช์ → หุ่นกลับ Initial

**ก่อนเปิด:** หุ่นที่ Initial หันทางเดิม · แขนท่า REST · สวิตช์ Kobuki · ปุ่มแดงแบตแขน · USB ครบ (VM: Connect ครบ) · ลูกบาศก์/จุดวางบนกล่องตรงเทป

เปิดทีละหน้าต่างตามเลข เปิดค้างไว้ทั้งหมด (แต่ละหน้าต่างเริ่มด้วย 2 บรรทัดกติกา):

**1 LiDAR** — ✅ `First complete scan`
```bash
python3 lidar_ros2.py --mirror
```
**2 Kobuki** — ✅ `Motion ready`
```bash
python3 kobuki_ros2.py --enable-motion
```
**3 โมเดล** — ✅ ไม่มีบรรทัดแดง
```bash
ros2 launch ./robot_model.launch.py arm:=none
```
**4 แขน** — ✅ `SO-100 ready`
```bash
python3 so100_driver.py
```
**5 MoveIt** — ✅ `You can start planning now!`
```bash
ros2 launch ./moveit.launch.py rviz:=false
```
**6 กล้อง** — ✅ ไม่มี `No devices connected`
```bash
ros2 run openni2_camera openni2_camera_driver --ros-args -r rgb/image_raw:=/camera/color/image_raw -r rgb/camera_info:=/camera/color/camera_info -p rgb_frame_id:=camera_color_optical_frame -p depth_registration:=false -p use_device_time:=false
```
**7 หาวัตถุ** — ต่อท้ายด้วยบรรทัดใน `~/hsv.txt` · ✅ `Cube detector on ...`
```bash
python3 cube_detector.py --ros-args -p surface_height:=0.09 -p cube_hsv_low:=[...] -p cube_hsv_high:=[...] -p marker_hsv_low:=[...] -p marker_hsv_high:=[...]
```
**8 Nav2** — ✅ `bt_navigator ... Activating`
```bash
ros2 launch ./nav2.launch.py map:=$HOME/maps/my_map.yaml
```
**9 RViz** — ✅ แผนที่ + จุดแดงทับกำแพง
```bash
rviz2 -d mission.rviz
```
**10 เสียง** — ✅ "หุ่นยนต์พร้อมแล้ว"
```bash
python3 announcer.py
```

**ก่อนกดเริ่ม:** RViz **2D Pose Estimate** ให้ตรงหุ่น · หน้าต่างสำรอง `python3 nav_check.py` → `ผ่านหมด`

**11 ภารกิจ**
```bash
python3 mission_nav2.py --moveit
```
✅ ~4 นาที จบ `Mission finished: succeeded` หุ่นกลับ Initial · **รอบต่อไป:** วางลูกบาศก์กลับเทป ยกหุ่นกลับ Initial รันหน้าต่าง 11 ซ้ำ (หน้าต่างอื่นไม่ต้องปิด)

กรรมการดู: กดเริ่มแล้ว **ห้ามแตะหุ่น ห้ามชี้ทาง**

**🛑 หยุดฉุกเฉิน:** `Ctrl+C` หน้าต่าง 11 แล้วหน้าต่าง 2 (ล้อหยุด)

---

## ถ้าติด

| อาการ | แก้ |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` (หรือเปิด terminal ใหม่หลังทำ A3) |
| `No unique Kobuki adapter` / `servo board not found` | ตัวนั้นยังไม่เสียบ / ยังไม่ Connect เข้า VM · `ls /dev/ttyUSB* /dev/ttyACM*` ต้องเห็น |
| `Permission denied: '/dev/ttyUSB0'` | ยังไม่ได้ logout/login หลัง `usermod -aG dialout` |
| `package ... not found` | ลงแพ็กเกจ A2 ไม่ครบ: รันบรรทัดนั้นอีกครั้ง |
| แขนใน RViz สีขาวกองที่พื้น | หน้าต่าง 4 ยังไม่ขึ้น `SO-100 ready` · ไม่ใช้แขน = หน้าต่าง 3 ใช้ `arm:=rest` |
| `servos not answering` | แบตแขนดับ: กดปุ่มแดง · แล้ว Connect บอร์ดแขนเข้า VM ใหม่ |
| MoveIt `error code -4` / `couldn't receive full current joint state` | หน้าต่างไหนลืมบรรทัด `export FASTRTPS...` → ปิด 3, 4, 5 เปิดใหม่พร้อมบรรทัดนั้น |
| MoveIt ไม่ผ่านแต่เวลาไม่พอ | หน้าต่าง 11 ตัด `--moveit` ออก |
| ลูกบาศก์หลุด | หน้าต่าง 4 ต้องขึ้น `gripper blocked ... holding gently` ตอนหนีบ · ไม่ขึ้น = แอปคาลิเบรต กด **วัดจุดคีบ** ใหม่ |
| กล้อง `No devices connected` | Xtion ยังไม่ Connect เข้า VM · USB ของ VM ต้องเป็น 2.0 · ถอด/Connect ใหม่ · ไม่ได้จริง ๆ ใช้เว็บแคม: `python3 camera_node.py --ros-args -p backend:=opencv -p device:=0 -p hfov_deg:=60` |
| สั่งเดินแล้วหุ่นเดินมั่ว / หมุนวน | `python3 nav_check.py` แก้ตาม ❌ · 2D Pose Estimate ใหม่ · slam ต้องปิดแล้ว · VM ช้า = ปิดโปรแกรมอื่น เพิ่ม CPU |
| จุดแดงอยู่บนตัวหุ่น | หน้าต่าง 1: `python3 lidar_ros2.py --mirror --self-radius 0.28` |
| `no cube in the camera image` | RViz ช่อง Camera เห็นลูกบาศก์ไหม: ไม่เห็น = C3 ใหม่ · เห็นแต่ไม่มีกรอบ = C2 ใหม่ (ไฟห้องเปลี่ยน) |
| `blocked while driving straight` | ล้อติด / ชนอยู่ / หน้าต่าง 2 ไม่มี `--enable-motion` |
| RViz ดำ / ช้ามาก | เปิด 3D acceleration ของ VM · ยังไม่ได้: `export LIBGL_ALWAYS_SOFTWARE=1` ก่อนเปิด RViz |
