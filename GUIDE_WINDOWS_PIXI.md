# คู่มือหุ่นจริง — Windows (pixi) · ฉบับเดียวจบ

Kobuki + LiDAR X2 + แขน SO-100 + กล้อง Xtion: ออกจาก Initial → หยิบลูกบาศก์ที่ Station 1 → วางที่ Station 2 → กลับ Initial
อ่านจากบนลงล่าง · **A–B ทำครั้งแรกครั้งเดียว · C ทำครั้งเดียวต่อสนาม · D ทำทุกรอบที่รันหุ่น** · ใช้ Ubuntu ให้ดู [GUIDE_UBUNTU.md](GUIDE_UBUNTU.md)

---

## กติกา (อ่านก่อน)

**ทุกหน้าต่าง PowerShell ใหม่ พิมพ์ 2 บรรทัดนี้ก่อนเสมอ** (ไม่ใส่บรรทัดที่ 2 = MoveIt ไม่ได้ท่าแขน แขนไม่ขยับ):
```powershell
cd D:\ROS2_Windows_Humble
$env:FASTRTPS_DEFAULT_PROFILES_FILE = "$PWD\robot\fastdds_local.xml"
```
(เครื่องที่ clone เอง: `cd ~\kobuki-pick-place` แทนบรรทัดแรก)

- 1 คำสั่งที่รันค้าง = 1 หน้าต่าง เปิดค้างไว้ หยุดด้วย `Ctrl+C`
- คำสั่งสั้นที่รันแล้วจบเอง ใช้หน้าต่าง **"สำรอง"** อันเดียวทั้งวัน
- **เสียบสายชาร์จโน้ตบุ๊กไว้ตลอด**

---

## A. ติดตั้ง (ครั้งแรกครั้งเดียว)

1. ลง git: https://git-scm.com/download/win · ลง pixi (PowerShell):
   ```powershell
   powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
   ```
   ปิด PowerShell แล้วเปิดใหม่
2. โหลดโค้ด + ROS (~0.9 GB, 10–30 นาที):
   ```powershell
   cd ~
   git clone https://github.com/SSRobotic/kobuki-pick-place.git
   cd kobuki-pick-place
   pixi install
   pixi add --pypi openni
   ```
3. กล้อง Xtion บน Windows ต้องมีไดรเวอร์ **OpenNI 2 (x64)**: ลง `OpenNI-Windows-x64-2.2` (มีไดรเวอร์ PrimeSense ในตัว) แล้วรีสตาร์ทเครื่อง

อัปเดตโค้ดรอบหลัง: `git pull` (ในโฟลเดอร์โปรเจกต์)

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

สายไม่ผ่านระดับเลเซอร์ (~25 ซม. จากพื้น) และไม่บังกล้อง · ตำแหน่งที่วัดไว้แล้วอยู่ใน `robot\kobuki_x2.urdf` (ย้ายชิ้นไหน = วัดใหม่แก้ `xyz` ของ joint นั้น)

### B2. เสียบสาย

1. เสียบแบต PINSEN เข้าบอร์ดเซอร์โว กดปุ่มแดงให้ไฟติด **ก่อน** เสียบ USB บอร์ด
2. Kobuki, LiDAR, บอร์ดแขน เข้า hub · **Xtion เสียบพอร์ตโน้ตบุ๊กตรง ๆ** · เปิดสวิตช์ Kobuki

✅ Device Manager → Ports: `USB Serial Port` (Kobuki), `Silicon Labs CP210x` (LiDAR), `USB-SERIAL CH343` (แขน)

### B3. คาลิเบรตแขน (มีไฟล์ `robot\so100_calibration.yaml` แล้ว + ไม่ได้ถอดแขน = ข้าม)

หน้าต่างสำรอง:
```powershell
pixi run python robot\so100_calibrate_app.py
```
**Torque OFF** (ประคองแขน) → จัดทีละท่า กด **บันทึกท่านี้** → **บันทึกไฟล์** → วางลูกบาศก์ระหว่างนิ้ว กด **วัดจุดคีบ** → ปิดแอป

| ท่า | จัดยังไง |
|---|---|
| REST | ฐานหันออกหน้า พับสุด มือห้อยเหนือกล้อง ปากหุบ |
| HOME | แขนบนตั้งตรง แขนล่างยื่นออกหน้าขนานพื้น มือชี้ลง **ปากอ้า ~60°** |
| PAN_90 | หมุนฐาน 90° ให้แขนชี้ไป **ซ้าย** ของหุ่น |

### B4. เช็คแขน

จัดแขนท่า REST · หน้าต่างแยก (เปิดค้าง) `pixi run python robot\so100_driver.py` รอ `SO-100 ready` · แล้วหน้าต่างสำรอง ทีละบรรทัด (มือพร้อมดึงปลั๊กแบตแขน):
```powershell
pixi run python robot\arm_control.py carry
pixi run python robot\arm_control.py joints 0.5 -1.57 1.57 1.57 0.8
pixi run python robot\arm_control.py pick 0.31 0.0 0.11
pixi run python robot\arm_control.py place 0.31 0.0 0.09
pixi run python robot\arm_control.py rest
```
✅ ทุกบรรทัดจบ `done` · `joints 0.5 ...` แขนต้องหัน**ซ้าย** (หันขวา = คาลิเบรตใหม่) · `pick` ลูกบาศก์บนกล่อง 9 ซม. กลางลูกบาศก์ห่างกลางหุ่น 31 ซม. ต้องคีบขึ้นมาได้ · เสร็จแล้ว `Ctrl+C` ไดรเวอร์

---

## C. ทำครั้งเดียวต่อสนาม (ย้ายกล่อง = ทำ C3 ใหม่อย่างเดียว)

### C1. แผนที่

**วางหุ่นที่ Initial หันไปทาง Station 1** (มาร์คเทป: จุดนี้ = (0,0) ของแผนที่) · เปิดค้าง 5 หน้าต่าง:
```powershell
pixi run python robot\lidar_ros2.py --mirror
```
```powershell
pixi run python robot\kobuki_ros2.py --enable-motion
```
```powershell
pixi run ros2 launch robot\robot_model.launch.py arm:=rest
```
```powershell
pixi run ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file robot\slam_params.yaml
```
```powershell
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35
```
และ RViz: `pixi run rviz2 -d robot\mapping.rviz`

ขับ (คลิกหน้าต่าง teleop + แป้นอังกฤษ): `i` หน้า · `,` ถอย · `j`/`l` หมุน · `k` หยุด · วนช้า ๆ ผ่านทั้ง 2 Station แล้วกลับ Initial · กำแพงต้องเป็นเส้นเดียวคม → หน้าต่างสำรอง:
```powershell
mkdir $HOME\maps -Force; pixi run ros2 run nav2_map_server map_saver_cli -f $HOME\maps\my_map --ros-args -p save_map_timeout:=10.0
```
✅ `Map saved successfully` → **ปิดทุกหน้าต่างของ C1** (slam ค้าง = ตอนนำทางหุ่นเดินมั่ว)

(ย้าย LiDAR มาใหม่และแผนที่ซ้อนเบลอ: ระหว่างเปิด LiDAR + Kobuki + teleop รัน `pixi run python robot\lidar_calib.py` ทำตามที่บอก แล้วใส่มุมที่ได้ใน `laser_joint` ของ URDF)

### C2. ค่าสี

วางลูกบาศก์ + แผ่นจุดวางให้กล้องเห็น ใต้ไฟห้องจริง · เปิดค้าง: `pixi run python robot\camera_node.py` (✅ `openni camera 640x480`) · หน้าต่างสำรอง:
```powershell
pixi run python robot\hsv_tuner.py
```
กด `1` คลิกลูกบาศก์ · `2` คลิกแผ่นจุดวาง · `q` → ได้บรรทัด `-p cube_hsv_low:=[...] ...` **ก๊อปเก็บใน `notepad $HOME\hsv.txt`** · ปิดกล้อง

### C3. ตำแหน่ง Station

1. วางกล่อง 9 ซม. + ลูกบาศก์ที่ Station 1 · กล่อง + แผ่นจุดวางที่ Station 2 ตรงที่ใช้จริง (มาร์คเทป)
2. หุ่นที่ Initial · เปิดค้าง: LiDAR, Kobuki, โมเดล (`arm:=rest`) จาก C1 และ
   ```powershell
   pixi run ros2 launch robot\nav2.launch.py map:=$HOME\maps\my_map.yaml
   ```
   ```powershell
   pixi run rviz2 -d robot\mission.rviz
   ```
3. RViz: **2D Pose Estimate** คลิกตรงหุ่นจริง ลากไปทางที่หุ่นหัน · จุดแดงต้องทับกำแพง · หน้าต่างสำรอง `pixi run python robot\nav_check.py` ต้อง `ผ่านหมด`
4. หน้าต่างสำรอง:
   ```powershell
   pixi run python robot\save_stations.py
   ```
   กด **Publish Point** ก่อนคลิกทุกครั้ง คลิก 4 ครั้ง: กลางลูกบาศก์ → พื้นฝั่งที่หุ่นจะวิ่งเข้าหา (โล่ง ห่าง ~1 ม.) → กลางแผ่นจุดวาง → พื้นฝั่งที่หุ่นจะวิ่งเข้าหา
   ✅ `เซฟแล้ว: ...robot\stations.yaml` (⚠ ชนกำแพง = Enter แล้วคลิกฝั่งอื่น)
5. ซ้อมวิ่ง: `pixi run python robot\mission_nav2.py --nav-only`
   ✅ จอดเอา**ด้านหน้าหุ่น**เข้าหากล่อง ทั้ง 2 Station แล้วกลับ Initial · `Mission finished: succeeded`

### C4. เก็บไฟล์

แผนที่ `$HOME\maps\my_map.*` · ค่าสี `$HOME\hsv.txt` · `robot\stations.yaml` · `robot\so100_calibration.yaml` → ก๊อปใส่ USB สำรองไว้

---

## D. ทุกรอบ: เปิดสวิตช์ → หุ่นกลับ Initial

**ก่อนเปิด:** หุ่นที่ Initial หันทางเดิม · แขนท่า REST · สวิตช์ Kobuki · ปุ่มแดงแบตแขน · เสียบ USB ครบ · ลูกบาศก์/จุดวางบนกล่องตรงเทป

เปิดทีละหน้าต่างตามเลข เปิดค้างไว้ทั้งหมด (แต่ละหน้าต่างเริ่มด้วย 2 บรรทัดกติกา):

| # | อะไร | คำสั่ง | ✅ |
|---|---|---|---|
| 1 | LiDAR | `pixi run python robot\lidar_ros2.py --mirror` | `First complete scan` |
| 2 | Kobuki | `pixi run python robot\kobuki_ros2.py --enable-motion` | `Motion ready` |
| 3 | โมเดล | `pixi run ros2 launch robot\robot_model.launch.py arm:=none` | ไม่มีบรรทัดแดง |
| 4 | แขน | `pixi run python robot\so100_driver.py` | `SO-100 ready` |
| 5 | MoveIt | `pixi run ros2 launch robot\moveit.launch.py rviz:=false` | `You can start planning now!` |
| 6 | กล้อง | `pixi run python robot\camera_node.py` | `openni camera 640x480` |
| 7 | หาวัตถุ | `pixi run python robot\cube_detector.py --ros-args -p surface_height:=0.09` + บรรทัดใน `hsv.txt` | `Cube detector on ...` |
| 8 | Nav2 | `pixi run ros2 launch robot\nav2.launch.py map:=$HOME\maps\my_map.yaml` | `bt_navigator ... Activating` |
| 9 | RViz | `pixi run rviz2 -d robot\mission.rviz` | แผนที่ + จุดแดงทับกำแพง |
| 10 | เสียง | `pixi run python robot\announcer.py` | "หุ่นยนต์พร้อมแล้ว" |

**ก่อนกดเริ่ม:** RViz **2D Pose Estimate** ให้ตรงหุ่น · หน้าต่างสำรอง `pixi run python robot\nav_check.py` → `ผ่านหมด`

| # | อะไร | คำสั่ง |
|---|---|---|
| 11 | **ภารกิจ** | `pixi run python robot\mission_nav2.py --moveit` |

✅ ~4 นาที จบ `Mission finished: succeeded` หุ่นกลับ Initial · **รอบต่อไป:** วางลูกบาศก์กลับเทป ยกหุ่นกลับ Initial รันหน้าต่าง 11 ซ้ำ (หน้าต่างอื่นไม่ต้องปิด)

กรรมการดู: กดเริ่มแล้ว **ห้ามแตะหุ่น ห้ามชี้ทาง**

**🛑 หยุดฉุกเฉิน:** `Ctrl+C` หน้าต่าง 11 แล้วหน้าต่าง 2 (ล้อหยุด)

---

## ถ้าติด

| อาการ | แก้ |
|---|---|
| หาพอร์ตไม่เจอ | ใส่เอง `--port COM10` (เลขจาก Device Manager) · ปิดหน้าต่างเก่าที่จับพอร์ตอยู่ |
| แขนใน RViz สีขาวกองที่พื้น | หน้าต่าง 4 ยังไม่ขึ้น `SO-100 ready` · ไม่ใช้แขน = หน้าต่าง 3 ใช้ `arm:=rest` |
| `servos not answering` | แบตแขนดับ: กดปุ่มแดง · สายเซอร์โวหลวม |
| แขนกระตุก / `supply dropped` | แบตแขนอ่อน / ปลั๊ก DC หลวม |
| MoveIt `error code -4` / `couldn't receive full current joint state` | หน้าต่างไหนลืมบรรทัด `$env:FASTRTPS...` → ปิด 3, 4, 5 เปิดใหม่พร้อมบรรทัดนั้น |
| MoveIt ไม่ผ่านแต่เวลาไม่พอ | หน้าต่าง 11 ตัด `--moveit` ออก |
| ลูกบาศก์หลุด | หน้าต่าง 4 ต้องขึ้น `gripper blocked ... holding gently` ตอนหนีบ · ไม่ขึ้น = แอปคาลิเบรต กด **วัดจุดคีบ** ใหม่ |
| `ModuleNotFoundError: openni` | `pixi add --pypi openni` (A2) |
| `camera gives no picture` / หา OpenNI ไม่เจอ | ลง OpenNI 2 (A3) · ถอดเสียบ Xtion พอร์ตตรง · ไม่ได้จริง ๆ ใช้เว็บแคม: `pixi run python robot\camera_node.py --ros-args -p backend:=opencv -p device:=1 -p hfov_deg:=60` (เลข device จาก `camera_node.py --list`) |
| สั่งเดินแล้วหุ่นเดินมั่ว / หมุนวน | `pixi run python robot\nav_check.py` แก้ตาม ❌ · 2D Pose Estimate ใหม่ · slam ต้องปิดแล้ว |
| จุดแดงอยู่บนตัวหุ่น | หน้าต่าง 1: `... --mirror --self-radius 0.28` |
| `no cube in the camera image` | RViz ช่อง Camera เห็นลูกบาศก์ไหม: ไม่เห็น = C3 ใหม่ · เห็นแต่ไม่มีกรอบ = C2 ใหม่ (ไฟห้องเปลี่ยน) |
| `blocked while driving straight` | ล้อติด / ชนอยู่ / หน้าต่าง 2 ไม่มี `--enable-motion` |
| โน้ตบุ๊กดับเองตอนถอดสายชาร์จ | เสียบสายชาร์จไว้ · `Fn+Q` ให้ไฟปุ่มเปิดเครื่องไม่เป็นสีแดง |
