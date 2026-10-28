# คู่มือรันคำสั่ง ตั้งแต่เปิดเครื่อง → ได้แผนที่ → หุ่นวิ่งไป Station เอง (ทวนใหม่ทีละขั้น)

ยังไม่ได้ติดตั้ง? ทำตาม `README.md` หน้าแรกของ repo ก่อน (Windows ใช้ pixi / Ubuntu ใช้ apt) แล้วค่อยกลับมาที่คู่มือนี้
วันเดโม (ประกอบหุ่น วัดตำแหน่ง แขน กล้อง ภารกิจเต็ม): ดู **[DEMO_DAY_TH.md](DEMO_DAY_TH.md)** คู่มือนี้คือขั้นแผนที่ + Nav2
ใช้ได้ 2 เครื่อง เลือกอย่างใดอย่างหนึ่ง:

| | Ubuntu VM (ใช้ต่อกับ Nav2) | Windows (เคยรันทำแผนที่ได้แล้ว 18 ต.ค.) |
|---|---|---|
| โฟลเดอร์ | `~/kobuki-pick-place/robot` | โฟลเดอร์โปรเจกต์ที่ clone มา เช่น `~\kobuki-pick-place` (ไฟล์อยู่ใน `robot\`) |
| ขึ้นต้นคำสั่ง | `source /opt/ros/humble/setup.bash` ก่อน 1 ครั้งต่อหน้าต่าง | `pixi run` นำหน้าทุกคำสั่ง |

> **ทุก node = 1 หน้าต่าง terminal** เปิดค้างไว้ อย่าปิด ถ้าจะหยุดกด `Ctrl+C` ในหน้าต่างนั้น

ภาพรวม: หุ่นยนต์ส่งข้อมูล 2 ทาง แล้วต่อเป็นแผนที่

```
LiDAR ──/scan──┐
               ├──► SLAM ──/map──► RViz (ดูภาพ)
Kobuki ──/odom─┘        ▲
  ▲                     │ TF: base_link → laser (จาก Robot model)
  └──/cmd_vel── Teleop (คีย์บอร์ด)
```

---

## ขั้น 0 — เตรียม terminal

**Ubuntu** (ทำในทุกหน้าต่างใหม่):
```bash
source /opt/ros/humble/setup.bash
cd ~/kobuki-pick-place/robot
```
(ถ้าไม่อยากพิมพ์ทุกครั้ง: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` ครั้งเดียวพอ)

**Windows** (PowerShell, ทุกหน้าต่างใหม่):
```powershell
cd ~\kobuki-pick-place      # โฟลเดอร์ที่ clone มา
```

เช็คว่า ROS ใช้ได้: `ros2 topic list` (Windows: `pixi run ros2 topic list`) ต้องเห็น `/parameter_events` กับ `/rosout`

---

## ขั้น 1 — เสียบ USB 2 เส้น  🔌 ต้องมีหุ่นยนต์

- **Ubuntu VM:** เมนู VMware > VM > Removable Devices
  - `Future Technology Devices` (Kobuki) > Connect
  - `Silicon Labs CP210x` (LiDAR) > Connect
  - เช็ค: `ls /dev/ttyUSB*` ต้องเห็น **2 พอร์ต**
- **Windows:** ปกติ Kobuki = `COM10`, LiDAR = `COM11` (เลขอาจเปลี่ยน ไม่ต้องใส่ `--port` สคริปต์หาเองได้)

---

## ขั้น 2 — หน้าต่าง 1: LiDAR  🔌

```bash
python3 lidar_ros2.py --mirror
```
Windows: `pixi run python robot\lidar_ros2.py --mirror`

✅ ต้องเห็น `Dropping returns within 0.24 m of the robot centre ...` แล้ว `First complete scan: ... valid returns`
`--mirror` = กลับทิศมุม (X2 หมุนตามเข็ม) — ค่านี้ calibrate แล้ว **ต้องใส่เสมอ**
จุดที่เลเซอร์โดนตัวหุ่นเอง (แขน แบต) ถูกตัดทิ้งให้อัตโนมัติ (`--self-radius 0.24` อ่านตำแหน่ง LiDAR จาก `kobuki_x2.urdf`)

---

## ขั้น 3 — หน้าต่าง 2: ฐานล้อ Kobuki  🔌

```bash
python3 kobuki_ros2.py --enable-motion
```
Windows: `pixi run python robot\kobuki_ros2.py --enable-motion`

✅ ต้องเห็น `First odometry` แล้วตามด้วย `Motion ready`
- ไม่ใส่ `--enable-motion` = อ่านอย่างเดียว หุ่นไม่ขยับ (ปลอดภัยไว้เช็ค)
- ใส่แล้ว: จำกัดความเร็ว 0.12 m/s, 0.6 rad/s และหยุดเองถ้าไม่มีคำสั่ง 0.4 s / ชนบัมเปอร์ / ล้อลอย

---

## ขั้น 4 — หน้าต่าง 3: โมเดลหุ่นยนต์ (TF)  ✔️ ไม่ต้องมีหุ่นก็รันได้

```bash
ros2 launch ./robot_model.launch.py
```
Windows: `pixi run ros2 launch robot\robot_model.launch.py`

ทำหน้าที่บอก ROS ว่า LiDAR อยู่ตรงไหนบนตัวหุ่น (หน้ากลางหุ่น 7.5 ซม. สูง 0.247 m หมุน -90°) จากไฟล์ `kobuki_x2.urdf`
และวาดหุ่นด้วยไฟล์ CAD ในโฟลเดอร์ `meshes/` (ต้องอยู่ข้างไฟล์ launch) พร้อมส่งท่าพับของแขน SO-100 ผ่าน `arm_rest_publisher.py`
วันที่มีไดรเวอร์แขนจริง: เติม `arm:=none` ท้ายคำสั่ง

---

## ขั้น 5 — หน้าต่าง 4: SLAM (สร้างแผนที่)

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file slam_params.yaml
```
Windows: `pixi run ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file robot\slam_params.yaml`

✅ ไม่ควรมี error สีแดงเรื่อง `TF` ค้างนาน ๆ (ถ้ามี = ขั้น 2–4 ยังไม่ขึ้น)

---

## ขั้น 6 — หน้าต่าง 5: RViz (หน้าจอดูภาพ)

```bash
rviz2 -d mapping.rviz
```
Windows: `pixi run rviz2 -d robot\mapping.rviz`

✅ เห็นแผนที่สีเทา จุดแดงของ LiDAR และโมเดลหุ่น (Fixed Frame = `map` ตั้งไว้แล้วในไฟล์)

---

## ขั้น 7 — หน้าต่าง 6: ขับด้วยคีย์บอร์ด

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35
```
Windows: `pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35`

**คลิกที่หน้าต่างนี้ก่อน + เปลี่ยนคีย์บอร์ดเป็นภาษาอังกฤษ**

| ปุ่ม | ทำอะไร |
|---|---|
| `i` | เดินหน้า |
| `,` | ถอยหลัง |
| `j` / `l` | หมุนซ้าย / ขวา |
| `k` หรือ Space | หยุด |

เทคนิคทำแผนที่: ขับช้า ๆ หมุนช้า ๆ วนให้ครบทุกห้อง แล้ว **กลับมาจุดเริ่ม** (ช่วยให้แผนที่ปิดวงได้ตรง)

---

## ขั้น 8 — เซฟแผนที่ (เปิดหน้าต่างใหม่)

```bash
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p save_map_timeout:=10.0
```
Windows: `mkdir $HOME\maps -Force; pixi run ros2 run nav2_map_server map_saver_cli -f $HOME\maps\my_map --ros-args -p save_map_timeout:=10.0`

✅ ได้ไฟล์ `my_map.pgm` (รูปแผนที่) + `my_map.yaml` (ค่าตั้ง) — **ไฟล์นี้ต้องใช้ต่อกับ Nav2**

---

## ขั้น 9 — Nav2: ให้หุ่นวิ่งไป Station เอง

มีแผนที่แล้ว **ปิด SLAM (หน้าต่าง 4) และ Teleop (หน้าต่าง 6)** แล้ววางหุ่นที่จุดเริ่ม (Initial) หันหน้าทางเดิมตอนเริ่มทำแผนที่
หน้าต่าง 1–3 (LiDAR, Kobuki, Robot model) เปิดค้างไว้เหมือนเดิม

**หน้าต่าง 4 ใหม่: Nav2** (โหลดแผนที่ + หาตำแหน่ง + วางเส้นทาง + ขับ)
```bash
ros2 launch ./nav2.launch.py map:=$HOME/maps/my_map.yaml
```
Windows: `pixi run ros2 launch robot\nav2.launch.py map:=$HOME\maps\my_map.yaml`
✅ เห็น `[bt_navigator]: Activating` = พร้อม

**หน้าต่าง 5: RViz** — Fixed Frame `map`, เพิ่ม Map `/global_costmap/costmap` (Color Scheme `costmap`) และ Path `/plan`
- ถ้าเลเซอร์ไม่ทับกำแพงบนแผนที่: กด **2D Pose Estimate** คลิกตำแหน่งหุ่นจริงแล้วลากไปทางที่หุ่นหัน
- ลองกด **2D Goal Pose** ที่ Station 1 → หุ่นต้องวิ่งไปเอง

**หน้าต่าง 6 ใหม่: ภารกิจเต็ม** Initial → Station 1 → Station 2 → กลับบ้าน
```bash
python3 mission_nav2.py
```
Windows: `pixi run python robot\mission_nav2.py`

> พิกัด Station ใน `mission_nav2.py` (ตัวแปร `STATIONS` = ตำแหน่งวัตถุ + ทิศที่หุ่นถอยเข้าหา) ตั้งไว้สำหรับห้องจำลอง
> **วันจริงต้องแก้ตามแผนที่ห้องจริง** (หาพิกัดด้วย Publish Point) ดู [DEMO_DAY_TH.md](DEMO_DAY_TH.md) ขั้น 7
> รายละเอียดทั้งหมด + การตั้ง RViz ดู [PRACTICE_NO_ROBOT_TH.md](PRACTICE_NO_ROBOT_TH.md) ภาค 4

---

## ทางลัด: เปิดขั้น 2–7 ทีเดียว

เมื่อจำขั้นได้แล้ว ใช้สคริปต์นี้แทน (มันเปิด 6 หน้าต่างตามลำดับข้างบนให้เอง):
- Ubuntu: `./start_mapping.sh`
- Windows: `.\robot\start_mapping.ps1`

## ตรวจว่าทุกอย่างทำงานไหม

```bash
python3 ros_check.py
```
Windows: `pixi run python robot\ros_check.py` — เขียนผลลง `ros_check.txt`

ค่าที่ควรเห็น: `/scan` ประมาณ **4 Hz**, `/odom` ประมาณ **50 Hz**, มี `/map`, และ TF ครบ `map → odom → base_link → laser`

## ปิดระบบ

กด `Ctrl+C` ทีละหน้าต่าง โดย **ปิด Teleop และ Kobuki ก่อน** (หุ่นจะหยุดล้อเองก่อนปิด USB)
ถ้าเปิด node ใหม่แล้วบอกว่าพอร์ตไม่ว่าง = ยังมีหน้าต่างเก่าจับ USB อยู่ ให้ปิดหน้าต่างนั้นก่อน

---

## ซ้อมแบบไม่มีหุ่นยนต์

ทำได้ตอนนี้เลย เพื่อจำลำดับคำสั่ง:
1. หน้าต่าง 1: ขั้น 4 (Robot model)
2. หน้าต่าง 2: `rviz2` (Windows: `pixi run rviz2`)
3. ใน RViz: ช่อง **Fixed Frame** พิมพ์ `base_link` → กด **Add** > **RobotModel** → ตั้ง Description Topic = `/robot_description`
   - จะเห็นหุ่นแบบที่ทีมประกอบ: Kobuki (จุดเหลืองบนหลัง), แผ่นอะคริลิก 2 ชั้น, LiDAR ฝาน้ำเงินด้านหน้า, แขน SO-100 สีดำด้านหลัง, กล้อง Xtion ใต้ชั้นกลาง
4. หน้าต่าง 3: `ros2 node list` และ `ros2 topic list` ดูว่ามีอะไรรันอยู่ (ต้องเห็น `/robot_state_publisher`)
   - ถ้าค้างหรือว่างเปล่า: `ros2 daemon stop` แล้ว `ros2 daemon start` แล้วลองใหม่

เทสต์โค้ดแบบไม่ต่อ USB (Windows):
```powershell
pixi run python -m unittest discover -s robot -p "test*.py" -v
```
