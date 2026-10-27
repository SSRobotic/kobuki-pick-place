# คู่มือซ้อมแบบไม่มีหุ่นยนต์ (ละเอียด)

ซ้อมได้ครบทุกขั้นเหมือนวันที่มีหุ่นจริง โดยใช้ **หุ่นยนต์ปลอม** (`fake_robot.py`)
หุ่นปลอมจำลองห้องตามผังในโจทย์ Final Project (Station 1, Station 2, จุดเริ่ม, โต๊ะ, ประตู)
แล้วส่งข้อมูลออกมา **ชื่อเดียวกับหุ่นจริงทุกอย่าง** (`/scan`, `/odom`, TF) — SLAM, RViz, Teleop จึงใช้คำสั่งเดิมได้เลย

| ภาค | ทำอะไร | ใช้เวลา |
|---|---|---|
| 1 | ดูโมเดลหุ่น + ตั้งค่า RViz เองทีละช่อง | ~20 นาที |
| 2 | หุ่นปลอม → ทำแผนที่ → เซฟแผนที่ (ซ้อมเหมือนวันจริง) | ~40 นาที |
| 3 | ฝึกคำสั่งตรวจระบบ + หาพิกัด Station | ~15 นาที |
| 4 | Nav2: หุ่นวิ่งไป Station 1 → 2 → กลับบ้านเอง | ~30 นาที |

---

## กติกาการพิมพ์คำสั่ง (อ่านก่อน)

คู่มือนี้เขียนคำสั่งแบบ **Ubuntu** ถ้าใช้ **Windows** ให้แปลงแบบนี้:

| Ubuntu | Windows (PowerShell) |
|---|---|
| เปิดหน้าต่างใหม่แล้ว `source /opt/ros/humble/setup.bash` และ `cd ~/kobuki-pick-place/robot` | เปิดหน้าต่างใหม่แล้ว `cd ~\kobuki-pick-place` (โฟลเดอร์ที่ clone มา) |
| `python3 fake_robot.py` | `pixi run python robot\fake_robot.py` |
| `ros2 ...` | `pixi run ros2 ...` |
| `rviz2 ...` | `pixi run rviz2 ...` |
| ไฟล์ `slam_params.yaml` | `robot\slam_params.yaml` |

> **1 คำสั่งที่รันค้าง = 1 หน้าต่าง** อย่าปิดหน้าต่าง หยุดด้วย `Ctrl+C`
> คำสั่งที่รันแล้วจบเอง (เช่น `ros2 topic list`) ใช้หน้าต่าง "สำรอง" หน้าต่างเดียวพอ

ไฟล์ที่ต้องมีในโฟลเดอร์: `fake_robot.py`, `kobuki_x2.urdf`, `robot_model.launch.py`, `arm_rest_publisher.py`, โฟลเดอร์ `meshes/` (ไฟล์ CAD ของหุ่น), `slam_params.yaml`, `mapping.rviz`, `ros_check.py`
(ทั้งหมดอยู่ในโฟลเดอร์ `robot/` ของ repo ได้มาจาก `git clone` ดูวิธีใน `README.md`)

---

# ภาค 1 — โมเดลหุ่น + ตั้งค่า RViz เองทั้งหมด

## 1.1 หน้าต่าง A: เปิดโมเดลหุ่น

```bash
ros2 launch ./robot_model.launch.py
```
Windows: `pixi run ros2 launch robot\robot_model.launch.py`

✅ ต้องเห็น:
```
[robot_state_publisher-1] ... got segment base_link
[robot_state_publisher-1] ... got segment gripper_tip
[robot_state_publisher-1] ... got segment laser
[robot_state_publisher-1] ... got segment lidar_deck
[robot_state_publisher-1] ... got segment so100_base
   ... (so100_gripper, so100_jaw, so100_lower_arm, so100_shoulder, so100_upper_arm, so100_wrist)
[robot_state_publisher-1] ... got segment wheel_left_link
[robot_state_publisher-1] ... got segment wheel_right_link
```
**คืออะไร:** อ่านไฟล์ `kobuki_x2.urdf` แล้วประกาศให้ ROS รู้ว่าหุ่นมีชิ้นส่วนอะไร อยู่ตรงไหน
(เช่น LiDAR สูง 0.20 m จากพื้น หมุน -90°) ส่งออกทาง topic `/robot_description` และ `/tf_static`
launch นี้เปิดอีกโปรแกรมด้วย คือ `arm_rest_publisher.py` ส่งมุมข้อต่อแขน SO-100 ท่าพับทาง `/joint_states`
(ไม่มีตัวนี้ แขนจะไม่ขึ้นใน RViz) วันที่มีไดรเวอร์แขนจริงแล้วให้สั่ง `... robot_model.launch.py arm:=none`
> รูปหุ่นใน RViz มาจากไฟล์ CAD ในโฟลเดอร์ `meshes/` (Kobuki ของ Yujin, SO-100 ของ TheRobotStudio, X2 ขึ้นรูปตามดาต้าชีท)
> ย้ายไฟล์ไปเครื่องอื่นต้องย้ายโฟลเดอร์ `meshes/` ไปด้วยเสมอ ตำแหน่งติดแขนและแผ่นรอง LiDAR ยังเป็นค่าประมาณ

## 1.2 หน้าต่าง B: เปิด RViz แบบเปล่า (ไม่ใช้ไฟล์ตั้งค่า)

```bash
rviz2
```
Windows: `pixi run rviz2`

## 1.3 รู้จักหน้าจอ RViz

```
┌───────────────────────────────────────────────────────────────┐
│ File  Panels  Help                                            │
│ [Interact][Move Camera][Select][Focus][Measure]               │
│ [2D Pose Estimate][2D Goal Pose][Publish Point]   ← แถบเครื่องมือ│
├──────────────┬────────────────────────────────┬───────────────┤
│ Displays     │                                │ Views         │
│  Global Opt. │        จอ 3D (ภาพหุ่น/แผนที่)      │ (มุมกล้อง)      │
│  Global Stat.│                                │               │
│  Grid        │                                │               │
│              │                                │               │
│ [Add][Duplicate][Remove][Rename]               │               │
├──────────────┴────────────────────────────────┴───────────────┤
│ Time: ROS Time ...                       [Reset]               │
└───────────────────────────────────────────────────────────────┘
```
- **Displays (ซ้าย):** รายการสิ่งที่จะแสดง แต่ละอันกดลูกศร ▸ เพื่อกางดูค่าตั้งได้
- **ปุ่ม Add (ล่างซ้าย):** เพิ่มสิ่งที่จะแสดง
- **Views (ขวา):** ถ้าไม่เห็น ไปที่เมนู Panels > Views
- **Reset (ล่างขวา):** กดเมื่อภาพค้าง/เพี้ยน หลังรีสตาร์ท node

**เมาส์ในจอ 3D**

| ทำอะไร | วิธี |
|---|---|
| หมุนมุมมอง | ลากปุ่มซ้าย |
| เลื่อนภาพ | ลากปุ่มกลาง (หรือ Shift + ลากซ้าย) |
| ซูม | หมุนลูกกลิ้ง (หรือลากปุ่มขวาขึ้นลง) |

## 1.4 ตั้ง Fixed Frame

1. ใน Displays กด ▸ ที่ **Global Options**
2. ช่อง **Fixed Frame** (ค่าเดิมคือ `map`) → คลิกแล้วพิมพ์ `base_link` → Enter
3. **Global Status** ต้องเปลี่ยนจาก ❌ สีแดงเป็น ✔️

**Fixed Frame คืออะไร:** "จุดที่กล้องยึดไว้นิ่ง ๆ" ภาค 1 ยังไม่มีแผนที่ เลยยึดกับตัวหุ่น (`base_link`) ไปก่อน

## 1.5 ตั้ง Grid (ตาราง)

กด ▸ ที่ **Grid** แล้วตั้ง:

| ช่อง | ค่า | ทำไม |
|---|---|---|
| Cell Size | `1` | 1 ช่อง = 1 เมตร ไว้กะระยะ |
| Plane Cell Count | `20` | ตาราง 20×20 เมตร |
| Color | `160; 160; 164` | สีเทา |

## 1.6 เพิ่มโมเดลหุ่น (RobotModel)

1. กด **Add** → แท็บ **By display type** → `rviz_default_plugins` → **RobotModel** → OK
2. กด ▸ ที่ **RobotModel** แล้วตั้ง:

| ช่อง | ค่า |
|---|---|
| Description Source | `Topic` |
| Description Topic | `/robot_description` |
| Description Topic > **Durability Policy** | **`Transient Local`** ← สำคัญ |
| Alpha | `1` |

✅ เห็นหุ่นจากไฟล์ CAD: Kobuki สีดำมี **จุดเหลืองบนหลัง** · LiDAR X2 สีดำบนแผ่นรองสีเทา · **แขน SO-100 สีส้มพับอยู่ด้านหน้า (ฝั่งที่แขนยื่นไป = ด้านหน้าหุ่น)**
   ถ้า RobotModel ขึ้น error `Could not load resource` = หาโฟลเดอร์ `meshes/` ไม่เจอ ต้องเปิดผ่าน `robot_model.launch.py` เท่านั้น

> **ทำไมต้อง Transient Local:** `robot_state_publisher` ส่งโมเดล **ครั้งเดียว** ตอนเปิด
> ถ้าเป็น `Volatile` RViz ที่เปิดทีหลังจะไม่ได้รับ → หุ่นไม่ขึ้น
> `Transient Local` = "ขอข้อความล่าสุดที่ส่งไปแล้วด้วย"

## 1.7 เพิ่มแกนพิกัด (TF)

1. **Add** → **TF** → OK
2. ตั้งค่า:

| ช่อง | ค่า |
|---|---|
| Show Names | ✔️ |
| Show Axes | ✔️ |
| Show Arrows | ☐ |
| Marker Scale | `0.4` |

✅ เห็นแกน 3 สี พร้อมชื่อ `base_link`, `laser`, `wheel_left_link`, `wheel_right_link`, `lidar_deck` และแกนของแขน `so100_*` กับ `gripper_tip` (จุดกลางระหว่างปากคีบ)
- **แดง = แกน X (หน้า)** / **เขียว = แกน Y (ซ้าย)** / **น้ำเงิน = แกน Z (บน)**
- สังเกต: แกนแดงของ `laser` ชี้ไป **ขวา** ของหุ่น (หมุน -90°) — นี่คือค่าที่ calibrate จากหุ่นจริง

## 1.8 ปรับมุมกล้อง (Views)

1. ใน Views: **Type** = `Orbit` (หมุนรอบจุด)
2. **Target Frame** = `base_link`
3. ลองเปลี่ยน Type เป็น `TopDownOrtho` = มองจากด้านบนตรง ๆ (เหมาะกับดูแผนที่) แล้วเปลี่ยนกลับ

## 1.9 เซฟการตั้งค่า

เมนู **File > Save Config As** → ตั้งชื่อ `practice_model.rviz` ไว้ในโฟลเดอร์งาน
ครั้งหน้าเปิดด้วย `rviz2 -d practice_model.rviz` ไม่ต้องตั้งใหม่

## 1.10 หน้าต่าง C (สำรอง): ดูข้อมูลด้วยคำสั่ง

```bash
ros2 node list
```
✅ `/robot_state_publisher`, `/rviz` (หรือ `/rviz2`)

```bash
ros2 topic list -t
```
✅ มี `/robot_description [std_msgs/msg/String]`, `/tf_static [tf2_msgs/msg/TFMessage]`

```bash
ros2 run tf2_ros tf2_echo base_link laser
```
✅ `Translation: [0.000, 0.000, 0.200]` และ `Rotation: in RPY (degree) [0.000, -0.000, -90.000]`
→ LiDAR สูง 20 ซม. หมุน -90° ตรงกับ URDF (กด `Ctrl+C` ออก)

```bash
ros2 run tf2_tools view_frames
```
✅ ได้ไฟล์ `frames_<วันที่>.pdf` เปิดดูจะเห็นแผนผัง `base_link → laser / wheel_*`

**จบภาค 1:** ปิด RViz และกด `Ctrl+C` ในหน้าต่าง A

---

# ภาค 2 — หุ่นปลอม + ทำแผนที่ (เหมือนวันจริง)

## ผังหน้าต่างที่ต้องเปิด

| หน้าต่าง | วันจริง | วันซ้อม (ใช้อันนี้) |
|---|---|---|
| 1 | LiDAR `lidar_ros2.py --mirror` | **`fake_robot.py`** (แทนทั้ง 1 และ 2) |
| 2 | Kobuki `kobuki_ros2.py --enable-motion` | — (ไม่ต้องเปิด) |
| 3 | Robot model | Robot model (เหมือนเดิม) |
| 4 | SLAM | SLAM (เหมือนเดิม) |
| 5 | RViz | RViz (ตั้งเองในข้อ 2.5) |
| 6 | Teleop | Teleop (เหมือนเดิม) |
| 7 | สำรอง | สำรอง |

> ⚠️ **ห้ามเปิด `fake_robot.py` พร้อมกับหุ่นจริง** — จะมี `/scan`, `/odom` ซ้ำ 2 แหล่ง ข้อมูลจะตีกัน

## 2.1 หน้าต่าง 1: หุ่นปลอม

```bash
python3 fake_robot.py
```
Windows: `pixi run python robot\fake_robot.py`

✅ ต้องเห็น:
```
[fake_robot]: Fake robot ready at Initial Location. Publishing /scan, /odom, TF odom -> base_link, /stations
[fake_robot]:   Station 1: x=5.04 y=1.32 (odom)
[fake_robot]:   Station 2: x=2.85 y=-2.58 (odom)
[fake_robot]:   Initial: x=0.00 y=0.00 (odom)
```
**ผังห้องจำลอง** (มองจากด้านบน หุ่นเริ่มที่ Initial หันหน้าขึ้นไปทาง Station 1):
```
 ┌──────────────────────┬────────┐
 │ [desk]               │        │
 │  ● Station 1         ┊ประตูบน  │
 │       [  โต๊ะ  ]      │        │
 │                 [ ]  │  ● Station 2 [desk]
 ├──────┐ ทางเข้า ┌──────┤        │
 │ [   ]          [ ]   │[ ]     │
 │ [ ]    ● Initial     ┊ประตูล่าง │
 │        [ FRONT ]     │        │
 └──────────────────────┴────────┘
       ↑ หุ่นหันหน้าขึ้น (แกน X) ซ้ายมือ = แกน Y
```
ขนาดห้องประมาณ 6 × 7.6 เมตร

## 2.2 หน้าต่าง 3: โมเดลหุ่น

```bash
ros2 launch ./robot_model.launch.py
```
Windows: `pixi run ros2 launch robot\robot_model.launch.py`

## 2.3 หน้าต่าง 7 (สำรอง): ตรวจก่อนเปิด SLAM

ตรวจว่าข้อมูลมาครบ (เหมือนที่ต้องเช็ควันจริง):

```bash
ros2 topic hz /scan
```
✅ `average rate: 4.000` (ประมาณ 4 ครั้ง/วินาที) → `Ctrl+C`

```bash
ros2 topic hz /odom
```
✅ ประมาณ `50` → `Ctrl+C`

```bash
ros2 run tf2_ros tf2_echo odom base_link
```
✅ `Translation: [0.000, 0.000, 0.000]` (หุ่นยังอยู่จุดเริ่ม) → `Ctrl+C`

## 2.4 หน้าต่าง 4: SLAM

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file slam_params.yaml
```
Windows: `pixi run ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file robot\slam_params.yaml`

✅ ต้องเห็น `Registering sensor: [Custom Described Lidar]`
❌ ถ้าขึ้น `Message Filter dropping message` รัว ๆ ไม่หยุด = ยังไม่ได้เปิดหน้าต่าง 1 หรือ 3

**SLAM ทำอะไร:** เอา `/scan` (ระยะกำแพง) + `/odom` (หุ่นเดินไปเท่าไหร่) มาต่อกันเป็นแผนที่ `/map`
และสร้าง TF `map → odom` (แก้ตำแหน่งหุ่นให้ตรงกับแผนที่)

## 2.5 หน้าต่าง 5: RViz — ตั้งค่าเองทีละอัน

```bash
rviz2
```
ทำตามลำดับ ทำเสร็จแต่ละข้อให้ดูว่าเห็นภาพตาม ✅ ก่อนไปข้อต่อไป

### ก) Fixed Frame = `odom`
Global Options > Fixed Frame = `odom` (ยังไม่ใช้ `map` จนกว่าข้อ ฉ)

### ข) Grid
ตั้งเหมือนข้อ 1.5 (Cell Size `1`, Plane Cell Count `20`)

### ค) RobotModel
ตั้งเหมือนข้อ 1.6 — **อย่าลืม Durability = Transient Local**
✅ หุ่นอยู่ตรงกลางตาราง

### ง) LaserScan (จุดเลเซอร์)
1. **Add** → **LaserScan** → OK
2. ตั้งค่า:

| ช่อง | ค่า |
|---|---|
| Topic | `/scan` |
| Topic > **Reliability Policy** | **`Best Effort`** ← สำคัญ |
| Topic > Durability Policy | `Volatile` |
| Style | `Flat Squares` |
| Size (m) | `0.04` |
| Color Transformer | `FlatColor` |
| Color | `255; 40; 40` (แดง) |
| Decay Time | `0` |

✅ เห็นจุดแดงเป็นรูปกำแพงห้องรอบหุ่น
> **ทำไมต้อง Best Effort:** LiDAR ส่งแบบ "ส่งเร็ว ไม่รอยืนยัน" (Best Effort)
> ถ้า RViz ตั้งเป็น `Reliable` จะไม่ตรงกัน → **ไม่ขึ้นอะไรเลย** แต่ไม่มี error ชัด ๆ (หุ่นจริงก็เหมือนกัน)

### จ) Odometry (ลูกศรตำแหน่งหุ่น)
1. **Add** → **Odometry** → OK
2. ตั้งค่า:

| ช่อง | ค่า |
|---|---|
| Topic | `/odom` |
| Topic > Reliability Policy | `Reliable` |
| Keep | `50` |
| Shape | `Arrow` |
| Covariance | ☐ (ปิด) |
| Position Tolerance | `0.1` |

✅ เห็นลูกศรแดงที่หุ่น (ขับแล้วจะทิ้งรอยลูกศรไว้ทุก 10 ซม.)

### ฉ) Map (แผนที่)
1. **Add** → **Map** → OK
2. ตั้งค่า:

| ช่อง | ค่า |
|---|---|
| Topic | `/map` |
| Topic > **Durability Policy** | **`Transient Local`** |
| Topic > Reliability Policy | `Reliable` |
| Color Scheme | `map` |
| Alpha | `0.8` |
| Draw Behind | ✔️ |

3. **เปลี่ยน Global Options > Fixed Frame เป็น `map`**

✅ เห็นแผนที่: **ขาว = ที่ว่าง** / **ดำ = กำแพง** / **เทา = ยังไม่รู้**
❌ ถ้า Map ขึ้น `No map received` → SLAM (หน้าต่าง 4) ยังไม่ทำงาน หรือ Durability ไม่ใช่ Transient Local

### ช) TF
ตั้งเหมือนข้อ 1.7 ✅ จะเห็นแกน `map`, `odom`, `base_link`, `laser`

### ซ) MarkerArray (จุด Station — มีเฉพาะหุ่นปลอม)
1. **Add** → **MarkerArray** → OK
2. Topic = `/stations`

✅ เห็นวงกลมแดง `Station 1`, `Station 2` และวงฟ้า `Initial` บนพื้น

### ฌ) มุมกล้องแบบมองบน
Views > Type = `TopDownOrtho`, Target Frame = `<Fixed Frame>` → ซูมออกให้เห็นทั้งห้อง

### ญ) เซฟ
**File > Save Config As** → `practice_mapping.rviz`
ครั้งหน้า: `rviz2 -d practice_mapping.rviz`

## 2.6 หน้าต่าง 6: ขับหุ่นด้วยคีย์บอร์ด

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35
```
Windows: `pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35`

**ต้องคลิกที่หน้าต่างนี้ + เปลี่ยนคีย์บอร์ดเป็นภาษาอังกฤษ ก่อนกดปุ่ม**

```
   u    i    o        i = เดินหน้า      , = ถอยหลัง
   j    k    l        j = หมุนซ้าย      l = หมุนขวา
   m    ,    .        k / Space = หยุด
                      u o m . = เลี้ยวโค้ง
q / z = เพิ่ม/ลดความเร็วทั้งหมด 10%  (หุ่นปลอม/หุ่นจริงจำกัดไว้ที่ 0.12 m/s อยู่แล้ว)
```
**กดค้าง** = เดินต่อเนื่อง ปล่อยแล้วหุ่นหยุดเองใน 0.4 วินาที (เหมือนหุ่นจริง)
ถ้าชนกำแพง/โต๊ะ หน้าต่าง 1 จะขึ้น `Bump! ...` → ให้ถอย (`,`) หรือหมุนออก

### เส้นทางแนะนำ (วนครบห้อง กลับจุดเดิม = แผนที่แม่นสุด)
1. ที่ Initial: กด `j` หมุนช้า ๆ 1 รอบ ให้ SLAM เห็นรอบตัว
2. `i` ตรงขึ้นไป ~2.5 ม. ลอด **ทางเข้า** ห้องบน (ทางเข้าอยู่ตรงหน้าหุ่นพอดี)
3. เบี่ยงซ้ายอ้อมโต๊ะกลาง → ไป **Station 1** (วงแดงซ้ายบน)
4. ไปทางขวา ออก **ประตูบน** สู่ทางเดินขวา
5. ลงมาที่ **Station 2** (วงแดงขวา ข้างโต๊ะ)
6. ลงต่อ เข้า **ประตูล่าง** กลับห้องล่าง
7. กลับ **Initial** — ดูใน RViz ว่ากำแพงเดิมทับกันพอดี (loop closure)

✅ ใน RViz แผนที่ค่อย ๆ ขยายตามที่หุ่นไป เส้นกำแพงคม ไม่เบลอซ้อน

## 2.7 เซฟแผนที่ (หน้าต่าง 7)

Ubuntu:
```bash
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/practice_map
```
Windows:
```powershell
mkdir $HOME\maps -Force; pixi run ros2 run nav2_map_server map_saver_cli -f $HOME\maps\practice_map
```
✅ ต้องเห็น `Map saved successfully` และได้ 2 ไฟล์:
- `practice_map.pgm` = รูปแผนที่ (ขาว/ดำ/เทา)
- `practice_map.yaml` = ค่าตั้ง เปิดด้วย Notepad ดูได้ จะเห็น `resolution: 0.05` (1 พิกเซล = 5 ซม.) และ `origin`

**วันจริงเซฟชื่อ `my_map` — ไฟล์นี้คือสิ่งที่ Nav2 ต้องใช้**

---

# ภาค 3 — ตรวจระบบ + หาพิกัด Station (ยังเปิดทุกหน้าต่างไว้)

## 3.1 ตรวจทั้งระบบในคำสั่งเดียว

```bash
python3 ros_check.py
```
Windows: `pixi run python robot\ros_check.py`

✅ ค่าที่ควรเห็น:
```
== rates over 6s ==
  /scan: ... 4.0 Hz
  /odom: ... 50 Hz
== TF ==
  Frame base_link exists with parent odom.
  Frame odom exists with parent map.
  Frame laser exists with parent base_link.
```
ผลถูกเขียนลง `ros_check.txt` ด้วย (ไว้ส่งให้คนอื่นดูตอนติดปัญหา)

## 3.2 ดูว่า node ไหนคุยกับใคร

```bash
ros2 run rqt_graph rqt_graph
```
✅ เห็นแผนผัง: `fake_robot → /scan → slam_toolbox → /map → rviz` และ `teleop → /cmd_vel → fake_robot`
(วันจริงจะเป็น `windows_ydlidar_x2` กับ `windows_kobuki_odometry` แทน `fake_robot`)

## 3.3 หาพิกัด Station (จะต้องใช้กับ Nav2)

1. หน้าต่าง 7: `ros2 topic echo /clicked_point`
2. ใน RViz กดปุ่ม **Publish Point** บนแถบเครื่องมือ → คลิกที่วงแดง Station 1 บนแผนที่
3. หน้าต่าง 7 จะขึ้น:
```
header:
  frame_id: map
point:
  x: 5.0...
  y: 1.3...
```
4. จดไว้ ทำซ้ำกับ Station 2 และ Initial

| จุด | x | y |
|---|---|---|
| Initial | | |
| Station 1 | | |
| Station 2 | | |

> วันจริงทำแบบเดียวกันบนแผนที่ห้องจริง แล้วเอาพิกัดไปใส่ใน state machine

## 3.4 ปุ่มอื่นบนแถบเครื่องมือ (รู้ไว้ ใช้กับ Nav2)

| ปุ่ม | ส่งไป topic | ใช้ตอนไหน |
|---|---|---|
| 2D Pose Estimate | `/initialpose` | บอก Nav2 ว่าหุ่นอยู่ตรงไหนบนแผนที่ (ใช้จริงในภาค 4) |
| 2D Goal Pose | `/goal_pose` | สั่ง Nav2 ให้หุ่นไปจุดนั้น (ใช้จริงในภาค 4) |
| Measure | — | คลิก 2 จุดวัดระยะ ลองวัดระยะ Initial → Station 1 |

---

# ภาค 4 — Nav2: ให้หุ่นวิ่งไป Station เอง

ภาค 2 ได้แผนที่แล้ว ภาคนี้ **ไม่ใช้ SLAM และไม่ใช้ Teleop** อีก
Nav2 จะโหลดแผนที่ที่เซฟไว้ หาตำแหน่งหุ่นบนแผนที่เอง (AMCL) แล้ววางเส้นทางและขับหุ่นไปจุดที่สั่ง

**ปิดหน้าต่าง 4 (SLAM) และ 6 (Teleop) ก่อน** แล้วปิดหุ่นปลอมเปิดใหม่ เพื่อให้หุ่นกลับไปอยู่ที่ Initial

```
แผนที่ .yaml ─► map_server ─/map─► AMCL ──TF map→odom──┐
fake_robot ─/scan, /odom─────────────┘                 ▼
                              planner (วางเส้นทาง /plan) ─► controller ─/cmd_vel─► fake_robot
                                          ▲
           mission_nav2.py หรือปุ่ม 2D Goal Pose ─► bt_navigator (สั่งไปจุดไหน)
```

ไฟล์ที่เพิ่มมา (อยู่ใน `robot/`):

| ไฟล์ | ทำอะไร |
|---|---|
| `nav2_params.yaml` | ค่าตั้ง Nav2 ทั้งหมด (ขนาดหุ่น 0.18 ม., ความเร็ว 0.12 ม./วิ, AMCL เริ่มที่ 0,0) |
| `nav2.launch.py` | เปิด Nav2 ทั้งชุดในหน้าต่างเดียว |
| `lifecycle_up.py` | ตัวสั่งเปิด node ของ Nav2 บน Windows (ตัวมาตรฐานของ Windows ค้าง) — launch เรียกเอง ไม่ต้องรัน |
| `mission_nav2.py` | state machine ภารกิจเต็ม: ไป Station 1 → Station 2 → กลับบ้าน |
| `maps/practice_map.yaml` + `.pgm` | แผนที่ห้องจำลองที่ทำจากหุ่นปลอมไว้แล้ว |

## ผังหน้าต่างภาค 4

| หน้าต่าง | คำสั่ง |
|---|---|
| 1 | หุ่นปลอม (เหมือนข้อ 2.1) |
| 3 | Robot model (เหมือนข้อ 2.2) |
| 4 | **Nav2** (แทน SLAM) |
| 5 | RViz (ตั้งใหม่ข้อ 4.2) |
| 6 | **mission** (แทน Teleop) |

## 4.1 หน้าต่าง 4: Nav2

> **ครั้งแรกครั้งเดียว — เช็คว่ามี Nav2 ไหม**
> Windows: ลงให้แล้ว (27 ต.ค.)
> Ubuntu: `ros2 pkg list | grep nav2_bt_navigator` ถ้าไม่ขึ้นอะไร ให้ลง (ลงแค่ Nav2 ไม่ได้ลง ROS ใหม่):
> `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`

```bash
ros2 launch ./nav2.launch.py
```
Windows: `pixi run ros2 launch robot\nav2.launch.py`

ใช้แผนที่ของตัวเองแทน: ต่อท้าย `map:=` ตามด้วยไฟล์ `.yaml` เช่น
`pixi run ros2 launch robot\nav2.launch.py map:=$HOME\maps\practice_map.yaml`

✅ รอประมาณ 10 วินาที ต้องเห็น (ไล่ลงมา):
```
[map_server]: Read map ... 153 X 121 map @ 0.05 m/cell
[amcl]: Setting pose (...): 0.000 0.000 0.000
[planner_server]: Activating plugin GridBased of type NavfnPlanner
[controller_server]: Activating controller: FollowPath of type regulated_pure_pursuit_controller::...
[bt_navigator]: Activating
```
ช่วงแรกจะมี `Timed out waiting for transform from base_link to map` ขึ้นไม่กี่บรรทัด = ปกติ (รอ AMCL)
❌ ถ้าขึ้นข้อความนี้รัว ๆ ไม่หยุดเกิน 30 วินาที = หน้าต่าง 1 หรือ 3 ไม่ได้เปิด

**แต่ละตัวทำอะไร**

| node | หน้าที่ |
|---|---|
| map_server | อ่านไฟล์แผนที่ ส่งออก `/map` |
| amcl | เทียบ `/scan` กับแผนที่ → รู้ว่าหุ่นอยู่ไหน → ส่ง TF `map → odom` (ทำงานแทน SLAM) |
| planner_server | วางเส้นทางจากหุ่นไปเป้าหมาย (`/plan`) โดยเลี่ยงกำแพง/โต๊ะ |
| controller_server | ขับตามเส้นทาง ส่ง `/cmd_vel` (Regulated Pure Pursuit: หมุนตัวก่อน แล้วเดินตามเส้น) |
| behavior_server | ท่าแก้ตัวเวลาติด: หมุน, ถอย, รอ |
| bt_navigator | สมองสั่งการ: รับเป้าหมาย → วางเส้นทาง → ขับ → ติดก็แก้ตัว |

## 4.2 หน้าต่าง 5: RViz สำหรับ Nav2

```bash
rviz2
```
ตั้งเหมือนข้อ 2.5 (Grid, RobotModel, LaserScan, TF, MarkerArray) แต่ **Fixed Frame = `map`** ตั้งแต่แรก แล้วเพิ่ม:

### ก) Map — แผนที่
| ช่อง | ค่า |
|---|---|
| Topic | `/map` |
| Topic > Durability Policy | **`Transient Local`** |
| Color Scheme | `map` |

### ข) Map — costmap (เขตอันตรายรอบสิ่งกีดขวาง)
**Add** → **Map** อีกอันหนึ่ง แล้วตั้ง:

| ช่อง | ค่า |
|---|---|
| Topic | `/global_costmap/costmap` |
| Topic > Durability Policy | `Transient Local` |
| Color Scheme | **`costmap`** |
| Alpha | `0.5` |

✅ เห็นขอบสีชมพู/ฟ้ารอบกำแพงและโต๊ะ = ระยะที่ Nav2 ไม่ยอมให้ตัวหุ่นเข้าไปใกล้
(อยากดูรอบตัวหุ่นแบบสด ๆ: เพิ่ม Map อีกอัน Topic = `/local_costmap/costmap` สี `costmap`)

### ค) Path — เส้นทางที่วางไว้
**Add** → **Path** → ตั้ง:

| ช่อง | ค่า |
|---|---|
| Topic | `/plan` |
| Color | `0; 200; 0` (เขียว) |
| Line Width | `0.03` |

✅ จะมีเส้นเขียวขึ้นเมื่อสั่งเป้าหมาย

### ง) PoseArray — จุดเดาตำแหน่งของ AMCL (ไม่บังคับ)
**Add** → **PoseArray** → Topic = `/particle_cloud`, Topic > Reliability = `Best Effort`
✅ เห็นลูกศรเล็ก ๆ กองรอบหุ่น ยิ่งกองแน่น = AMCL ยิ่งมั่นใจ

**File > Save Config As** → `nav2.rviz`

## 4.3 ลองสั่งด้วยมือก่อน (ใน RViz)

1. **2D Pose Estimate** (ถ้าหุ่นในแผนที่ไม่ตรงกับเลเซอร์): คลิกที่ตำแหน่งหุ่นบนแผนที่ ค้างแล้วลากไปทางที่หุ่นหัน
   - หุ่นปลอมเริ่มที่ Initial หันขึ้นตรงกับแผนที่แล้ว (AMCL ตั้ง 0,0 ให้เอง) **ปกติไม่ต้องกด**
   - วันจริงต้องกดทุกครั้งที่วางหุ่นไม่ตรงจุดเริ่ม
2. **2D Goal Pose**: คลิกที่วง Station 1 ค้างแล้วลากไปทางที่อยากให้หุ่นหัน (หันเข้าโต๊ะ = ขึ้นบน)
3. ✅ เส้นเขียว `/plan` ขึ้น → หุ่นหมุนหาเส้นแล้วเดินตาม ลอดทางเข้ากลางห้องไปถึงวงแดง
   หน้าต่าง 4 ขึ้น `Reached the goal!`
4. ลองสั่งไป Station 2 และกลับ Initial

## 4.4 หน้าต่าง 6: ภารกิจเต็มแบบอัตโนมัติ

ปิดหุ่นปลอม (หน้าต่าง 1) เปิดใหม่ให้หุ่นกลับไปที่ Initial ก่อน แล้ว (ถ้าหุ่นกระโดดในแผนที่ ให้กด 2D Pose Estimate ที่ Initial)

```bash
python3 mission_nav2.py
```
Windows: `pixi run python robot\mission_nav2.py`

✅ ผลที่ได้ตอนรันทดสอบจริง (หุ่นปลอม, Windows):
```
[basic_navigator]: [INIT] robot is at Initial Location - telling AMCL and waiting for Nav2
[basic_navigator]: [INIT] Nav2 is active
[basic_navigator]: [NAV_STATION1] go to STATION1 x=5.04 y=1.32 yaw=0 deg
[basic_navigator]: [NAV_STATION1] 5.88 m to go, 0 recoveries
   ...
[basic_navigator]: [NAV_STATION1] arrived in 54 s
[basic_navigator]: [DETECT_OBJECT] detect object on desk (fake) ...
[basic_navigator]: [PICK] arm picks object (fake) ...
[basic_navigator]: [NAV_STATION2] arrived in 58 s
[basic_navigator]: [DETECT_STATION] detect place location (fake) ...
[basic_navigator]: [PLACE] arm places object (fake) ...
[basic_navigator]: [RETURN_HOME] arrived in 61 s
[basic_navigator]: Mission finished: succeeded in 185 s
```
หน้าต่าง 4 (Nav2) ขึ้น `Reached the goal!` ทุกครั้งที่ถึง และหน้าต่าง 1 ไม่มี `Bump!` เลย
เส้นทางจริงที่หุ่นวิ่ง (ยาว 17 ม.) ดูได้ที่ [nav2_path_fake_robot.png](nav2_path_fake_robot.png) — หุ่นหยุดห่างจุดเป้าหมายจริง 1–11 ซม.

> ⏱️ ทั้งรอบประมาณ 3 นาที เพราะจำกัดความเร็ว 0.12 ม./วิ เท่าหุ่นจริง

**state machine นี้ทำอะไร** (ชื่อ state เดียวกับ `final_project_state_machine.py`)

| state | ตอนนี้ | วันเดโมต้องเปลี่ยนเป็น |
|---|---|---|
| INIT | บอก AMCL ว่าอยู่ Initial แล้วรอ Nav2 พร้อม | เหมือนเดิม |
| NAV_STATION1 | **Nav2 จริง** ไป (5.04, 1.32) หันเข้าโต๊ะ | แก้พิกัดตามแผนที่ห้องจริง |
| DETECT_OBJECT | รอ 2 วิ (ปลอม) | กล้องหาวัตถุ |
| PICK | รอ 2 วิ (ปลอม) | สั่งแขน SO-100 หยิบ |
| NAV_STATION2 | **Nav2 จริง** ไป (2.85, −2.58) หันเข้าโต๊ะ | แก้พิกัด |
| DETECT_STATION | รอ 2 วิ (ปลอม) | กล้องหาจุดวาง |
| PLACE | รอ 2 วิ (ปลอม) | สั่งแขนวาง |
| RETURN_HOME | **Nav2 จริง** กลับ (0, 0) | เหมือนเดิม |

ถ้า Nav2 ไปไม่ถึง (ติด/หาเส้นทางไม่ได้) state นั้นจะลองใหม่เอง สูงสุด 3 ครั้ง แล้วจบด้วย `aborted`

**แก้พิกัด Station:** เปิด `mission_nav2.py` แก้ตรง `GOALS = {...}` เป็น `(x, y, มุมหัน)`
ค่า x, y ได้จาก Publish Point (ข้อ 3.3) มุมเป็นเรเดียน: หันขึ้น (แกน X) = `0`, หันซ้าย = `math.pi / 2`, หันขวา = `-math.pi / 2`, หันลง = `math.pi`

---

## ปิดระบบ

กด `Ctrl+C` ตามลำดับ: **6 Teleop → 1 หุ่นปลอม → 4 SLAM → 3 Robot model → ปิด RViz**
(วันจริง: ปิด Teleop กับ Kobuki ก่อนเสมอ หุ่นจะหยุดล้อก่อนปิด USB)

---

## ปัญหาที่เจอบ่อย

| อาการ | สาเหตุ | แก้ |
|---|---|---|
| `ros2: command not found` (Ubuntu) | ยังไม่ได้ source | `source /opt/ros/humble/setup.bash` |
| RobotModel ไม่ขึ้นหุ่น | Durability ไม่ใช่ Transient Local | ตั้ง Description Topic > Durability = `Transient Local` |
| RobotModel ขึ้น `No transform from [base_link]` | Fixed Frame ผิด หรือยังไม่เปิดหุ่นปลอม/โมเดล | เช็คหน้าต่าง 1, 3 และ Fixed Frame |
| LaserScan ไม่มีจุดแดง | Reliability ไม่ตรง | ตั้ง `/scan` > Reliability = `Best Effort` |
| `Fixed Frame [map] does not exist` | SLAM ยังไม่เปิด/ยังไม่ได้รับ scan | ใช้ `odom` ไปก่อน เช็คหน้าต่าง 4 |
| Map: `No map received` | SLAM ไม่ทำงาน หรือ Durability ผิด | ตั้ง `/map` > Durability = `Transient Local` |
| กดปุ่มแล้วหุ่นไม่เดิน | หน้าต่าง Teleop ไม่ได้ถูกเลือก / คีย์บอร์ดไทย | คลิกหน้าต่าง 6 + เปลี่ยนเป็น EN |
| หุ่นไม่เดินแม้กด `i` | ชนอยู่ (`Bump!`) | กด `,` ถอย หรือ `j`/`l` หมุนออก |
| `ros2 node list` ว่าง/ค้าง | daemon ค้าง | `ros2 daemon stop` แล้ว `ros2 daemon start` |
| แผนที่ซ้อนเบลอ | ขับ/หมุนเร็วไป | หมุนช้า ๆ กด `z` ลดความเร็ว |
| ภาพ RViz ค้างหลังรีสตาร์ท node | เวลาเก่าค้าง | กด **Reset** มุมล่างขวา |
| Nav2: `Timed out waiting for transform ... to map` ไม่หยุด | AMCL ยังไม่ได้ `/scan` หรือไม่มี TF `odom → base_link` | เช็คหน้าต่าง 1 และ 3 ว่าเปิดอยู่ |
| Nav2: เปิด SLAM ค้างไว้ด้วย | `map → odom` มา 2 แหล่ง (SLAM + AMCL) หุ่นกระโดดไปมา | ภาค 4 ต้องปิด SLAM |
| สั่ง 2D Goal Pose แล้วไม่มีเส้นเขียว | จุดเป้าหมายอยู่ในกำแพง/ชิดโต๊ะเกินไป | คลิกให้ห่างสิ่งกีดขวาง > 25 ซม. |
| เลเซอร์ไม่ทับกำแพงบนแผนที่ | AMCL ไม่รู้ว่าหุ่นอยู่ไหน | กด **2D Pose Estimate** ที่ตำแหน่งหุ่นจริง |
| หุ่นหมุนวนที่เดิม / `number_of_recoveries` เพิ่มเรื่อย ๆ | ติด ทางแคบ | ดู costmap ว่าปิดทางไหม ลด `inflation_radius` ใน `nav2_params.yaml` |
| `mission_nav2.py` ค้างที่ `Waiting for amcl_pose` | Nav2 (หน้าต่าง 4) ยังไม่เปิดหรือยังไม่พร้อม | เปิด Nav2 ก่อน รอ `bt_navigator: Activating` |

---

## เช็คลิสต์ซ้อมผ่าน

- [ ] ภาค 1: เห็นหุ่น CAD (Kobuki + X2 + แขนส้มด้านหน้า) + แกน `laser` หมุน -90°
- [ ] ภาค 1: `tf2_echo base_link laser` ได้ z = 0.200
- [ ] ภาค 2: `/scan` ~4 Hz, `/odom` ~50 Hz
- [ ] ภาค 2: ตั้ง RViz เองครบ 8 อย่าง (Grid, RobotModel, LaserScan, Odometry, Map, TF, MarkerArray, View) และเซฟ `.rviz`
- [ ] ภาค 2: ขับวนครบห้องกลับจุดเริ่ม แผนที่ไม่ซ้อน
- [ ] ภาค 2: เซฟแผนที่ได้ `.pgm` + `.yaml`
- [ ] ภาค 3: `ros_check.py` TF ครบ `map → odom → base_link → laser`
- [ ] ภาค 3: จดพิกัด Station 1, 2, Initial ด้วย Publish Point
- [ ] ภาค 4: Nav2 เปิดครบถึง `bt_navigator: Activating`
- [ ] ภาค 4: RViz เห็น costmap + เส้นเขียว `/plan`
- [ ] ภาค 4: กด 2D Goal Pose แล้วหุ่นวิ่งไป Station 1 เอง
- [ ] ภาค 4: `mission_nav2.py` จบด้วย `Mission finished: succeeded`

ผ่านครบแล้ว วันได้หุ่นจริงแค่เปลี่ยนหน้าต่าง 1 จาก `fake_robot.py` เป็น LiDAR + Kobuki (ดู [RUN_GUIDE_TH.md](RUN_GUIDE_TH.md))
