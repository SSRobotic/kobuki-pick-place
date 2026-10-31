# วันจริง 1 พ.ย. — หุ่นจริง รวดเดียวจบ

ทำจากบนลงล่างทีละขั้น ไม่มีซ้อม ไม่มีหุ่นปลอม · ทุกขั้นมี **✅ ผ่านเมื่อ** ถ้าไม่ผ่าน อย่าไปขั้นต่อไป ดู **❌** ใต้ขั้นนั้น หรือตาราง "ถ้าติด" ท้ายไฟล์
รูปประกอบและคำอธิบายละเอียดของแต่ละขั้นอยู่ใน [DEMO_DAY_TH.md](DEMO_DAY_TH.md) (เลขขั้นตรงกัน)

**กติกาพิมพ์คำสั่ง (Windows / PowerShell):**
- ทุกหน้าต่างใหม่ เริ่มด้วย `cd` เข้าโฟลเดอร์โปรเจกต์ก่อนเสมอ:
  - โน้ตบุ๊กหลักของทีม: `cd D:\ROS2_Windows_Humble`
  - เครื่องที่ clone จาก GitHub: `cd ~\kobuki-pick-place`
- **1 คำสั่งที่รันค้าง = 1 หน้าต่าง** อย่าปิด หยุดด้วย `Ctrl+C` ในหน้าต่างนั้น
- Ubuntu: `cd ~/kobuki-pick-place/robot` แล้วตัด `pixi run` และ `robot\` ออก ใช้ `python3` แทน `python`

**ผังหน้าต่างทั้งวัน** (เลขเดียวกันตลอดไฟล์ ตั้งชื่อแท็บ terminal ตามเลขจะไม่งง):

| หน้าต่าง | อะไร | เปิดตอน |
|---|---|---|
| 1 | LiDAR | ขั้น 4 |
| 2 | Kobuki | ขั้น 4 |
| 3 | โมเดลหุ่น | ขั้น 4 |
| 4 | ไดรเวอร์แขน | ขั้น 5 |
| 5 | กล้อง | ขั้น 6 |
| 6 | ตัวหาวัตถุ | ขั้น 6 |
| 7 | SLAM → (ขั้น 8) Nav2 | ขั้น 7 |
| 8 | RViz | ขั้น 7 |
| 9 | คีย์บอร์ด → (ขั้น 8) ภารกิจ | ขั้น 7 |
| 10 | เสียงหุ่น | ขั้น 9 |
| 11 | **สำรอง** คำสั่งสั้น ๆ ที่รันแล้วจบเอง | ทั้งวัน |

---

## ขั้น 0 — โน้ตบุ๊ก + ของ (ก่อน 9 โมง)

**ดึงโค้ดล่าสุด** (หน้าต่าง 11):
```powershell
cd D:\ROS2_Windows_Humble
git pull
pixi run ros2 topic list
```
✅ `git pull` ไม่มี error · `topic list` ขึ้น `/parameter_events` กับ `/rosout` (ครั้งแรกหลัง pull อาจโหลดแพ็กเกจเพิ่มสักพัก)
❌ `fatal: not a git repository` = ยังไม่ได้ `cd` เข้าโฟลเดอร์โปรเจกต์ (คำสั่ง git ต้องรันในโฟลเดอร์ที่มีโฟลเดอร์ `.git`)
❌ โน้ตบุ๊กหลักมี commit ที่ยังไม่ได้ส่งขึ้น GitHub: `git push origin main` (ในโฟลเดอร์เดียวกัน) ก่อน แล้วเพื่อนค่อย `git pull`

**ของ:**
- [ ] โน้ตบุ๊ก + สายชาร์จ · Kobuki ชาร์จเต็ม + สาย USB
- [ ] LiDAR X2 + กล่องแปลง USB สีดำ · USB hub มีไฟเลี้ยง (4 พอร์ต)
- [ ] แขน SO-100 + บอร์ดเซอร์โว + สาย USB · แบต PINSEN ชาร์จเต็ม + สาย DC
- [ ] กล้อง Xtion + **เว็บแคมสำรอง**
- [ ] ตลับเมตร ไม้บรรทัด เทปกาวกระดาษ ไขควง สายรัด
- [ ] ลูกบาศก์ 4 ซม. สีสด + แผ่นจุดวาง ~10 ซม. (คนละสีกับลูกบาศก์และพื้น)

---

## ขั้น 1 — ประกอบ (45 นาที)

| ชิ้น | วางตรงไหน |
|---|---|
| หน้า / หลัง | **หน้า = กันชน** · **หลัง = แผงพอร์ต USB** |
| แขน SO-100 | แผ่นขาวชั้นกลาง ค่อนไปหลัง **ฐานหันออกท้ายหุ่น** |
| กล้อง Xtion | ชั้นล่าง ท้ายหุ่น **หันออกท้าย** ตั้งตรง |
| LiDAR | ขายึดขาว ชั้นกลาง ค่อนไปหน้า เยื้องขวา ด้านมอเตอร์หันไปหน้า |
| แบต PINSEN | ชั้นกลาง หลังซ้าย ปุ่มแดงหันหลัง |

- [ ] สายทุกเส้นไม่ผ่านระดับเลเซอร์ (~25 ซม. จากพื้น) และไม่บังกล้อง
- [ ] ขยับแขนด้วยมือ (ยังไม่เสียบไฟ) ทุกท่า สายไม่ตึง

✅ ผ่านเมื่อ: หน้าตาเหมือนรูปใน [DEMO_DAY_TH.md ขั้น 1](DEMO_DAY_TH.md)

---

## ขั้น 2 — วัด 3 จุด แก้ `robot/kobuki_x2.urdf` (15 นาที)

จุดอ้างอิง = กลางระหว่างล้อ 2 ข้าง · x = ไปหน้า (หลัง = ติดลบ) · y = ไปซ้าย (ขวา = ติดลบ) · z = สูงจากพื้น · **หน่วยเมตร**

| วัดอะไร | แก้ `xyz` ใน | ค่าเดิม (ประมาณจากรูป) |
|---|---|---|
| กลางฝาหมุน LiDAR (z = ใต้ขอบบนฝาน้ำเงิน 8 มม.) | `<joint name="laser_joint">` (**ห้ามแก้ rpy**) | `0.075 -0.035 0.247` |
| กลางเซอร์โวตัวล่างของแขน (z = ผิวบนแผ่นขาว) | `<joint name="so100_mount_joint">` | `-0.055 -0.03 0.208` |
| กลางตัวกล้อง | `<joint name="camera_joint">` (มุมกล้องปล่อยไว้ ขั้น 6 หาให้) | `-0.135 0 0.1775` |

✅ ผ่านเมื่อ: เซฟไฟล์แล้ว (ขั้น 4 จะเช็คค่าให้)

---

## ขั้น 3 — เสียบสาย (5 นาที)

1. เสียบ **แบต PINSEN เข้าบอร์ดเซอร์โวก่อน** แล้วค่อยเสียบ USB บอร์ด
2. Kobuki, LiDAR, บอร์ดแขน เข้า hub · **Xtion เสียบพอร์ตโน้ตบุ๊กตรง ๆ**
3. เปิดสวิตช์ Kobuki

✅ ผ่านเมื่อ: Device Manager → Ports เห็น `USB Serial Port` (Kobuki), `Silicon Labs CP210x` (LiDAR), `USB-SERIAL CH340/CH343` (แขน) และมี `PrimeSense`/`Xtion` ในรายการอุปกรณ์

---

## ขั้น 4 — หน้าต่าง 1–3: LiDAR + Kobuki + โมเดล (10 นาที)

| หน้าต่าง | คำสั่ง | ✅ ต้องเห็น |
|---|---|---|
| 1 | `pixi run python robot\lidar_ros2.py --mirror` | `Dropping returns within 0.24 m ...` แล้ว `First complete scan` |
| 2 | `pixi run python robot\kobuki_ros2.py --enable-motion` | `First odometry` แล้ว `Motion ready` |
| 3 | `pixi run ros2 launch robot\robot_model.launch.py arm:=none` | รายการ `got segment` ไม่มีบรรทัดแดง |

หน้าต่าง 11 เช็คตำแหน่ง LiDAR ที่วัด:
```powershell
pixi run ros2 run tf2_ros tf2_echo base_link laser
```
✅ `Translation:` = ค่าที่ใส่ในขั้น 2 → `Ctrl+C`

- `--mirror` ต้องใส่เสมอ · Kobuki หยุดล้อเองถ้าไม่มีคำสั่ง 0.4 วินาที / ชนกันชน / ล้อลอย
- ❌ หาพอร์ตไม่เจอ: ใส่เอง `... lidar_ros2.py --mirror --port COM11` / `... kobuki_ros2.py --enable-motion --port COM10` (เลขจาก Device Manager)
- ❌ แก้ URDF ทีหลัง: `Ctrl+C` หน้าต่าง 3 แล้วเปิดใหม่ทุกครั้ง

---

## ขั้น 5 — แขน SO-100 (25 นาที)

**5.1 คาลิเบรต** (ครั้งเดียว ถ้าในโฟลเดอร์ `robot` ยังไม่มีไฟล์ `so100_calibration.yaml`) — หน้าต่าง 11:
```powershell
pixi run python robot\so100_calibrate.py
```
**ประคองแขนก่อนกด Enter** (แรงบิดหาย แขนจะตก) แล้วจัดท่าด้วยมือตามที่โปรแกรมบอก 3 ท่า (รูปท่าใน [DEMO_DAY_TH.md ขั้น 5.2](DEMO_DAY_TH.md)):

| ท่า | จัดยังไง |
|---|---|
| 1 REST | ฐานหันออกท้าย พับสุด มือห้อยออกท้าย ปากหุบสนิท |
| 2 HOME | แขนบนตั้งตรง แขนล่างยื่นออกท้ายขนานพื้น มือชี้ลง ปากอ้า ~60° |
| 3 PAN_90 | หมุนฐาน 90° ให้แขนชี้ไปทาง **ขวา** ของหุ่น |

✅ `saved ...so100_calibration.yaml` ไม่มี `WARNING` · ❌ มี `WARNING ... moved .. deg ... expected 90 deg` = จัดท่าไม่ตรง → รันใหม่

**5.2 ไดรเวอร์** — จัดแขนท่า REST แล้ว หน้าต่าง 4:
```powershell
pixi run python robot\so100_driver.py
```
✅ `SO-100 ready, supply 7.4 V, torque on, holding: ...` (lift ≈ −3.3, elbow ≈ +3.1) · ❌ `servo board not found` → `... so100_driver.py --ros-args -p port:=COM5` (เลขจาก Device Manager)

**5.3 ลองสั่ง** — หน้าต่าง 11 ทีละบรรทัด **มือหนึ่งพร้อมดึงปลั๊กแบตแขน** ถ้าวิ่งผิดทาง:
```powershell
pixi run python robot\arm_control.py carry
pixi run python robot\arm_control.py open
pixi run python robot\arm_control.py close
pixi run python robot\arm_control.py reach -0.27 0.0 0.10
pixi run python robot\arm_control.py reach -0.27 0.0 0.03
pixi run python robot\arm_control.py rest
```
✅ ทุกบรรทัดจบด้วย `done` · `reach ... 0.03` วัดด้วยไม้บรรทัด: กลางปากคีบอยู่หลังกลางหุ่น 27 ซม. สูงจากพื้น 3 ซม. คลาดไม่เกิน 1 ซม.

**ลองหยิบด้วยมือ:** วางลูกบาศก์หลังกลางหุ่น 27 ซม. → `open` → `reach -0.27 0.0 0.02` → `close` → `reach -0.27 0.0 0.10`
✅ ลูกบาศก์ลอยขึ้น → `rest`

- ❌ คลาดเกิน 1 ซม. ไปทางเดียวกันทุกครั้ง = วัด `so100_mount_joint` ใหม่ (ขั้น 2) แล้วเปิดหน้าต่าง 3 และ 4 ใหม่
- ❌ ข้อไหนหมุนกลับทาง = คาลิเบรตใหม่ (5.1)

---

## ขั้น 6 — กล้อง (25 นาที)

**6.1 หน้าต่าง 5: กล้อง**
```powershell
pixi run python robot\camera_node.py
```
✅ `openni camera 640x480, fx=577 -> /camera/color/image_raw`
- ❌ หา `OpenNI2.dll` ไม่เจอ: `... camera_node.py --ros-args -p openni_path:="C:\Program Files\OpenNI2\Redist"`
- ❌ **15 นาทีแล้ว Xtion ยังไม่ได้ → ใช้เว็บแคมเลย:** ติดเว็บแคมแทนที่ Xtion (ท้ายหุ่น หันออกท้าย) วัดกลางเลนส์ใส่ `camera_joint` ใหม่ เปิดหน้าต่าง 3 ใหม่ แล้ว
  ```powershell
  pixi run python robot\camera_node.py --list
  pixi run python robot\camera_node.py --ros-args -p backend:=opencv -p device:=1 -p hfov_deg:=60
  ```
  (`device` = เลขที่ `--list` บอกว่า `OK` · `hfov_deg` = มุมมองแนวนอนตามสเปกเว็บแคม)

**6.2 หาสี** — วางลูกบาศก์และแผ่นจุดวางให้อยู่ในภาพกล้อง **ใต้ไฟห้องเดโม** แล้วหน้าต่าง 11:
```powershell
pixi run python robot\hsv_tuner.py
```
กด `1` คลิกลูกบาศก์ → ภาพขวาเป็นก้อนขาวเฉพาะลูกบาศก์ · กด `2` คลิกแผ่นจุดวาง · กด `q`
✅ ได้บรรทัด `-p cube_hsv_low:=[...] -p cube_hsv_high:=[...] -p marker_hsv_low:=[...] -p marker_hsv_high:=[...]` → **คัดลอกเก็บไว้**
(ก้อนขาวมีรู = ลดแถบ S min / V min · มีจุดขาวที่พื้น = เพิ่ม S min)

**6.3 หน้าต่าง 6: ตัวหาวัตถุ** (วางบรรทัดจาก 6.2 ต่อท้าย):
```powershell
pixi run python robot\cube_detector.py --ros-args -p cube_hsv_low:=[...] -p cube_hsv_high:=[...] -p marker_hsv_low:=[...] -p marker_hsv_high:=[...]
```
✅ `Cube detector on /camera/color/image_raw, objects on a surface 0.00 m high`

**6.4 เช็คมุมกล้อง** — วางลูกบาศก์ **หลังกลางหุ่น 0.65 ม. ตรงแนวกลาง** (ตลับเมตร + เทปมาร์ค) แล้วหน้าต่าง 11:
```powershell
pixi run python robot\camera_check.py -0.65 0.0 --write
```
→ `Ctrl+C` หน้าต่าง 3 แล้วเปิดใหม่ → รันอีกรอบ **ไม่ใส่** `--write`:
```powershell
pixi run python robot\camera_check.py -0.65 0.0
```
✅ รอบสอง `off by ...` ไม่เกิน 1 ซม. / `good enough (< 1 cm)`

---

## ขั้น 7 — ทำแผนที่ห้องจริง (20 นาที)

**วางหุ่นที่ Initial Location หันทางเดียวกับตอนเริ่มเดโม** (มาร์คเทปที่พื้นไว้ด้วย จุดนี้ = (0, 0) ของแผนที่ = บ้าน)

| หน้าต่าง | คำสั่ง | ✅ |
|---|---|---|
| 7 | `pixi run ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file robot\slam_params.yaml` | ไม่มีแดงเรื่อง TF ค้าง |
| 8 | `pixi run rviz2 -d robot\mapping.rviz` (**เปิดหลังหน้าต่าง 7**) | หุ่น + จุดแดงเลเซอร์ + แผนที่เทาเริ่มขึ้น |
| 9 | `pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.35` | ผังปุ่ม |

ขับ (คลิกหน้าต่าง 9 + **แป้นอังกฤษ** + กดค้าง): `i` หน้า · `,` ถอย · `j`/`l` หมุน · `k` หยุด
ขับช้า ๆ วนให้ครบทุกห้อง ผ่าน Station 1 และ Station 2 แล้ว **กลับมาที่ Initial**

✅ ใน RViz กำแพงเป็นเส้นเดียวคม ไม่ซ้อน 2 ชั้น → หน้าต่าง 11 เซฟ:
```powershell
mkdir $HOME\maps -Force; pixi run ros2 run nav2_map_server map_saver_cli -f $HOME\maps\my_map --ros-args -p save_map_timeout:=10.0
```
✅ `Map saved successfully`
❌ `Failed to spin map subscription` = หน้าต่าง 7 ต้องยังเปิดอยู่ แล้วรันใหม่

แล้ว **ปิด** หน้าต่าง 9 → 7 → ปิด RViz (หน้าต่าง 1–6 เปิดค้างไว้)

---

## ขั้น 8 — Nav2 + ใส่ตำแหน่งวัตถุ (20 นาที)

**ยกหุ่นกลับ Initial หันทางเดิม** แล้ว:

| หน้าต่าง | คำสั่ง | ✅ |
|---|---|---|
| 7 | `pixi run ros2 launch robot\nav2.launch.py map:=$HOME\maps\my_map.yaml` | รอ ~10 วินาที มี `[bt_navigator]: Activating` |
| 8 | `pixi run rviz2 -d robot\mission.rviz` | แผนที่ + ขอบสีรอบกำแพง + จุดแดงเลเซอร์ทับกำแพง |

❌ จุดแดงไม่ทับกำแพง: กด **2D Pose Estimate** คลิกที่ตำแหน่งหุ่นจริง ลากไปทางที่หุ่นหัน

**วางลูกบาศก์ที่ Station 1 และแผ่นจุดวางที่ Station 2 แล้วมาร์คเทปรอบ ๆ** → หน้าต่าง 11:
```powershell
pixi run ros2 topic echo /clicked_point
```
ใน RViz กด **Publish Point** คลิกตรงลูกบาศก์ → จด x, y · กด Publish Point อีกครั้ง คลิกตรงแผ่นจุดวาง → จด x, y → `Ctrl+C`

แก้ `robot/mission_nav2.py` ตรงนี้ แล้วเซฟ:
```python
STATIONS = {
    "STATION1": (5.30, 1.32, 0.0),            # (x ลูกบาศก์, y ลูกบาศก์, ทิศที่หุ่นถอยเข้าหา)
    "STATION2": (2.85, -2.90, -math.pi / 2),  # (x แผ่นจุดวาง, y แผ่นจุดวาง, ทิศ)
}
```
- x, y = ตำแหน่ง **วัตถุ** ที่จดมา โปรแกรมหาจุดจอดเอง (ห่าง 0.65 ม. หันท้ายเข้าหา แล้วถอยเข้าไป)
- ทิศ = จากจุดจอดมองไปหาวัตถุ: `0` = วัตถุอยู่ทาง +x · `math.pi / 2` = ทาง +y · `-math.pi / 2` = ทาง −y · `math.pi` = ทาง −x (ดูแกนแดง/เขียวของ `map` ใน RViz)
- เลือกทิศที่ **พื้นหน้าวัตถุโล่งอย่างน้อย 0.9 ม.** (Kobuki ไม่มีกันชนด้านหลัง)

**ซ้อมวิ่งอย่างเดียว** (หุ่นอยู่ Initial) — หน้าต่าง 9:
```powershell
pixi run python robot\mission_nav2.py --nav-only
```
✅ จอดหน้า Station 1 **ท้ายหันเข้าวัตถุ** → Station 2 → กลับบ้าน · จบด้วย `Mission finished: succeeded`
❌ จอดผิดที่/ผิดทิศ = แก้ `STATIONS` แล้วรันใหม่ (ยกหุ่นกลับ Initial ก่อนทุกครั้ง)

---

## ขั้น 9 — ภารกิจเต็ม 🎯

**หน้าต่าง 10: เสียงหุ่น** (เปิดเสียงโน้ตบุ๊กดังสุด):
```powershell
pixi run python robot\announcer.py
```
✅ ได้ยิน "หุ่นยนต์พร้อมแล้ว" (ไม่ได้ยิน = ข้ามได้ ภารกิจไม่พึ่งเสียง)

วางลูกบาศก์ + แผ่นจุดวางตรงเทป · หุ่นที่ Initial หันทางเดิม · แขนท่า REST → หน้าต่าง 9:
```powershell
pixi run python robot\mission_nav2.py
```
✅ ประมาณ 4 นาที หุ่นพูด: "ถึงสถานีที่หนึ่งแล้ว" → "เจอวัตถุแล้ว" → "หยิบวัตถุสำเร็จ" → "ถึงสถานีที่สองแล้ว" → "เจอจุดวางแล้ว" → "วางวัตถุเรียบร้อย" → "กลับถึงจุดเริ่มต้นแล้ว"
หน้าต่าง 9 จบด้วย `Mission finished: succeeded`

**รอบต่อไป:** วางลูกบาศก์กลับเทป ยกหุ่นกลับ Initial แล้วรันหน้าต่าง 9 ซ้ำ (ไม่ต้องปิดหน้าต่างอื่น โปรแกรมบอกตำแหน่งเริ่มให้ Nav2 เอง)

---

## ขั้น 10 — รอบกรรมการ

- [ ] หน้าต่าง 1–8 และ 10 เปิดอยู่ ไม่มีบรรทัดแดงค้าง · ภารกิจเต็ม `succeeded` มาแล้วอย่างน้อย 1 รอบ
- [ ] แบต Kobuki / แบตแขน / โน้ตบุ๊ก ยังพอ (ชาร์จระหว่างรอคิว)
- [ ] ลูกบาศก์ + แผ่นจุดวางตรงเทป · หุ่นที่ Initial หันทางเดิม · แขนท่า REST
- [ ] ไฟห้องเหมือนตอนหาสี (ไม่เหมือน = ทำขั้น 6.2–6.3 ใหม่)
- [ ] กด Enter รันหน้าต่าง 9 แล้ว **ห้ามแตะหุ่น ห้ามชี้ ห้ามยกมือบอกทาง** (คนช่วยหยิบ/วาง −40 · ช่วยนำทาง/ชี้วัตถุ −5)

**หยุดฉุกเฉิน:** `Ctrl+C` หน้าต่าง 9 (ภารกิจ) แล้วหน้าต่าง 2 (ล้อหยุด) · แขนค้างท่าเดิม

---

## ขั้น 11 — หลังเดโม: เก็บค่าที่แก้วันนี้ขึ้น GitHub

ค่าที่วัด/แก้วันนี้ (`kobuki_x2.urdf`, `so100_calibration.yaml`, `STATIONS` ใน `mission_nav2.py`) เก็บไว้ใช้ต่อ — หน้าต่าง 11:
```powershell
cd D:\ROS2_Windows_Humble
git add -A
git commit -m "Demo day: measured URDF, arm calibration, real station positions"
git push origin main
```

---

## ถ้าติด

| อาการ | ทำอะไรก่อน |
|---|---|
| `fatal: not a git repository` | ยังไม่ได้ `cd D:\ROS2_Windows_Humble` |
| LiDAR / Kobuki หาพอร์ตไม่เจอ | ใส่ `--port COMx` ตาม Device Manager · ปิดหน้าต่างเก่าที่ยังจับพอร์ตอยู่ |
| RViz ไม่มีจุดแดง | หน้าต่าง 1 ยังเปิดไหม · ใน RViz `/scan` ต้อง Reliability = Best Effort |
| จุดแดงบนตัวหุ่น | หน้าต่าง 1: `... --mirror --self-radius 0.28` |
| `servos not answering` | แบตเข้าบอร์ดไหม · สายเซอร์โวหลวมตรงข้อที่บอก |
| แขนกระตุก / `supply dropped` | แบตแขนอ่อน หรือปลั๊ก DC หลวม |
| `camera gives no picture` | ถอดเสียบ Xtion (พอร์ตตรง) → 15 นาทีไม่ได้ ใช้เว็บแคม (6.1) |
| แผนที่ซ้อนเบลอ | ขับช้าลง เริ่มใหม่: ปิดหน้าต่าง 7, 8 แล้วทำขั้น 7 ใหม่ |
| Nav2 `Timed out waiting for transform` ไม่หยุด | หน้าต่าง 1–3 เปิดไหม · SLAM (ขั้น 7) ต้องปิดแล้ว |
| หุ่นหมุนวน / ไปไม่ถึง | 2D Pose Estimate ใหม่ · ดู costmap รอบตัวหุ่นมีจุดดำไหม (มี = เพิ่ม `--self-radius`) |
| `[DETECT_OBJECT] no cube in the camera image` | หน้าต่าง 5, 6 เปิดไหม · ช่อง Camera ใน RViz เห็นลูกบาศก์ไหม → ไม่เห็น = แก้ `STATIONS` · เห็นแต่ไม่มีกรอบ = หาสีใหม่ (6.2) |
| `target ... out of reach` | เช็คมุมกล้องใหม่ (6.4) |
| หุ่นถอยแล้วชน | หลังวัตถุไม่โล่ง → เปลี่ยนทิศใน `STATIONS` |
| `blocked while driving straight` | ล้อติด / ชนอยู่ / หน้าต่าง 2 ไม่มี `--enable-motion` |
