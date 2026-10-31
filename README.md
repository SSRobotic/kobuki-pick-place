# Kobuki Pick & Place — Final Project 01208567

หุ่นยนต์ Kobuki + YDLIDAR X2 + แขน SO-100 + กล้อง ASUS Xtion วิ่งจาก Initial ไป Station 1 หยิบวัตถุบนพื้น ไปวางที่ Station 2 แล้วกลับบ้านเอง
แขนกับกล้องติดด้านหลังหุ่น (ตามที่ทีมประกอบ) หุ่นจึงจอดหน้าวัตถุแล้วถอยเข้าไปหยิบ
ทั้งหมดซ้อมได้บนคอมเครื่องเดียว **ไม่ต้องมีหุ่นจริง** เพราะมีหุ่นปลอมที่จำลองห้องสอบตามผังในโจทย์

- **🔴 วันจริง 1 พ.ย. หุ่นจริง รวดเดียวจบ (git pull → ประกอบ → แขน → กล้อง → แผนที่ → ภารกิจ):** [`docs/REAL_DAY_TH.md`](docs/REAL_DAY_TH.md)
- **เว็บคู่มือ (โมเดลหุ่น 3D + จำลองภารกิจ):** https://ssrobotic.github.io/kobuki-pick-place/handbook/
- **วันซ้อม ฉบับเดียวจบ (เปิดเครื่อง → ทำแผนที่ → Nav2 → กล้อง → แขน → ภารกิจเต็ม):** [`docs/PRACTICE_DAY_TH.md`](docs/PRACTICE_DAY_TH.md)
- **คู่มือรันคำสั่งทีละขั้น:** [`docs/RUN_GUIDE_TH.md`](docs/RUN_GUIDE_TH.md)
- **คู่มือซ้อมแบบไม่มีหุ่น + ตั้งค่า RViz ทุกช่อง:** [`docs/PRACTICE_NO_ROBOT_TH.md`](docs/PRACTICE_NO_ROBOT_TH.md)
- **วันเดโม: ประกอบ วัด คาลิเบรตแขน ตั้งกล้อง วิ่งภารกิจ (มีรูปเทียบทุกขั้น):** [`docs/DEMO_DAY_TH.md`](docs/DEMO_DAY_TH.md)

## ตอนนี้ทำอะไรได้แล้วบ้าง

| ส่วน | สถานะ |
|---|---|
| หุ่นปลอม (`/scan`, `/odom`, TF) ในห้องจำลอง | ✅ ใช้ได้ |
| ทำแผนที่ด้วย SLAM | ✅ ใช้ได้ (แผนที่ห้องจำลองอยู่ใน `robot/maps/`) |
| Nav2: วิ่งไป Station 1 → Station 2 → กลับบ้านเอง | ✅ ผ่าน 3/3 รอบบน Windows |
| โมเดลหุ่นใน RViz ตามที่ทีมประกอบ (CAD จริง + ขนาดจากรูปถ่าย) | ✅ ใช้ได้ · วันจริงวัดตำแหน่งแล้วแก้ URDF |
| ตรวจจับลูกบาศก์ + จุดวางบนพื้นด้วยกล้อง (Xtion ปลอมในซิม) | ✅ ใช้ได้ในซิม · วันจริงหาสีด้วย `hsv_tuner.py` |
| ไดรเวอร์กล้องจริง `camera_node.py` (Xtion / เว็บแคมสำรอง) + `camera_check.py` | ✅ เว็บแคม/ไฟล์ภาพทดสอบแล้ว · Xtion ยังไม่ได้ลองกับของจริง |
| แขน SO-100 หยิบ/วาง (IK จากไฟล์ CAD) | ✅ ใช้ได้ในซิม (ภารกิจเต็มผ่าน 28 ต.ค.) |
| ไดรเวอร์เซอร์โวจริง `so100_driver.py` + `so100_calibrate.py` | ✅ ทดสอบกับเซอร์โวจำลองแล้ว · ยังไม่ได้ลองกับแขนจริง |
| ไดรเวอร์หุ่นจริง (Kobuki, LiDAR) | ✅ ทดสอบกับหุ่นจริงแล้ว 11 และ 18 ต.ค. |

---

## 1. เอาไฟล์มาลงเครื่อง (ทำครั้งเดียว)

ต้องมี **git** ก่อน ([ดาวน์โหลดสำหรับ Windows](https://git-scm.com/download/win) / Ubuntu: `sudo apt install git`)

```bash
git clone https://github.com/SSRobotic/kobuki-pick-place.git
```
จะได้โฟลเดอร์ `kobuki-pick-place` ในโฟลเดอร์ที่เปิด terminal อยู่ (ปกติคือโฟลเดอร์ผู้ใช้ `~`)

**อัปเดตเมื่อเพื่อนแก้ไฟล์:** เข้าโฟลเดอร์แล้วสั่ง `git pull`

## 2A. ติดตั้งบน Windows (ใช้ pixi ไม่ต้องลง ROS เอง)

pixi จะโหลด ROS 2 Humble + SLAM + Nav2 + RViz ชุดเดียวกับที่ทีมใช้ ตามไฟล์ `pixi.lock` (ทุกเครื่องได้เวอร์ชันตรงกัน)

1. เปิด **PowerShell** แล้วลง pixi:
   ```powershell
   powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
   ```
2. **ปิด PowerShell แล้วเปิดใหม่** (ให้รู้จักคำสั่ง `pixi`)
3. ติดตั้ง ROS ลงในโฟลเดอร์โปรเจกต์:
   ```powershell
   cd ~\kobuki-pick-place
   pixi install
   ```
   โหลดประมาณ 0.9 GB ใช้พื้นที่ประมาณ 5.5 GB รอ 10–30 นาทีแล้วแต่เน็ต
4. ทดสอบ:
   ```powershell
   pixi run ros2 topic list
   ```
   ต้องเห็น `/parameter_events` และ `/rosout`

ทุกคำสั่งบน Windows: **อยู่ในโฟลเดอร์ `kobuki-pick-place`** แล้วขึ้นต้นด้วย `pixi run` เช่น `pixi run python robot\fake_robot.py`

## 2B. ติดตั้งบน Ubuntu 22.04 (ใช้ apt)

ถ้ายังไม่มี ROS 2 Humble ให้ลงตาม [คู่มือทางการ](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (ใช้แบบ `ros-humble-desktop`) แล้วลงแพ็กเกจที่โปรเจกต์ใช้:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard python3-serial python3-numpy
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

ทุกคำสั่งบน Ubuntu: เปิด terminal ใหม่ → `cd ~/kobuki-pick-place/robot` → รันแบบไม่ต้องมี `pixi run` เช่น `python3 fake_robot.py`

> Nav2 ทดสอบบน Windows แล้ว ส่วนบน Ubuntu ยังไม่ได้ลองเต็มรอบ ถ้าติดตรงไหนแจ้งในกลุ่ม

---

## 3. ลองรันเลย: ภารกิจเต็มบนหุ่นปลอม (7 หน้าต่าง)

เปิด terminal 7 หน้าต่าง ทุกหน้าต่างเริ่มด้วย `cd` เข้าโฟลเดอร์ (Windows: `cd ~\kobuki-pick-place` / Ubuntu: `cd ~/kobuki-pick-place/robot`) แล้วรันทีละหน้าต่างตามลำดับ:

| หน้าต่าง | Windows | Ubuntu |
|---|---|---|
| 1 หุ่นปลอม | `pixi run python robot\fake_robot.py` | `python3 fake_robot.py` |
| 2 โมเดล + แขนปลอม | `pixi run ros2 launch robot\robot_model.launch.py arm:=sim` | `ros2 launch ./robot_model.launch.py arm:=sim` |
| 3 กล้องปลอม | `pixi run python robot\fake_camera.py` | `python3 fake_camera.py` |
| 4 ตัวหาวัตถุ | `pixi run python robot\cube_detector.py` | `python3 cube_detector.py` |
| 5 Nav2 | `pixi run ros2 launch robot\nav2.launch.py` | `ros2 launch ./nav2.launch.py` |
| 6 RViz | `pixi run rviz2 -d robot\mission.rviz` | `rviz2 -d mission.rviz` |
| 7 ภารกิจ | `pixi run python robot\mission_nav2.py` | `python3 mission_nav2.py` |

หุ่นจะวิ่งไป Station 1 → กล้องหาลูกบาศก์สีเขียว → แขนหยิบ → วิ่งไป Station 2 → หาแผ่นสีฟ้า → วาง → กลับบ้าน เองทั้งหมด
รายละเอียดทีละขั้นอยู่ใน `docs/PRACTICE_NO_ROBOT_TH.md` ภาค 5 · ซ้อมแค่การวิ่ง (ไม่ต้องเปิดหน้าต่าง 3–4): `mission_nav2.py --nav-only`

**กติกา:** 1 คำสั่งที่รันค้าง = 1 หน้าต่าง หยุดด้วย `Ctrl+C` ในหน้าต่างนั้น

---

## ไฟล์ในโปรเจกต์

| ที่อยู่ | คืออะไร |
|---|---|
| `robot/fake_robot.py` | หุ่นปลอม: ห้องสอบจำลอง ส่ง `/scan` `/odom` TF เหมือนหุ่นจริง |
| `robot/lidar_ros2.py`, `robot/kobuki_ros2.py` | ไดรเวอร์หุ่นจริง (ใช้วันจริงแทน `fake_robot.py`) |
| `robot/kobuki_x2.urdf`, `robot/robot_model.launch.py`, `robot/meshes/` | โมเดลหุ่นตามที่ประกอบ + ตำแหน่ง LiDAR / แขน / กล้อง (โค้ดทุกตัวอ่านจากไฟล์นี้) |
| `robot/arm_rest_publisher.py` | ส่งท่าพับของแขน SO-100 ตอนไม่ได้เปิดไดรเวอร์แขน |
| `robot/so100_driver.py`, `robot/so100_calibrate.py`, `robot/so100_calibrate_app.py`, `robot/feetech.py` | ไดรเวอร์แขนจริง (เซอร์โว Feetech STS3215) + คาลิเบรต 3 ท่า (แอปมีหน้าต่าง ค่าสด เช็คท่า วัดจุดคีบ GRIP) |
| `robot/camera_node.py`, `robot/hsv_tuner.py`, `robot/camera_check.py` | กล้องจริง, หาสีวัตถุ, เช็คมุมกล้องด้วยตลับเมตร |
| `robot/fake_camera.py`, `robot/fake_arm.py`, `robot/cube_detector.py`, `robot/arm_control.py` | กล้อง/แขนปลอมในซิม, ตัวหาวัตถุ, IK + ท่าหยิบวาง |
| `robot/slam_params.yaml`, `robot/mapping.rviz` | ค่าตั้งทำแผนที่และ RViz |
| `robot/nav2_params.yaml`, `robot/nav2.launch.py`, `robot/lifecycle_up.py` | Nav2 |
| `robot/mission_nav2.py` | state machine ภารกิจเต็ม |
| `robot/announcer.py`, `robot/voice_lines.py`, `robot/sounds/` | หุ่นพูดภาษาไทยบอกทุกขั้นของภารกิจ (เสียงอัดไว้แล้ว ไม่ต้องใช้เน็ต) — `tools/make_voice.py` อัดใหม่ |
| `robot/moveit.launch.py`, `robot/moveit/`, `robot/so100_trajectory_bridge.py` | MoveIt 2 วางแผนท่าแขน SO-100 ไม่ให้ชนตัวหุ่น (`arm_control.py --moveit`, `mission_nav2.py --moveit`) — `tools/moveit_collision_pairs.py` หาคู่ชิ้นที่ปิดการเช็คชน |
| `robot/foxglove.launch.py`, `robot/image_compress.py` | ดู/ขับหุ่นจากเบราว์เซอร์หรือมือถือผ่านไวไฟ (Foxglove) — `docs/DEMO_DAY_TH.md` ขั้น 11 |
| `robot/maps/` | แผนที่ (ห้องจำลอง) |
| `robot/start_mapping.ps1`, `robot/start_mapping.sh` | เปิดทุกหน้าต่างสำหรับทำแผนที่กับหุ่นจริงทีเดียว |
| `robot/ros_check.py` | ตรวจทั้งระบบในคำสั่งเดียว |
| `docs/` | คู่มือภาษาไทย + ต้นฉบับเว็บคู่มือ (`docs/handbook/`) |
| `pixi.toml`, `pixi.lock` | รายการแพ็กเกจ ROS สำหรับ Windows |

## ที่มาของโมเดล 3D

- Kobuki: ไฟล์ CAD ของ Yujin Robot จาก [kobuki_description](https://github.com/kobuki-base/kobuki_ros/tree/devel/kobuki_description) (BSD)
- SO-100: ไฟล์ของ TheRobotStudio ผ่าน [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/trs_so_arm100) (Apache-2.0)
- YDLIDAR X2, ASUS Xtion, แผ่นอะคริลิก: ขึ้นรูปเองตามดาต้าชีทและรูปถ่าย (`tools/make_meshes.py`)
- รายละเอียดอยู่ใน `robot/meshes/NOTICE.txt`
