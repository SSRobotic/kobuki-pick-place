# Kobuki Pick & Place — Final Project 01208567

หุ่นยนต์ Kobuki + YDLIDAR X2 + แขน SO-100 + กล้อง ASUS Xtion ออกจาก Initial ไปหยิบลูกบาศก์ที่ Station 1 ไปวางที่ Station 2 แล้วกลับ Initial เอง
แขนกับกล้องหันออกด้านหน้าหุ่น · ลูกบาศก์ 4 ซม. และจุดวางอยู่บนกล่องสูง 9 ซม.

## 📖 คู่มือ — เปิดอันเดียวตามเครื่องที่ใช้ อ่านจบในหน้าเดียว

| เครื่อง | คู่มือ |
|---|---|
| **Windows** (ใช้ pixi) | **[GUIDE_WINDOWS_PIXI.md](GUIDE_WINDOWS_PIXI.md)** |
| **Ubuntu 22.04 / VM** (ไม่ใช้ pixi) | **[GUIDE_UBUNTU.md](GUIDE_UBUNTU.md)** |

ทั้งสองอันลำดับเดียวกัน: A ติดตั้ง → B ประกอบ + ตั้งแขน → C แผนที่ + ค่าสี + ตำแหน่ง Station → D รันภารกิจ → ถ้าติด

ซ้อมโดยไม่มีหุ่น: `robot/fake_robot.py` (หุ่นปลอม) แทนหน้าต่าง 1–2, `robot_model.launch.py arm:=sim`, `fake_camera.py` แทนกล้อง

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
| `robot/save_stations.py` | คลิกลูกบาศก์/จุดวางใน RViz → `robot/stations.yaml` (ภารกิจอ่านเอง) |
| `robot/nav_check.py` | เช็คว่าทำไม Nav2 เดินมั่ว (slam ค้าง, ตำแหน่งหุ่น, จุด LiDAR ทับกำแพง) |
| `robot/fake_camera.py`, `robot/fake_arm.py`, `robot/cube_detector.py`, `robot/arm_control.py` | กล้อง/แขนปลอมในซิม, ตัวหาวัตถุ, IK + ท่าหยิบวาง |
| `robot/slam_params.yaml`, `robot/mapping.rviz` | ค่าตั้งทำแผนที่และ RViz |
| `robot/nav2_params.yaml`, `robot/nav2.launch.py`, `robot/lifecycle_up.py` | Nav2 |
| `robot/mission_nav2.py` | state machine ภารกิจเต็ม |
| `robot/announcer.py`, `robot/voice_lines.py`, `robot/sounds/` | หุ่นพูดภาษาไทยบอกทุกขั้นของภารกิจ (เสียงอัดไว้แล้ว ไม่ต้องใช้เน็ต) — `tools/make_voice.py` อัดใหม่ |
| `robot/moveit.launch.py`, `robot/moveit/`, `robot/so100_trajectory_bridge.py` | MoveIt 2 วางแผนท่าแขน SO-100 ไม่ให้ชนตัวหุ่น (`arm_control.py --moveit`, `mission_nav2.py --moveit`) — `tools/moveit_collision_pairs.py` หาคู่ชิ้นที่ปิดการเช็คชน |
| `robot/foxglove.launch.py`, `robot/image_compress.py` | ดู/ขับหุ่นจากเบราว์เซอร์หรือมือถือผ่านไวไฟ (Foxglove) |
| `robot/maps/` | แผนที่ (ห้องจำลอง) |
| `robot/start_mapping.ps1`, `robot/start_mapping.sh` | เปิดทุกหน้าต่างสำหรับทำแผนที่กับหุ่นจริงทีเดียว |
| `robot/ros_check.py` | ตรวจทั้งระบบในคำสั่งเดียว |
| `docs/` | เว็บคู่มือ (`docs/handbook/`) + รูป |
| `pixi.toml`, `pixi.lock` | รายการแพ็กเกจ ROS สำหรับ Windows |

## ที่มาของโมเดล 3D

- Kobuki: ไฟล์ CAD ของ Yujin Robot จาก [kobuki_description](https://github.com/kobuki-base/kobuki_ros/tree/devel/kobuki_description) (BSD)
- SO-100: ไฟล์ของ TheRobotStudio ผ่าน [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/trs_so_arm100) (Apache-2.0)
- YDLIDAR X2, ASUS Xtion, แผ่นอะคริลิก: ขึ้นรูปเองตามดาต้าชีทและรูปถ่าย (`tools/make_meshes.py`)
- รายละเอียดอยู่ใน `robot/meshes/NOTICE.txt`
