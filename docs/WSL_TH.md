# รันหุ่นจริงใน WSL — ตั้งแต่ git clone

ใช้ pixi + RoboStack **Humble** ชุดเดียวกับ Windows (WSL เครื่องนี้เป็น Ubuntu 26.04 ซึ่ง apt มีแต่ ROS 2 รุ่นใหม่ ห้ามใช้ `/opt/ros` แทน: ค่า Nav2/MoveIt ของโปรเจกต์นี้เขียนสำหรับ Humble)
⚠ ทดสอบจริงแล้วบน Windows เท่านั้น · บน WSL ยังไม่เคยรันกับหุ่นจริง · วันเดโมถ้า WSL มีปัญหา กลับไปใช้ [RUN_ORDER_TH.md](RUN_ORDER_TH.md) บน Windows ได้ทันที

## A. Windows — ครั้งเดียว (PowerShell แบบ Run as administrator)

ติดตั้ง usbipd (ให้ WSL มองเห็นอุปกรณ์ USB):
```powershell
winget install --exact dorssel.usbipd-win
```
ปิด-เปิด PowerShell (admin) ใหม่ เสียบ USB ทั้ง 4 ตัว แล้ว:
```powershell
usbipd list
```
จดเลข `BUSID` ของ 4 ตัวนี้:

| อุปกรณ์ | VID:PID ใน `usbipd list` |
|---|---|
| Kobuki | `0403:6001` (USB Serial Converter / FTDI) |
| LiDAR | `10c4:ea60` (CP210x) |
| แขน | `1a86:55d3` (CH343) |
| กล้อง Xtion | `1d27:0601` (หรือเว็บแคมที่ใช้แทน) |

แชร์ทีละตัว (ครั้งเดียวต่อเครื่อง ใส่ BUSID ของตัวเอง):
```powershell
usbipd bind --busid 1-1
```

## B. WSL — ครั้งเดียว

```bash
curl -fsSL https://pixi.sh/install.sh | sh
exec bash
sudo usermod -aG dialout $USER
cd ~
git clone https://github.com/SSRobotic/kobuki-pick-place.git
cd kobuki-pick-place
pixi install
```
- clone ไว้ใน `~` (ดิสก์ของ Linux) **ไม่ใช่** `/mnt/d/...` : เร็วกว่ามาก และ pixi ต้องการ
- `usermod` มีผลหลังปิด WSL ครั้งหนึ่ง: ที่ PowerShell `wsl --shutdown` แล้วเปิด WSL ใหม่
- แผนที่ที่ทำบน Windows ไว้แล้ว: `mkdir -p ~/maps && cp /mnt/c/Users/$USER_WIN/maps/my_map.* ~/maps/` (แทน `$USER_WIN` ด้วยชื่อผู้ใช้ Windows เช่น `pomma`)

## C. ทุกครั้งก่อนรัน

**Windows (PowerShell ธรรมดา):** ส่ง USB ทั้ง 4 ตัวเข้า WSL (ทำใหม่ทุกครั้งที่ถอด-เสียบ หรือรีสตาร์ท)
```powershell
usbipd attach --wsl --busid 1-1
```
(ทำซ้ำกับ BUSID ของอีก 3 ตัว · ส่งเข้า WSL แล้ว Windows จะมองไม่เห็นตัวนั้นจนกว่าจะ `usbipd detach --busid ...`)

**WSL:** เช็คว่ามาครบ
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
✅ `ttyUSB0` `ttyUSB1` (Kobuki, LiDAR) + `ttyACM0` (แขน)

## D. เปิดเครื่อง → กลับจุดเริ่ม

0. หุ่นที่ Initial หันทางเดิม · แขนท่า REST · สวิตช์ Kobuki · ปุ่มแดงแบตแขน · ลูกบาศก์/จุดวางบนกล่อง 9 ซม.

**ทุกหน้าต่าง WSL เริ่มด้วย:**
```bash
cd ~/kobuki-pick-place
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/robot/fastdds_local.xml
```

**1 LiDAR**
```bash
pixi run python robot/lidar_ros2.py --mirror
```

**2 Kobuki**
```bash
pixi run python robot/kobuki_ros2.py --enable-motion
```

**3 โมเดลหุ่น**
```bash
pixi run ros2 launch robot/robot_model.launch.py arm:=none
```

**4 แขน**
```bash
pixi run python robot/so100_driver.py
```

**5 กล้อง** (Xtion ผ่าน OpenNI บน WSL ยังไม่เคยลอง: ถ้าไม่ขึ้น ใช้เว็บแคม)
```bash
pixi run python robot/camera_node.py --ros-args -p backend:=opencv -p device:=0 -p hfov_deg:=60
```

**6 ตัวหาวัตถุ** (ค่าสีของตัวเองแทน `[...]`)
```bash
pixi run python robot/cube_detector.py --ros-args -p surface_height:=0.09 -p cube_hsv_low:=[...] -p cube_hsv_high:=[...] -p marker_hsv_low:=[...] -p marker_hsv_high:=[...]
```

**7 Nav2**
```bash
pixi run ros2 launch robot/nav2.launch.py map:=$HOME/maps/my_map.yaml
```

**8 RViz**
```bash
pixi run rviz2 -d robot/mission.rviz
```

**12 MoveIt**
```bash
pixi run ros2 launch robot/moveit.launch.py rviz:=false
```

**10 เสียงหุ่น**
```bash
pixi run python robot/announcer.py
```

**9 ภารกิจ** (เปิดสุดท้าย)
```bash
pixi run python robot/mission_nav2.py --moveit
```

จบที่ `Mission finished: succeeded` · หุ่นกลับ Initial · หยุดฉุกเฉิน: `Ctrl+C` หน้าต่าง 9 แล้วหน้าต่าง 2

## ถ้าติด

| อาการ | แก้ |
|---|---|
| `No unique Kobuki adapter` / `servo board not found` | ยังไม่ได้ `usbipd attach` ตัวนั้น · `ls /dev/ttyUSB* /dev/ttyACM*` ต้องเห็น |
| `Permission denied: '/dev/ttyUSB0'` | ยังไม่ได้ `usermod -aG dialout` หรือยังไม่ได้ `wsl --shutdown` หลังจากนั้น |
| RViz ไม่ขึ้นหน้าต่าง | Windows 11 มี WSLg อยู่แล้ว: `wsl --update` ที่ PowerShell แล้ว `wsl --shutdown` |
| `pixi: command not found` | ปิดเปิดหน้าต่าง WSL ใหม่ (หรือ `exec bash`) หลังติดตั้ง pixi |
