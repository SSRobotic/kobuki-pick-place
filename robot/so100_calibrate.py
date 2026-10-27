"""SO-100 calibration: find how each servo's steps map to the joint angles of kobuki_x2.urdf.

The torque goes off, you move the arm by hand into 3 poses (pictures in docs/DEMO_DAY_TH.md), and the
tool works out each joint's zero and direction from what the servos read. Result: so100_calibration.yaml.
    pixi run python robot\\so100_calibrate.py           (Windows)      python3 so100_calibrate.py   (Ubuntu)
    ... --port COM5              if the board is not found by itself
    ... --relax                  only switch the torque off (to put the arm down by hand), then quit
    ... --mock                   no arm: a virtual one with a hidden calibration, checks the maths
HOLD THE ARM before you start: with the torque off it falls under its own weight.
"""
import argparse
import datetime
import math
import random

import yaml

from feetech import RAD_PER_STEP, STEPS, FeetechBus, VirtualServos
from so100_driver import CAL_FILE, IDS, JOINTS, LIMITS, find_port

# the three poses, in so_arm100.xml joint angles (rad)
POSES = {
    "REST": {"shoulder_pan": 0.0, "shoulder_lift": -3.32, "elbow_flex": 3.11, "wrist_flex": 1.18,
             "wrist_roll": -1.5708, "gripper": -0.17},
    "HOME": {"shoulder_pan": 0.0, "shoulder_lift": -1.57, "elbow_flex": 1.57, "wrist_flex": 1.57,
             "wrist_roll": -1.5708, "gripper": 1.0},
    "PAN_90": {"shoulder_pan": math.pi / 2},
}
STEPS_TEXT = {
    "REST": "ท่า REST: ฐานแขนหันตรงออกท้ายหุ่น พับแขนสุด (แขนบนพับนอนไปทางหัวหุ่น แขนล่างพับทับ) "
            "มือจับห้อยออกท้ายหุ่น ปากคีบหุบสนิท ปากคีบอ้าออกด้านข้าง (แนวนอน) เหมือนรูป calib_1_rest",
    "HOME": "ท่า HOME: ฐานหันตรงออกท้ายหุ่น แขนบนตั้งตรงขึ้นฟ้า แขนล่างยื่นออกท้ายหุ่นขนานพื้น "
            "มือจับชี้ลงพื้นตรง ๆ อ้าปากคีบกว้างประมาณ 60 องศา เหมือนรูป calib_2_home",
    "PAN_90": "หมุนฐานแขน 90 องศาให้แขนชี้ไปทางขวาของหุ่น (มองจากบน = ทวนเข็มนาฬิกา) ข้ออื่นท่าไหนก็ได้ "
              "แต่ยกให้พ้นแบต เหมือนรูป calib_3_pan_90",
}


def solve(readings):
    """readings[pose][joint] = servo steps -> ({joint: {zero, sign}}, warnings)."""
    cal, warn = {}, []
    rest, home, pan = readings["REST"], readings["HOME"], readings["PAN_90"]
    for j in ("shoulder_lift", "elbow_flex", "wrist_flex", "gripper"):
        dq = POSES["HOME"][j] - POSES["REST"][j]
        ds = home[j] - rest[j]
        sign = 1 if ds * dq > 0 else -1
        ratio = abs(ds) * RAD_PER_STEP / abs(dq)
        if j != "gripper" and not 0.6 < ratio < 1.5:
            warn.append(f"{j}: moved {math.degrees(abs(ds) * RAD_PER_STEP):.0f} deg between REST and HOME, "
                        f"expected {math.degrees(abs(dq)):.0f} deg - check the poses")
        if j == "gripper":   # REST is the hard stop (closed): trust it, HOME only gives the direction
            zero = rest[j] - sign * POSES["REST"][j] / RAD_PER_STEP
        else:
            zero = (rest[j] - sign * POSES["REST"][j] / RAD_PER_STEP + home[j] - sign * POSES["HOME"][j] / RAD_PER_STEP) / 2
        cal[j] = {"zero": round(zero, 1), "sign": sign}
    ds = pan["shoulder_pan"] - rest["shoulder_pan"]
    if abs(ds) * RAD_PER_STEP < math.radians(45):
        warn.append("shoulder_pan hardly moved in step 3 - turn the base 90 deg so the arm points to the robot's right")
    cal["shoulder_pan"] = {"zero": float(rest["shoulder_pan"]), "sign": 1 if ds > 0 else -1}
    # wrist_roll is held still by arm_control.py: its REST reading becomes the -90 deg it always uses
    cal["wrist_roll"] = {"zero": round(rest["wrist_roll"] + math.pi / 2 / RAD_PER_STEP, 1), "sign": 1}
    for j, c in cal.items():
        used = (-math.pi / 2,) if j == "wrist_roll" else LIMITS[j]     # wrist_roll only ever sits at -90 deg
        ends = [c["zero"] + c["sign"] * a / RAD_PER_STEP for a in used]
        if min(ends) < 0 or max(ends) >= STEPS:
            warn.append(f"{j}: its range would cross the servo's 0/4095 point ({min(ends):.0f}..{max(ends):.0f}) - "
                        "the arm can still work, but moves near that end will be refused")
    return cal, warn


def main():
    ap = argparse.ArgumentParser(description="SO-100 calibration")
    ap.add_argument("--port")
    ap.add_argument("--relax", action="store_true")
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()

    if args.mock:
        truth = {j: (random.uniform(1500, 2600), random.choice((1, -1))) for j in JOINTS}
        virtual = VirtualServos({IDS[j]: 2048 for j in JOINTS})
        bus = FeetechBus(virtual)
    else:
        port = args.port or find_port()
        if not port:
            raise SystemExit("servo board not found - plug it in or use --port COM5 (Windows) / --port /dev/ttyACM0")
        bus = FeetechBus(port)
    missing = [j for j in JOINTS if not bus.ping(IDS[j])]
    if missing:
        raise SystemExit(f"servos not answering: {missing} - check the power supply and the cable chain")

    if not args.mock:
        input("ประคองแขนไว้ด้วยมือ แล้วกด Enter เพื่อปล่อยแรงบิด (แขนจะอ่อน) ")
    for j in JOINTS:
        bus.write(IDS[j], "torque_enable", 0)
    if args.relax:
        print("torque off - put the arm down gently")
        return

    readings = {}
    for n, pose in enumerate(POSES, 1):
        print(f"\n{n}/3 {STEPS_TEXT[pose]}")
        if args.mock:   # the "hand" puts the virtual arm in the pose, with a little error
            for j, q in POSES[pose].items():
                zero, sign = truth[j]
                virtual.set_position(IDS[j], zero + sign * q / RAD_PER_STEP + random.uniform(-15, 15))
        else:
            input("   จัดท่าแล้วกด Enter ")
        readings[pose] = {j: bus.read_position(IDS[j]) for j in JOINTS}
        print("   อ่านได้: " + "  ".join(f"{j}={s}" for j, s in readings[pose].items()))

    cal, warnings = solve(readings)
    for w in warnings:
        print("WARNING:", w)
    if args.mock:
        for j in JOINTS:
            if j == "wrist_roll":
                continue
            zero, sign = truth[j]
            err = (cal[j]["zero"] - zero) * RAD_PER_STEP if cal[j]["sign"] == sign else float("nan")
            print(f"   {j:14s} sign {'ok' if cal[j]['sign'] == sign else 'WRONG'}  zero error {math.degrees(err):+.1f} deg")
        return
    out = {"joints": cal, "readings": readings, "date": datetime.datetime.now().isoformat(timespec="seconds"),
           "note": "angle = sign * (step - zero) * 2*pi/4096, angles as in kobuki_x2.urdf"}
    yaml.safe_dump(out, open(CAL_FILE, "w", encoding="utf-8"), sort_keys=False, allow_unicode=True)
    print(f"\nsaved {CAL_FILE}")
    print("ต่อไป: เปิด so100_driver.py แล้วลอง arm_control.py rest / carry ช้า ๆ (ดู docs/DEMO_DAY_TH.md)")
    print("แรงบิดยังปิดอยู่ - วางแขนลงท่า REST ก่อนเปิด driver")


if __name__ == "__main__":
    main()
