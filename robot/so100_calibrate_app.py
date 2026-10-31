"""SO-100 calibration in a window: the same 3 poses as so100_calibrate.py, with live servo readings, a check on every
pose before it is recorded, and a measurement of where the jaws close on the cube (GRIP).
    pixi run python robot\\so100_calibrate_app.py              (Windows)      python3 so100_calibrate_app.py   (Ubuntu)
    ... --port COM12      if the board is not found by itself
    ... --mock            no arm: virtual servos, to try the window
Stop so100_driver.py first: only one program can use the servo port. HOLD THE ARM before "Torque off".
Writes so100_calibration.yaml (the old one is kept as so100_calibration.yaml.bak); arm_control.py reads GRIP from it.
"""
import argparse
import datetime
import math
import os
import random
import shutil
import time
import tkinter as tk
from tkinter import messagebox, ttk

import yaml

from feetech import RAD_PER_STEP, BusError, FeetechBus, VirtualServos
from so100_calibrate import POSES, STEPS_TEXT, solve
from so100_driver import CAL_FILE, IDS, JOINTS, find_port, load_calibration, to_angle, to_step

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGES = {"REST": "calib_1_rest.png", "HOME": "calib_2_home.png", "PAN_90": "calib_3_pan_90.png"}
GRIP_PROBE = -0.10   # rad: the jaws close toward this; the cube stops them earlier = where they touch it
GRIP_SQUEEZE = 0.10  # rad: GRIP = touch angle - this, so the jaws really squeeze (31 Oct: touch 0.20 -> GRIP 0.10)
FONT = ("Leelawadee UI", 11)


def expected_moves(pose):
    """{joint: degrees it should move from REST} for a pose, from the angles in so100_calibrate.POSES."""
    if pose == "HOME":
        return {j: math.degrees(POSES["HOME"][j] - POSES["REST"][j]) for j in POSES["HOME"] if j != "shoulder_pan"}
    if pose == "PAN_90":
        return {"shoulder_pan": 90.0}
    return {}


class CalibrationApp:
    def __init__(self, root, bus):
        self.root, self.bus = root, bus
        self.readings = {}            # pose -> {joint: step}
        self.result = None            # (cal, warnings) after the 3 poses
        self.cal = load_calibration() # the saved one, for showing angles
        self.steps = {}
        self.pose_index = 0
        self.images = {}
        root.title("SO-100 calibration")
        root.option_add("*Font", FONT)
        style = ttk.Style()   # ttk widgets ignore option_add: without this, Thai on buttons and frame titles is tiny
        style.configure(".", font=FONT)
        style.configure("TLabelframe.Label", font=(FONT[0], 11, "bold"))
        self.build()
        self.show_pose()
        self.tick()

    # ---------- layout ----------
    def build(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        self.status = ttk.Label(top, text="")
        self.status.pack(side="left")
        ttk.Button(top, text="Torque ON (ล็อกท่า)", command=lambda: self.torque(1)).pack(side="right", padx=4)
        ttk.Button(top, text="Torque OFF (ประคองแขนก่อน!)", command=lambda: self.torque(0)).pack(side="right", padx=4)

        body = ttk.Frame(self.root, padding=8)
        body.pack(fill="both", expand=True)
        table = ttk.LabelFrame(body, text="ค่าเซอร์โวสด", padding=8)
        table.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        heads = ["ข้อ", "step", "มุม (°)", "REST", "HOME", "PAN_90"]
        for c, h in enumerate(heads):
            ttk.Label(table, text=h, font=(FONT[0], 10, "bold")).grid(row=0, column=c, padx=6, sticky="w")
        self.cells = {}
        for r, j in enumerate(JOINTS, 1):
            ttk.Label(table, text=f"{IDS[j]} {j}").grid(row=r, column=0, padx=6, sticky="w")
            for c in range(1, 6):
                lbl = ttk.Label(table, text="-", width=7)
                lbl.grid(row=r, column=c, padx=6, sticky="e")
                self.cells[(j, c)] = lbl

        self.step_box = ttk.LabelFrame(body, text="", padding=8)
        self.step_box.grid(row=0, column=1, sticky="nsew")
        self.picture = ttk.Label(self.step_box)
        self.picture.pack()
        self.instruction = ttk.Label(self.step_box, text="", wraplength=470, justify="left")
        self.instruction.pack(fill="x", pady=6)
        self.check = tk.Label(self.step_box, text="", justify="left", anchor="w", font=FONT)
        self.check.pack(fill="x")
        buttons = ttk.Frame(self.step_box)
        buttons.pack(fill="x", pady=6)
        self.back_button = ttk.Button(buttons, text="< ย้อนกลับ", command=self.back)
        self.back_button.pack(side="left")
        self.record_button = ttk.Button(buttons, text="บันทึกท่านี้", command=self.record)
        self.record_button.pack(side="right")

        self.result_box = ttk.LabelFrame(body, text="ผลคาลิเบรต", padding=8)
        self.result_box.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        self.result_text = tk.Label(self.result_box, text="ยังไม่ครบ 3 ท่า", justify="left", anchor="w", font=FONT)
        self.result_text.pack(fill="x")
        self.save_button = ttk.Button(self.result_box, text="บันทึกไฟล์ so100_calibration.yaml", command=self.save,
                                      state="disabled")
        self.save_button.pack(anchor="e")

        grip = ttk.LabelFrame(body, text="วัดจุดคีบลูกบาศก์ (GRIP) - ทำหลังคาลิเบรต", padding=8)
        grip.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        ttk.Label(grip, text="อ้าปากคีบ วางลูกบาศก์ 4 ซม. ไว้ระหว่างนิ้ว (ถือไว้หรือวางบนกล่อง) แล้วกดปุ่ม: ปากคีบจะหนีบ "
                             "หาจุดที่แตะลูกบาศก์ แล้วอ้าคืน", wraplength=900, justify="left").pack(fill="x")
        row = ttk.Frame(grip)
        row.pack(fill="x", pady=4)
        self.grip_text = ttk.Label(row, text=self.grip_summary())
        self.grip_text.pack(side="left")
        ttk.Button(row, text="วัดจุดคีบ", command=self.measure_grip).pack(side="right")
        body.columnconfigure(1, weight=1)

    # ---------- servos ----------
    def read_all(self):
        return {j: self.bus.read_position(IDS[j]) for j in JOINTS}

    def torque(self, on):
        if not on and not messagebox.askokcancel("Torque OFF", "แขนจะอ่อนและตกทันที\nประคองแขนไว้ด้วยมือแล้วกด OK"):
            return
        try:
            if on:   # goal = where each servo is now, or torque on makes it jump to an old goal
                self.bus.write_goals({IDS[j]: s for j, s in self.read_all().items()}, 600, 30)
            for j in JOINTS:
                self.bus.write(IDS[j], "torque_enable", on)
        except BusError as e:
            messagebox.showerror("servo", str(e))

    def tick(self):
        try:
            self.steps = self.read_all()
            bad = {IDS[j]: w for j in JOINTS if (w := self.bus.errors.get(IDS[j]))}
            self.status.config(text="เซอร์โวครบ 6 ตัว" + (f" · เตือน: {bad}" if bad else ""), foreground="black")
        except BusError as e:
            self.steps = {}
            self.status.config(text=f"อ่านเซอร์โวไม่ได้: {e} (แบตแขนดับ? ปิด so100_driver.py แล้วหรือยัง?)", foreground="red")
        cal = self.result_cal() or self.cal
        for j in JOINTS:
            s = self.steps.get(j)
            self.cells[(j, 1)].config(text="-" if s is None else str(s))
            self.cells[(j, 2)].config(text="-" if s is None or cal is None else f"{math.degrees(to_angle(cal, j, s)):+.0f}")
            for c, pose in enumerate(("REST", "HOME", "PAN_90"), 3):
                self.cells[(j, c)].config(text=str(self.readings.get(pose, {}).get(j, "-")))
        self.update_check()
        self.root.after(150, self.tick)

    # ---------- the 3 poses ----------
    def pose(self):
        return list(POSES)[self.pose_index] if self.pose_index < len(POSES) else None

    def show_pose(self):
        pose = self.pose()
        if pose is None:
            self.step_box.config(text="ครบ 3 ท่าแล้ว")
            self.instruction.config(text="ดูผลด้านล่าง ถ้าไม่มีคำเตือนสีแดง กดบันทึกไฟล์ · ไม่พอใจ กดย้อนกลับไปบันทึกท่าใหม่")
            self.picture.config(image="")
            self.record_button.config(state="disabled")
            return
        self.step_box.config(text=f"{self.pose_index + 1}/3  ท่า {pose}")
        if pose not in self.images:
            try:
                self.images[pose] = tk.PhotoImage(file=os.path.join(HERE, "..", "docs", "img", IMAGES[pose])).subsample(2)
            except tk.TclError:
                self.images[pose] = None
        self.picture.config(image=self.images[pose] or "")
        self.instruction.config(text=STEPS_TEXT[pose])
        self.back_button.config(state="normal" if self.pose_index else "disabled")

    def update_check(self):
        """Live check of the pose being set up; HOME cannot be recorded while the jaws are closed."""
        pose, ok, lines = self.pose(), True, []
        if pose is None or not self.steps:
            self.check.config(text="")
            return
        rest = self.readings.get("REST")
        for j, deg in expected_moves(pose).items():
            moved = abs(self.steps[j] - rest[j]) * math.degrees(RAD_PER_STEP)
            good = 0.6 * abs(deg) < moved < 1.5 * abs(deg)
            if j == "gripper":
                good = moved > 30
                ok = ok and good
            lines.append(f"{'✓' if good else '✗'} {j}: ขยับจาก REST {moved:.0f}° (ควรประมาณ {abs(deg):.0f}°)"
                         + ("" if good or j != "gripper" else "  ← อ้าปากคีบก่อน!"))
        if pose == "REST":
            lines.append("พับแขนสุด ปากคีบหุบสนิท แล้วกดบันทึก (ท่านี้ใช้เป็นจุดเริ่มของทุกข้อ)")
        self.check.config(text="\n".join(lines), fg="dark green" if ok and all(l[0] != "✗" for l in lines if l[0] in "✓✗")
                          else "firebrick")
        self.record_button.config(state="normal" if ok else "disabled")

    def record(self):
        if not self.steps:
            return
        self.readings[self.pose()] = dict(self.steps)
        self.pose_index += 1
        if self.pose_index == len(POSES):
            self.result = solve(self.readings)
            self.show_result()
        self.show_pose()

    def back(self):
        self.pose_index = max(0, self.pose_index - 1)
        self.readings.pop(self.pose(), None)
        self.result = None
        self.result_text.config(text="ยังไม่ครบ 3 ท่า", fg="black")
        self.save_button.config(state="disabled")
        self.show_pose()

    def result_cal(self):
        if not self.result:
            return None
        return {j: (c["zero"], c["sign"]) for j, c in self.result[0].items()}

    def show_result(self):
        cal, warnings = self.result
        lines = [f"{j:14s} zero {c['zero']:7.1f}  sign {c['sign']:+d}" for j, c in cal.items()]
        lines += [f"⚠ {w}" for w in warnings] or ["ไม่มีคำเตือน"]
        self.result_text.config(text="\n".join(lines), fg="firebrick" if warnings else "dark green", font=("Consolas", 10))
        self.save_button.config(state="normal")

    def save(self):
        cal, _ = self.result
        old = {}
        if os.path.exists(CAL_FILE):
            shutil.copy(CAL_FILE, CAL_FILE + ".bak")
            with open(CAL_FILE, encoding="utf-8") as f:
                old = yaml.safe_load(f) or {}
        out = {"joints": cal, "readings": self.readings, "date": datetime.datetime.now().isoformat(timespec="seconds"),
               "note": "angle = sign * (step - zero) * 2*pi/4096, angles as in kobuki_x2.urdf (so100_calibrate_app.py)"}
        for key in ("grip", "grip_contact"):   # a grip measured before stays valid
            if key in old:
                out[key] = old[key]
        with open(CAL_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)
        self.cal = self.result_cal()
        messagebox.showinfo("บันทึกแล้ว", f"{CAL_FILE}\n(ไฟล์เก่าเก็บเป็น .bak)\n\nต่อไป: วัดจุดคีบ แล้ววางแขนท่า REST ก่อนเปิด so100_driver.py")

    # ---------- GRIP ----------
    def grip_summary(self):
        try:
            with open(CAL_FILE, encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            if "grip" in data:
                return f"ตอนนี้: แตะลูกบาศก์ที่ {data['grip_contact']:+.2f} rad → GRIP = {data['grip']:+.2f} rad"
        except OSError:
            pass
        return "ยังไม่ได้วัด (arm_control.py ใช้ค่า GRIP ในโค้ด)"

    def measure_grip(self):
        cal = self.result_cal() or self.cal
        if cal is None:
            messagebox.showwarning("GRIP", "คาลิเบรตและบันทึกไฟล์ก่อน")
            return
        sid = IDS["gripper"]
        try:
            self.bus.write(sid, "torque_limit", 500)          # same half torque as so100_driver.py
            self.bus.write_goals({sid: to_step(cal, "gripper", GRIP_PROBE)}, 600, 30)
            self.bus.write(sid, "torque_enable", 1)
            last, still_since, start = None, None, time.time()
            while time.time() - start < 4.0:                  # until the jaw stops on the cube
                self.root.update()
                step = self.bus.read_position(sid)
                still_since = (still_since or time.time()) if last is not None and abs(step - last) < 2 else None
                last = step
                if still_since and time.time() - still_since > 0.5:
                    break
                time.sleep(0.05)
            contact = to_angle(cal, "gripper", last)
            self.bus.write_goals({sid: to_step(cal, "gripper", 0.8)}, 600, 30)   # open again
        except BusError as e:
            messagebox.showerror("servo", str(e))
            return
        if contact < GRIP_PROBE + 0.05:
            messagebox.showwarning("GRIP", f"ปากคีบหุบสุด ({contact:+.2f} rad) ไม่เจอลูกบาศก์ - วางลูกบาศก์ระหว่างนิ้วแล้วลองใหม่")
            return
        grip = round(contact - GRIP_SQUEEZE, 2)
        with open(CAL_FILE, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        data["grip_contact"], data["grip"] = round(contact, 3), grip
        with open(CAL_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.grip_text.config(text=self.grip_summary())
        messagebox.showinfo("GRIP", f"แตะลูกบาศก์ที่ {contact:+.2f} rad\nGRIP = {grip:+.2f} rad (บันทึกในไฟล์แล้ว)")


def main():
    ap = argparse.ArgumentParser(description="SO-100 calibration window")
    ap.add_argument("--port")
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()
    if args.mock:   # virtual servos that stand still: shows the window and the live table, nothing to calibrate
        virtual = VirtualServos({IDS[j]: 2048 + random.randint(-400, 400) for j in JOINTS})
        bus = FeetechBus(virtual)
    else:
        port = args.port or find_port()
        if not port:
            raise SystemExit("servo board not found - plug it in or use --port COM12")
        try:
            bus = FeetechBus(port)
        except Exception as e:   # noqa: BLE001 - serial errors differ per OS
            raise SystemExit(f"cannot open {port}: {e}\n(is so100_driver.py still running? stop it first)")
    root = tk.Tk()
    CalibrationApp(root, bus)
    root.mainloop()


if __name__ == "__main__":
    main()
