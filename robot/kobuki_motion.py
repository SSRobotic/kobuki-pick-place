"""Kobuki command framing and bounded teleoperation, independent of ROS/USB.

Conversion follows kobuki_core/src/driver/diff_drive.cpp. The protocol is
documented at https://kobuki.readthedocs.io/en/devel/protocol.html.
"""
import math
import struct


WHEEL_SEPARATION = 0.23


def command_frame(linear, angular):
    """Convert planar m/s and rad/s to the speed/radius firmware command."""
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError("Velocity must be finite")
    if abs(angular) < 0.0001:
        speed, radius = linear * 1000.0, 0.0
    else:
        radius = linear * 1000.0 / angular
        if abs(linear) < 0.0001 or abs(radius) <= 1.0:
            speed, radius = 1000.0 * WHEEL_SEPARATION * angular / 2.0, 1.0
        else:
            outer_radius = radius + math.copysign(1000.0 * WHEEL_SEPARATION / 2.0, radius)
            speed = outer_radius * angular
    bounded = lambda value: max(-32768, min(32767, int(value)))
    payload = struct.pack("<BBhh", 1, 4, bounded(speed), bounded(radius))
    body = bytes([len(payload)]) + payload
    checksum = 0
    for byte in body:
        checksum ^= byte
    return b"\xaa\x55" + body + bytes([checksum])


STOP_FRAME = command_frame(0.0, 0.0)


class MotionController:
    def __init__(self, command_timeout=0.4, telemetry_timeout=0.2):
        self.command_timeout = command_timeout
        self.telemetry_timeout = telemetry_timeout
        self.last_command = None
        self.last_telemetry = None
        self.linear = self.angular = 0.0
        self.blocked_reason = "waiting for telemetry"
        self.reason = self.blocked_reason

    def update_telemetry(self, data, now):
        self.last_telemetry = now
        faults = [name for name in ("bumper", "wheel_drop", "cliff", "overcurrent", "charger")
                  if data.get(name, 1)]
        self.blocked_reason = ", ".join(faults)
        if faults:
            self.stop()

    def stop(self):
        self.linear = self.angular = 0.0
        self.last_command = None

    def accept(self, linear, angular, now):
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.stop()
            self.reason = "invalid velocity"
            return False
        if (self.blocked_reason or self.last_telemetry is None
                or now - self.last_telemetry > self.telemetry_timeout):
            self.stop()
            self.reason = self.blocked_reason or "telemetry timeout"
            return False
        # Limits for this first physical keyboard test, independent of teleop.
        self.linear = max(-0.12, min(0.12, float(linear)))
        self.angular = max(-0.6, min(0.6, float(angular)))
        self.last_command = now
        return True

    def frame(self, now):
        if self.blocked_reason:
            self.stop()
            self.reason = self.blocked_reason
        elif self.last_telemetry is None or now - self.last_telemetry > self.telemetry_timeout:
            self.stop()
            self.reason = "telemetry timeout"
        elif self.last_command is None or now - self.last_command > self.command_timeout:
            self.stop()
            self.reason = "command timeout"
        else:
            self.reason = "driving" if self.linear or self.angular else "stopped"
        return command_frame(self.linear, self.angular)


def disable_console_selection():
    """Prevent classic Windows QuickEdit selection from blocking motor timers.

    Return a function that restores the original console mode at shutdown.
    A redirected/non-Windows terminal requires no change.
    """
    import sys
    if sys.platform != "win32":
        return lambda: None
    import ctypes
    from ctypes import wintypes
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.GetStdHandle.argtypes = [wintypes.DWORD]
    kernel.GetStdHandle.restype = wintypes.HANDLE
    kernel.GetConsoleMode.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    kernel.GetConsoleMode.restype = wintypes.BOOL
    kernel.SetConsoleMode.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.SetConsoleMode.restype = wintypes.BOOL
    handle = kernel.GetStdHandle(wintypes.DWORD(-10).value)
    old_mode = wintypes.DWORD()
    if not kernel.GetConsoleMode(handle, ctypes.byref(old_mode)):
        return lambda: None
    # ENABLE_EXTENDED_FLAGS makes the removal of ENABLE_QUICK_EDIT_MODE apply.
    if not kernel.SetConsoleMode(handle, (old_mode.value | 0x80) & ~0x40):
        return lambda: None
    return lambda: kernel.SetConsoleMode(handle, old_mode.value)
