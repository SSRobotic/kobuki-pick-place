"""Feetech STS3215 serial bus (the six SO-100 servos), plus a virtual bus for testing without the arm.

Protocol and register addresses follow Feetech's own Python SDK (FTServo_Python, sms_sts.py /
protocol_packet_handler.py) and match LeRobot's STS3215 table:
  packet = FF FF id length instruction params... checksum,  length = params + 2,
  checksum = ~(id + length + instruction + params) & 0xFF,  16-bit values little-endian,
  positions are sign-magnitude with the sign in bit 15, 4096 steps per turn.
Only pyserial is needed (already in pixi.toml / python3-serial on Ubuntu).
"""
import math
import time

ADDR = {  # register: (address, bytes)
    "model": (3, 2), "id": (5, 1), "baud_rate": (6, 1), "min_angle_limit": (9, 2), "max_angle_limit": (11, 2),
    "max_torque_limit": (16, 2), "p_coefficient": (21, 1), "d_coefficient": (22, 1), "i_coefficient": (23, 1),
    "protection_current": (28, 2), "offset": (31, 2), "mode": (33, 1), "overload_torque": (36, 1),
    "torque_enable": (40, 1), "acceleration": (41, 1), "goal_position": (42, 2), "goal_time": (44, 2),
    "goal_speed": (46, 2), "torque_limit": (48, 2), "lock": (55, 1), "present_position": (56, 2),
    "present_speed": (58, 2), "present_load": (60, 2), "present_voltage": (62, 1), "present_temperature": (63, 1),
    "moving": (66, 1), "present_current": (69, 2),
}
PING, READ, WRITE, SYNC_WRITE = 0x01, 0x02, 0x03, 0x83
BROADCAST = 0xFE
STEPS = 4096
RAD_PER_STEP = 2 * math.pi / STEPS
ERRORS = {1: "voltage", 2: "angle", 4: "overheat", 8: "over-current", 32: "overload"}


def packet(sid, instruction, params=()):
    body = [sid, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])


def to_sign_magnitude(value, bit=15):
    return (-value) | (1 << bit) if value < 0 else value


def from_sign_magnitude(value, bit=15):
    return -(value & ((1 << bit) - 1)) if value & (1 << bit) else value


class BusError(Exception):
    pass


class FeetechBus:
    """Half-duplex request/reply on one serial port. `port` is a device name or an already open serial-like object."""

    def __init__(self, port, baudrate=1_000_000, timeout=0.02):
        if isinstance(port, str):
            import serial
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        else:
            self.ser = port
        self.timeout = timeout
        self.errors = {}   # {id: "overload, ..."} from the last reply of each servo that reported a problem

    def close(self):
        self.ser.close()

    def _request(self, sid, instruction, params=(), reply_params=0):
        tx = packet(sid, instruction, params)
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        if sid == BROADCAST:
            return b""
        need = 6 + reply_params
        buf = b""
        deadline = time.time() + self.timeout + 0.01
        while time.time() < deadline:
            buf += self.ser.read(max(1, need + len(tx) - len(buf)))
            if buf.startswith(tx):          # some USB adapters echo what we sent
                buf = buf[len(tx):]
            elif tx.startswith(buf):        # still receiving that echo
                continue
            start = buf.find(b"\xff\xff" + bytes([sid]))
            if start >= 0 and len(buf) - start >= need:
                reply = buf[start:start + need]
                if reply[3] != reply_params + 2 or (~sum(reply[2:-1]) & 0xFF) != reply[-1]:
                    raise BusError(f"servo {sid}: bad reply {reply.hex(' ')}")
                # an error flag (overload, overheat...) still comes with good data: keep it, report it in self.errors
                if reply[4]:
                    self.errors[sid] = ", ".join(n for b, n in ERRORS.items() if reply[4] & b) or f"0x{reply[4]:02x}"
                else:
                    self.errors.pop(sid, None)
                return reply[5:-1]
        raise BusError(f"servo {sid}: no reply")

    def ping(self, sid):
        try:
            self._request(sid, PING)
            return True
        except BusError:
            return False

    def read(self, sid, name):
        addr, size = ADDR[name]
        data = self._request(sid, READ, (addr, size), reply_params=size)
        return data[0] if size == 1 else data[0] | data[1] << 8

    def write(self, sid, name, value):
        addr, size = ADDR[name]
        data = (value & 0xFF,) if size == 1 else (value & 0xFF, (value >> 8) & 0xFF)
        self._request(sid, WRITE, (addr, *data))

    def read_position(self, sid):
        return from_sign_magnitude(self.read(sid, "present_position"))

    def read_state(self, sid):
        """Position (steps), speed (steps/s) and load (-1000..1000) in one request (registers 56-61)."""
        d = self._request(sid, READ, (ADDR["present_position"][0], 6), reply_params=6)
        word = lambda i: d[i] | d[i + 1] << 8  # noqa: E731
        return from_sign_magnitude(word(0)), from_sign_magnitude(word(2)), from_sign_magnitude(word(4), 10)

    def write_goals(self, goals, speed, acceleration):
        """One SYNC_WRITE for many servos: {id: position steps}. Writes acceleration, goal position, time and speed."""
        params = [ADDR["acceleration"][0], 7]
        for sid, pos in goals.items():
            p = to_sign_magnitude(int(round(pos)))
            params += [sid, acceleration, p & 0xFF, p >> 8, 0, 0, speed & 0xFF, speed >> 8]
        self._request(BROADCAST, SYNC_WRITE, params)


class VirtualServos:
    """Pretends to be the serial port with STS3215 servos on it (for --mock runs and tests).
    Torque on: each servo moves toward its goal at the commanded speed. Torque off: it stays where the test puts it."""

    def __init__(self, positions):
        self.mem = {}
        for sid, pos in positions.items():
            m = bytearray(128)
            m[3:5] = (777).to_bytes(2, "little")      # model number of an STS3215
            m[5] = sid
            m[56:58] = int(pos).to_bytes(2, "little")
            m[42:44] = int(pos).to_bytes(2, "little")
            m[62], m[63] = 74, 32                     # 7.4 V, 32 C
            self.mem[sid] = m
        self.out = b""
        self.last = time.time()

    def set_position(self, sid, pos):                 # "move the arm by hand" while torque is off
        self.mem[sid][56:58] = int(pos).to_bytes(2, "little")

    def _step(self):
        now = time.time()
        dt, self.last = now - self.last, now
        for m in self.mem.values():
            if m[40]:
                cur = from_sign_magnitude(int.from_bytes(m[56:58], "little"))
                goal = from_sign_magnitude(int.from_bytes(m[42:44], "little"))
                speed = int.from_bytes(m[46:48], "little") or 3000
                cur += max(-speed * dt, min(speed * dt, goal - cur))
                m[56:58] = to_sign_magnitude(int(round(cur))).to_bytes(2, "little")

    # serial.Serial look-alike
    def reset_input_buffer(self):
        self.out = b""

    def close(self):
        pass

    def read(self, n=1):
        data, self.out = self.out[:n], self.out[n:]
        return data

    def write(self, data):
        self._step()
        sid, inst, params = data[2], data[4], data[5:-1]
        if (~sum(data[2:-1]) & 0xFF) != data[-1]:
            return
        if inst == SYNC_WRITE:
            addr, size = params[0], params[1]
            for i in range(2, len(params), size + 1):
                if params[i] in self.mem:
                    self.mem[params[i]][addr:addr + size] = params[i + 1:i + 1 + size]
            return
        if sid not in self.mem:
            return
        reply = b""
        if inst == READ:
            reply = bytes(self.mem[sid][params[0]:params[0] + params[1]])
        elif inst == WRITE:
            self.mem[sid][params[0]:params[0] + len(params) - 1] = params[1:]
        body = [sid, len(reply) + 2, 0, *reply]
        self.out += bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])
