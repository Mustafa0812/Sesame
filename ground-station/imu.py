"""MPU6500 reader for the Raspberry Pi (I2C bus 1, address 0x68) with a complementary tilt filter.

Wiring (Pi 3.3 V logic only): VCC -> 3V3 (pin 1), GND -> GND (pin 6), SDA -> GPIO2 (pin 3),
SCL -> GPIO3 (pin 5), AD0 -> GND (address 0x68). Enable I2C first: sudo raspi-config -> Interfaces -> I2C.
"""
import math
import threading
import time

try:
    from smbus2 import SMBus
except ImportError:  # lets the rest of the app start on a PC without the library
    SMBus = None

WHO_AM_I_OK = (0x70, 0x71, 0x68, 0x73)  # MPU6500, MPU9250, MPU6050/6886 variants


class Imu:
    def __init__(self, bus=1, addr=0x68):
        self.bus_no, self.addr = bus, addr
        self.bus = None
        self.pitch = 0.0   # degrees, + = nose down toward +X of the chip (see invert flags in guidance)
        self.roll = 0.0
        self.ok = False
        self.error = "not started"
        self.updated = 0.0
        self._stop = False

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        if SMBus is None:
            raise RuntimeError("smbus2 not installed (pip install smbus2)")
        self.bus = SMBus(self.bus_no)
        who = self.bus.read_byte_data(self.addr, 0x75)
        if who not in WHO_AM_I_OK:
            raise RuntimeError(f"unexpected WHO_AM_I 0x{who:02X} at 0x{self.addr:02X}")
        self.bus.write_byte_data(self.addr, 0x6B, 0x00)   # wake, internal oscillator
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x1A, 0x03)   # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, 0x1B, 0x00)   # gyro +-250 dps
        self.bus.write_byte_data(self.addr, 0x1C, 0x00)   # accel +-2 g

    @staticmethod
    def _s16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def _read(self):
        d = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
        ax, ay, az = (self._s16(d[i], d[i + 1]) / 16384.0 for i in (0, 2, 4))
        gx, gy = (self._s16(d[i], d[i + 1]) / 131.0 for i in (8, 10))
        return ax, ay, az, gx, gy

    def _run(self):
        last = None
        while not self._stop:
            try:
                if self.bus is None:
                    self._open()
                    last = None
                ax, ay, az, gx, gy = self._read()
                now = time.time()
                acc_roll = math.degrees(math.atan2(ay, az))
                acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                if last is None:
                    self.roll, self.pitch = acc_roll, acc_pitch
                else:
                    dt = min(now - last, 0.2)
                    a = 0.98   # trust the gyro short-term, the accelerometer long-term
                    self.roll = a * (self.roll + gx * dt) + (1 - a) * acc_roll
                    self.pitch = a * (self.pitch + gy * dt) + (1 - a) * acc_pitch
                last = now
                self.updated = now
                self.ok, self.error = True, ""
                time.sleep(0.01)
            except Exception as e:  # I2C glitch, unplugged, library missing
                self.ok, self.error = False, str(e)
                try:
                    if self.bus:
                        self.bus.close()
                except Exception:
                    pass
                self.bus = None
                time.sleep(1)

    def fresh(self):
        return self.ok and time.time() - self.updated < 0.5
