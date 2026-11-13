"""MPU6500 reader for the Raspberry Pi (SPI0, CE0) with a complementary tilt filter.

Wiring (Pi 3.3 V logic only), board labels as on GY-6500 / MPU-9250 breakouts:
    VCC        -> 3V3  (pin 1)       GND -> GND (pin 6)
    SCL / SCLK -> GPIO11 SCLK (pin 23)
    SDA / SDI  -> GPIO10 MOSI (pin 19)
    AD0 / SDO  -> GPIO9  MISO (pin 21)
    NCS        -> GPIO8  CE0  (pin 24)   chip select, driven by the kernel SPI driver
Enable SPI first: sudo raspi-config -> Interfaces -> SPI, then reboot (check: ls /dev/spidev0.0).
SPI needs an MPU6500 or MPU9250. The MPU6050 (GY-521) is I2C only and will not answer here.
"""
import math
import threading
import time

try:
    import spidev
except ImportError:  # lets the rest of the app start on a PC without the library
    spidev = None

WHO_AM_I_OK = (0x70, 0x71, 0x73)  # MPU6500, MPU9250, MPU9255 (all have SPI)

REG_USER_CTRL = 0x6A
I2C_IF_DIS = 0x10      # USER_CTRL bit 4: turn the I2C interface off so the chip stays in SPI mode


class Imu:
    def __init__(self, bus=0, device=0, speed_hz=1_000_000):
        # 1 MHz is the MPU6500's limit for writing config registers; fast enough for 100 Hz reads.
        self.bus_no, self.device, self.speed_hz = bus, device, speed_hz
        self.spi = None
        self.pitch = 0.0   # degrees, + = nose down toward +X of the chip (see invert flags in guidance)
        self.roll = 0.0
        self.ok = False
        self.error = "not started"
        self.updated = 0.0
        self._stop = False

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    # SPI register access: bit 7 of the first byte is 1 for read, 0 for write.
    def _write(self, reg, val):
        self.spi.xfer2([reg & 0x7F, val])

    def _read_regs(self, reg, n):
        return self.spi.xfer2([reg | 0x80] + [0] * n)[1:]

    def _open(self):
        if spidev is None:
            raise RuntimeError("spidev not installed (pip install spidev)")
        self.spi = spidev.SpiDev()
        self.spi.open(self.bus_no, self.device)
        self.spi.max_speed_hz = self.speed_hz
        self.spi.mode = 3
        self._write(REG_USER_CTRL, I2C_IF_DIS)
        who = self._read_regs(0x75, 1)[0]
        if who not in WHO_AM_I_OK:
            raise RuntimeError(f"unexpected WHO_AM_I 0x{who:02X} on /dev/spidev{self.bus_no}.{self.device} "
                               "(0x00/0xFF = no answer: check wiring and NCS on CE0)")
        self._write(0x6B, 0x00)   # wake, internal oscillator
        time.sleep(0.1)
        self._write(0x1A, 0x03)   # DLPF ~44 Hz
        self._write(0x1B, 0x00)   # gyro +-250 dps
        self._write(0x1C, 0x00)   # accel +-2 g

    @staticmethod
    def _s16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def _read(self):
        d = self._read_regs(0x3B, 14)
        ax, ay, az = (self._s16(d[i], d[i + 1]) / 16384.0 for i in (0, 2, 4))
        gx, gy = (self._s16(d[i], d[i + 1]) / 131.0 for i in (8, 10))
        return ax, ay, az, gx, gy

    def _run(self):
        last = None
        while not self._stop:
            try:
                if self.spi is None:
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
            except Exception as e:  # SPI glitch, unplugged, library missing
                self.ok, self.error = False, str(e)
                try:
                    if self.spi:
                        self.spi.close()
                except Exception:
                    pass
                self.spi = None
                time.sleep(1)

    def fresh(self):
        return self.ok and time.time() - self.updated < 0.5
