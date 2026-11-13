"""BMP388 temperature + pressure publisher (MQTT), same pattern as the lab's sensor publishers.

Wiring (I2C bus 1, 3.3 V):
    VIN -> 3V3 (pin 17)       GND -> GND (pin 9)     (pins 1/6 are taken by the MPU6500)
    SCK -> GPIO3 SCL (pin 5)
    SDI -> GPIO2 SDA (pin 3)
    SDO -> leave open or 3V3 = address 0x77 (default)   SDO -> GND = address 0x76 (pass --addr 0x76)
    CS  -> leave open on Adafruit boards (pulled up), else tie to 3V3. CS low at power-up locks it in SPI mode.
The MPU6500 is on SPI (see imu.py), so the two sensors do not share a bus.
Enable I2C first: sudo raspi-config -> Interfaces -> I2C, then reboot. Check: sudo i2cdetect -y 1 shows 77 (or 76).

    pip install adafruit-circuitpython-bmp3xx
    python3 bmp388_publisher.py --host <broker> [--addr 0x77] [--period 1]

Publishes plain numbers (like the lab):  <prefix>/env/temperature  (deg C)   <prefix>/env/pressure  (hPa)
"""
import argparse
import os
import time

import paho.mqtt.client as mqtt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("MQTT_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    ap.add_argument("--user", default=os.environ.get("MQTT_USER"))
    ap.add_argument("--password", default=os.environ.get("MQTT_PASS", ""))
    ap.add_argument("--prefix", default=os.environ.get("MQTT_PREFIX", "sesame"))
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x77,
                    help="I2C address: 0x77 (SDO high/open, default) or 0x76 (SDO to GND)")
    ap.add_argument("--period", type=float, default=1.0, help="seconds between readings")
    a = ap.parse_args()

    import board                      # imported here so --help works on a PC
    import adafruit_bmp3xx

    sensor = adafruit_bmp3xx.BMP3XX_I2C(board.I2C(), address=a.addr)
    sensor.pressure_oversampling = 8
    sensor.temperature_oversampling = 2

    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:   # paho 1.x
        c = mqtt.Client()
    if a.user:
        c.username_pw_set(a.user, a.password)
    c.reconnect_delay_set(min_delay=1, max_delay=5)   # reconnect within seconds after a broker restart
    c.connect_async(a.host, a.port, 30)
    c.loop_start()
    print(f"BMP388 (I2C 0x{a.addr:02X}) -> {a.host}:{a.port} {a.prefix}/env/temperature|pressure every {a.period:g} s")

    while True:
        try:
            t, p = sensor.temperature, sensor.pressure
            c.publish(f"{a.prefix}/env/temperature", f"{t:.2f}")
            c.publish(f"{a.prefix}/env/pressure", f"{p:.2f}")
            print(f"Published: {t:.2f} C, {p:.2f} hPa")
        except Exception as e:           # occasional read error: retry, like the DHT11 lab script
            print("BMP388 read error:", e)
        time.sleep(a.period)


if __name__ == "__main__":
    main()
