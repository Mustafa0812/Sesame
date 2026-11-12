"""BMP388 temperature + pressure publisher (MQTT), same pattern as the lab's sensor publishers.

Wiring (I2C, 3.3 V): VIN -> 3V3, GND -> GND, SDA -> GPIO2 (pin 3), SCL -> GPIO3 (pin 5).
Address 0x77 by default (SDO high), 0x76 if SDO is tied to GND. It can share the I2C bus with the MPU6500
(0x68). Enable I2C first: sudo raspi-config -> Interfaces -> I2C.

    pip install adafruit-circuitpython-bmp3xx
    python3 bmp388_publisher.py --host <broker> [--address 0x77] [--period 1]

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
    ap.add_argument("--address", default="0x77", help="I2C address, 0x77 or 0x76")
    ap.add_argument("--period", type=float, default=1.0, help="seconds between readings")
    a = ap.parse_args()

    import board                      # imported here so --help works on a PC
    import adafruit_bmp3xx

    sensor = adafruit_bmp3xx.BMP3XX_I2C(board.I2C(), address=int(a.address, 16))
    sensor.pressure_oversampling = 8
    sensor.temperature_oversampling = 2

    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:   # paho 1.x
        c = mqtt.Client()
    if a.user:
        c.username_pw_set(a.user, a.password)
    c.connect_async(a.host, a.port, 30)
    c.loop_start()
    print(f"BMP388 -> {a.host}:{a.port} {a.prefix}/env/temperature|pressure every {a.period:g} s")

    while True:
        try:
            t, p = sensor.temperature, sensor.pressure
            c.publish(f"{a.prefix}/env/temperature", f"{t:.2f}")
            c.publish(f"{a.prefix}/env/pressure", f"{p:.2f}")
            print(f"Published: {t:.2f} C, {p:.2f} hPa")
        except Exception as e:           # occasional I2C read error: retry, like the DHT11 lab script
            print("BMP388 read error:", e)
        time.sleep(a.period)


if __name__ == "__main__":
    main()
