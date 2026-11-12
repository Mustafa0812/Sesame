"""Optional InfluxDB 1.x logging, as in the lab (database SRH_ES_iot). Enabled when INFLUX_HOST is set.

Environment: INFLUX_HOST, INFLUX_PORT (8086), INFLUX_DB (SRH_ES_iot), INFLUX_USER, INFLUX_PASS
Measurements:  env (sensor=bmp388: temperature, pressure)   tilt (pitch, roll)   bot (tag id: volts, ma, pct, rssi)
Needs: pip install influxdb
"""
import os
import threading
import time


def start(store, tilt_fn, gw):
    from influxdb import InfluxDBClient   # imported here so the app runs without it

    db = os.environ.get("INFLUX_DB", "SRH_ES_iot")
    client = InfluxDBClient(host=os.environ["INFLUX_HOST"], port=int(os.environ.get("INFLUX_PORT", "8086")),
                            username=os.environ.get("INFLUX_USER") or None,
                            password=os.environ.get("INFLUX_PASS") or None)
    client.create_database(db)
    client.switch_database(db)

    def loop():
        while True:
            time.sleep(1)
            pts = []
            env = {k: store.metric(k) for k in ("temperature", "pressure")}
            env = {k: v for k, v in env.items() if v is not None}
            if env:
                pts.append({"measurement": "env", "tags": {"sensor": "bmp388"}, "fields": env})
            tl = tilt_fn()
            if tl:
                pts.append({"measurement": "tilt", "tags": {"source": tl["src"]},
                            "fields": {"pitch": float(tl["pitch"]), "roll": float(tl["roll"])}})
            for b in gw.snapshot()["bots"]:
                t = b.get("tlm") or {}
                if b.get("id") and b["online"] and t:
                    try:
                        pts.append({"measurement": "bot", "tags": {"id": b["id"], "role": t.get("role", "?")},
                                    "fields": {"volts": float(t.get("v", 0)), "ma": float(t.get("ma", 0)),
                                               "pct": float(t.get("pct", -1)), "rssi": float(b["rssi"])}})
                    except ValueError:
                        pass
            if pts:
                try:
                    client.write_points(pts)
                except Exception as e:
                    print("InfluxDB write failed:", e)

    threading.Thread(target=loop, daemon=True).start()
    print(f"InfluxDB logging to {os.environ['INFLUX_HOST']} database {db}")
