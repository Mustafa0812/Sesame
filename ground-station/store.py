"""In-memory data shared by the MQTT subscriber, the REST API and the live stream."""
import threading
import time
from collections import deque

HISTORY = 7200   # samples kept per metric (2 h at 1 Hz)


class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.series = {"temperature": deque(maxlen=HISTORY), "pressure": deque(maxlen=HISTORY)}
        self.last = {}                    # metric -> (t, value)
        self.events = deque(maxlen=200)   # sequence start events for the simulator
        self._event_id = 0
        self.last_run = {}                # bot mac -> last telemetry 'run' value

    def add_metric(self, name, value):
        now = time.time()
        with self.lock:
            self.series.setdefault(name, deque(maxlen=HISTORY)).append((now, value))
            self.last[name] = (now, value)

    def metric(self, name, max_age=30):
        v = self.last.get(name)
        return v[1] if v and time.time() - v[0] < max_age else None

    def history(self, name, limit=900):
        with self.lock:
            pts = list(self.series.get(name, []))[-limit:]
        return [[round(t, 1), v] for t, v in pts]

    def add_event(self, target, name, offset_ms=0, source="cmd"):
        with self.lock:
            self._event_id += 1
            self.events.append({"id": self._event_id, "t": time.time(), "target": target,
                                "name": name, "offset": offset_ms, "source": source})
            return self._event_id

    def events_since(self, last_id):
        with self.lock:
            return [e for e in self.events if e["id"] > last_id]

    def latest_event_id(self):
        return self._event_id
