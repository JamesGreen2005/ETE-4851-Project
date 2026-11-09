"""Thread-safe front/rear video cache. This does not authorize robot motion."""
import math
import threading
import time


class CameraFrames:
    def __init__(self, clock=time.monotonic, max_age=1.0):
        self.clock = clock
        self.max_age = max_age
        self.lock = threading.Lock()
        self.frames = {'front': None, 'rear': None}

    def update(self, name, jpeg, source_stamp, ros_now):
        # Arrival alone must not make a delayed or repeated ROS image look live.
        if not jpeg or not all(math.isfinite(t) for t in (source_stamp, ros_now)):
            return False
        if source_stamp <= 0 or not -.1 <= ros_now-source_stamp <= self.max_age:
            return False
        with self.lock:
            old = self.frames[name]
            if old is not None and source_stamp <= old[2]:
                return False
            self.frames[name] = (bytes(jpeg), self.clock(), source_stamp,
                                 max(0., ros_now-source_stamp))
        return True

    def _age(self, frame):
        return None if frame is None else max(0., self.clock()-frame[1])+frame[3]

    def get(self, name):
        with self.lock:
            frame = self.frames[name]
            return frame[0] if frame and self._age(frame) <= self.max_age else None

    def status(self):
        with self.lock:
            result = {}
            for name, frame in self.frames.items():
                age = self._age(frame)
                result[name] = dict(age_s=age, fresh=age is not None and age <= self.max_age,
                                    state='waiting' if age is None else
                                          'live' if age <= self.max_age else 'stale')
            return result
