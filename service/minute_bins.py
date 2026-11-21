"""1-minute averages of the boat state, for shore telemetry.

Same bins as the plan series (service/planner.py, SERIES_EVERY_MIN = 1): one bin per whole minute of
autopilot time, labelled by the minute's start ("t"), value = mean over that minute. So actual and plan
are compared like for like.

One sample per new autopilot time (SYSTEM_TIME, and everything else, streams at 1 Hz), so a full
minute holds ~60 samples; "n" says how many. A bin is sent when the first sample of the next minute
arrives, so shore is always one minute behind.

  mean of : speed, throttle_pct, starlink (fraction of the minute on, 0..1),
            batt_v, batt_a, load_w, solar_w (only samples with fresh battery data; None if none)
  last of : mode, armed, lat, lon
"""
import math

MEAN_KEYS = ("speed", "throttle_pct", "starlink")
BATT_KEYS = ("batt_v", "batt_a", "load_w", "solar_w")
ROUND = {"speed": 2, "throttle_pct": 1, "starlink": 2, "batt_v": 3, "batt_a": 2, "load_w": 1, "solar_w": 1}


class MinuteBins:
    def __init__(self):
        self.minute = None      # start of the current bin (unix s)
        self.last_t = None      # autopilot time of the last sample
        self._reset()

    def _reset(self):
        self.n = 0
        self.sums = dict.fromkeys(MEAN_KEYS, 0.0)
        self.batt_n = 0
        self.batt_sums = dict.fromkeys(BATT_KEYS, 0.0)
        self.last = {}

    def add(self, state):
        """Feed one BoatState snapshot. Returns the finished previous bin (a dict) when a new minute
        starts, else None. Repeated snapshots with the same autopilot time are ignored."""
        if not state.t_unix or state.t_unix == self.last_t:
            return None
        self.last_t = state.t_unix
        minute = math.floor(state.t_unix / 60) * 60
        done = None
        if self.minute is not None and minute != self.minute and self.n:
            done = self._record()
        if minute != self.minute:
            self.minute = minute
            self._reset()

        self.n += 1
        self.sums["speed"] += state.speed
        self.sums["throttle_pct"] += state.throttle_pct
        self.sums["starlink"] += float(state.starlink_on)
        if state.battery_fresh():
            self.batt_n += 1
            self.batt_sums["batt_v"] += state.batt_v
            self.batt_sums["batt_a"] += state.batt_a
            self.batt_sums["load_w"] += state.load_v * state.load_a
            self.batt_sums["solar_w"] += state.solar_v * state.solar_a
        self.last = {"mode": state.mode, "armed": state.armed, "lat": state.lat, "lon": state.lon}
        return done

    def _record(self):
        rec = {"t": self.minute, "n": self.n, **self.last}
        for k in MEAN_KEYS:
            rec[k] = round(self.sums[k] / self.n, ROUND[k])
        for k in BATT_KEYS:   # no fresh battery data in this minute -> null, never a fake 0
            rec[k] = round(self.batt_sums[k] / self.batt_n, ROUND[k]) if self.batt_n else None
        return rec
