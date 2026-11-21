"""Mission progress: which waypoints are done. Survives a service or autopilot restart.

Saved in service/state/progress.json (atomic write) every time a waypoint is reached.
The autopilot can't keep this for us: on reboot ArduPilot keeps the mission in storage but
resets to the first waypoint (AP_Mission::init -> reset).

Index convention: `next_wp` indexes mission["waypoints"]. We upload [home] + waypoints, so
autopilot seq k == mission waypoint k - 1.

mission_id = hash of the waypoint list: a different route starts from zero.
"""
import hashlib
import json
import os
from pathlib import Path


def mission_id(mission):
    wps = json.dumps(mission["waypoints"], sort_keys=True)
    return hashlib.sha1(wps.encode()).hexdigest()[:12]


class Progress:
    def __init__(self, path, mission, fresh=False):
        self.path = Path(path)
        self.mission_id = mission_id(mission)
        self.n_wp = len(mission["waypoints"])
        self.next_wp = 0
        self.reached = []                       # [{"wp": i, "t": autopilot unix s}]
        if fresh:
            print(f"progress: --fresh -> mission {self.mission_id} from waypoint 0")
            self.save()                         # overwrite the old file now
        else:
            self._load()

    def _load(self):
        if not self.path.exists():
            print(f"progress: none saved -> mission {self.mission_id} from waypoint 0")
            return
        saved = json.loads(self.path.read_text())
        if saved.get("mission_id") != self.mission_id:
            print(f"progress: saved file is for mission {saved.get('mission_id')}, "
                  f"this is {self.mission_id} -> from waypoint 0")
            return
        self.next_wp = int(saved["next_wp"])
        self.reached = saved.get("reached", [])
        print(f"progress: loaded, next waypoint {self.next_wp}/{self.n_wp}")

    @property
    def done(self):
        return self.next_wp >= self.n_wp

    def on_reached(self, seq, t_unix):
        """Autopilot reached mission item `seq`. Only moves forward; repeats/old seqs are ignored."""
        wp = seq - 1                            # seq 0 = home
        if wp < self.next_wp or wp >= self.n_wp:
            return False
        self.next_wp = wp + 1
        self.reached.append({"wp": wp, "t": int(t_unix)})
        self.save()
        print(f"progress: reached waypoint {wp} -> next {self.next_wp}/{self.n_wp}")
        return True

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"mission_id": self.mission_id, "next_wp": self.next_wp,
                                   "n_wp": self.n_wp, "reached": self.reached}, indent=1))
        os.replace(tmp, self.path)
