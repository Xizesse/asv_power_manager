"""Links to shore (boat side): Starlink and Iridium, both store and forward.

The boat always starts the exchange (like a satellite modem). Each link has its own queue and its own
background thread; the control loop only hands records over and never waits for shore.

  StarlinkLink  -> proxy on :8001 -> shore :8000. Only talks while the Starlink terminal is ON
                   (measured relay output). Boat -> shore: the full 1-minute records, the plan, and the
                   boat's status (state, which mission it holds, started?). Shore -> boat: in the REPLY
                   to that same POST (the boat never listens on a port): a new mission + forecast, or
                   the start command. It contacts shore every few seconds even with no data, to poll.
  IridiumLink   -> proxy on :8002 -> shore :8000. Always on. Every 5 one-minute bins (autopilot time)
                   it sends ONE small message with those 5 points: voltage, load, solar, speed, plus
                   the latest position and the waypoint the autopilot is heading to.

Killing a proxy = that link is down. Records stay queued and go out on the next successful contact.
Shore keeps one record per minute: a Starlink record replaces an Iridium one for the same minute.

Queues are kept in memory (lost if the service restarts).
"""
import json
import threading
import time
import urllib.request
from collections import deque

MAX_QUEUE = 10_000        # oldest items are dropped beyond this (10,000 minutes ~ 7 days)
MAX_BATCH = 500

IRIDIUM_EVERY_BINS = 5    # one message per 5 one-minute bins = every 5 min of autopilot time
IRIDIUM_KEYS = ("t", "batt_v", "load_w", "solar_w", "speed")


class Link:
    """Queue + sender thread. Subclasses say when the link is usable and what goes out."""
    name = "link"

    def __init__(self, base_url, every_s=1.0):
        self.base_url = base_url.rstrip("/")
        self.every_s = every_s        # wall-clock contact interval
        self.queue = deque(maxlen=MAX_QUEUE)
        self.lock = threading.Lock()
        self.link_up = None           # None = no contact tried yet

    def start(self):
        threading.Thread(target=self._run, daemon=True, name=self.name).start()

    def available(self):
        """Is the radio on? (Not: is shore reachable. That we only learn by trying.)"""
        return True

    def _post(self, path, payload):
        """POST JSON, return shore's JSON reply ({} if it isn't JSON)."""
        req = urllib.request.Request(self.base_url + path, data=json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json"})
        body = urllib.request.urlopen(req, timeout=10).read()
        try:
            return json.loads(body)
        except ValueError:
            return {}

    def _deliver(self):
        """Send what is pending. Raise on failure. Return a short text for the log, or None if nothing was sent."""
        with self.lock:
            batch = list(self.queue)[:MAX_BATCH]
        if not batch:
            return None
        self._post("/api/telemetry", batch)
        with self.lock:                       # delivered: drop what we sent
            for _ in range(len(batch)):
                self.queue.popleft()
        return f"{len(batch)} records"

    def _run(self):
        while True:
            time.sleep(self.every_s)
            if not self.available():
                continue
            try:
                sent = self._deliver()
            except Exception as e:            # any error: stay alive, retry on the next contact
                if self.link_up is not False:
                    print(f"{self.name}: link DOWN ({e}); {len(self.queue)} queued", flush=True)
                self.link_up = False
                continue
            if sent and not self.link_up:
                print(f"{self.name}: link UP, delivered {sent}", flush=True)
            if sent:
                self.link_up = True


class StarlinkLink(Link):
    """Full telemetry + the plan + boat status, only while the Starlink terminal is on.

    Telemetry is a queue (every record matters). The plan is a single slot: only the latest plan
    matters, it is re-sent until shore acks it (then `plan_acked` = its id).
    report()   -> dict sent with every contact (boat state, mission id, started?)
    on_reply() <- shore's reply to each telemetry POST, for the control loop (mission / start command)"""
    name = "starlink"

    def __init__(self, base_url="http://127.0.0.1:8001", is_on=lambda: True, report=dict,
                 on_reply=lambda reply: None, every_s=1.0):
        super().__init__(base_url, every_s)
        self.is_on = is_on            # e.g. lambda: bridge.snapshot().starlink_on
        self.report = report
        self.on_reply = on_reply
        self.pending_plan = None
        self.plan_acked = None        # id of the last plan shore confirmed

    def available(self):
        return bool(self.is_on())

    def send(self, record):
        with self.lock:
            self.queue.append(dict(record, link="starlink"))

    def send_plan(self, plan):
        """Replace any unsent plan with this one; it goes out on the next contact."""
        with self.lock:
            self.pending_plan = plan

    def _deliver(self):
        with self.lock:
            plan = self.pending_plan
        if plan is not None:
            self._post("/api/plan", plan)
            with self.lock:
                if self.pending_plan is plan:     # a newer plan may have arrived meanwhile
                    self.pending_plan = None
            self.plan_acked = plan.get("plan_id")
            print(f"{self.name}: plan {plan.get('plan_id')} delivered", flush=True)
        # every contact = one telemetry POST (possibly with no records): it carries our status and
        # its reply is how shore talks to us
        with self.lock:
            batch = list(self.queue)[:MAX_BATCH]
        reply = self._post("/api/telemetry", {"boat": self.report(), "records": batch})
        with self.lock:
            for _ in range(len(batch)):
                self.queue.popleft()
        self.on_reply(reply)
        parts = ([f"plan {plan.get('plan_id')}"] if plan is not None else []) + \
                ([f"{len(batch)} records"] if batch else [])
        return " + ".join(parts) or "status"



class IridiumLink(Link):
    """Small status messages: every 5 one-minute bins, those 5 points in one message."""
    name = "iridium"

    def __init__(self, base_url="http://127.0.0.1:8002", every_s=1.0):
        super().__init__(base_url, every_s)
        self.bins = []                # bins collected for the next message

    def add_bin(self, record):
        """Feed every finished 1-minute bin. Every IRIDIUM_EVERY_BINS bins, one message is queued."""
        self.bins.append({k: record.get(k) for k in IRIDIUM_KEYS})
        if len(self.bins) < IRIDIUM_EVERY_BINS:
            return
        # position and waypoint once per message (on the last point): saves bytes on a slow link
        self.bins[-1].update(lat=record.get("lat"), lon=record.get("lon"), wp=record.get("wp"))
        msg = [dict(b, link="iridium") for b in self.bins]
        self.bins = []
        with self.lock:
            self.queue.append(msg)

    def _deliver(self):
        """One queued message = one POST (a list of 5 records), oldest first."""
        n = 0
        while True:
            with self.lock:
                if not self.queue:
                    break
                msg = self.queue[0]
            self._post("/api/telemetry", msg)
            with self.lock:
                self.queue.popleft()
            n += 1
        return f"{n} messages" if n else None
