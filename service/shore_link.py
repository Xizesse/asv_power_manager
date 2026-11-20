"""Shore link (boat side): store and forward telemetry to the shore server.

The control loop calls `send(record)`; a background thread POSTs queued records
to shore in batches. If shore can't be reached (link down), records stay queued
and go out on the next successful contact. The boat always starts the exchange,
like a satellite modem would.

Telemetry is a queue (every record matters). The plan is a single slot: only the
latest plan matters, it is re-sent until shore acks it (POST /api/plan).

Queue kept in memory. Later: persist the queue to disk and read a pending mission
from the server's reply.
"""
import json
import threading
import time
import urllib.request
from collections import deque

MAX_QUEUE = 10_000        # oldest records are dropped beyond this
MAX_BATCH = 500


class ShoreLink:
    def __init__(self, url="http://127.0.0.1:8000/api/telemetry", every_s=2.0):
        self.url = url
        self.every_s = every_s        # wall-clock contact interval
        self.queue = deque(maxlen=MAX_QUEUE)
        self.lock = threading.Lock()
        self.plan_url = url.rsplit("/api/", 1)[0] + "/api/plan"
        self.pending_plan = None      # latest plan not yet acked by shore
        self.link_up = False

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, record):
        with self.lock:
            self.queue.append(record)

    def send_plan(self, plan):
        """Replace any unsent plan with this one; it goes out on the next contact."""
        with self.lock:
            self.pending_plan = plan

    def _post(self, url, payload):
        req = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=10).read()

    def _run(self):
        while True:
            time.sleep(self.every_s)
            with self.lock:
                batch = list(self.queue)[:MAX_BATCH]
                plan = self.pending_plan
            if not batch and plan is None:
                continue
            try:
                if plan is not None:
                    self._post(self.plan_url, plan)
                    with self.lock:
                        if self.pending_plan is plan:   # a newer plan may have arrived meanwhile
                            self.pending_plan = None
                    print(f"shore_link: plan {plan.get('plan_id')} delivered")
                if batch:
                    self._post(self.url, batch)
            except OSError as e:
                if self.link_up:
                    print(f"shore_link: link DOWN ({e}); {len(self.queue)} records queued")
                self.link_up = False
                continue
            with self.lock:                       # delivered: drop what we sent
                for _ in range(len(batch)):
                    self.queue.popleft()
            if not self.link_up:
                print(f"shore_link: link UP, delivered {len(batch)} records")
            self.link_up = True
