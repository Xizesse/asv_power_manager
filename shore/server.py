"""Shore server: mailbox for boat telemetry + the web page.

  POST /api/telemetry   boat sends records (appended to shore/data/telemetry.jsonl). Each record says
                        which link brought it ("link": starlink | iridium).
                          Both: {"link", "boat": status, "records": [...]}. The REPLY carries shore -> boat
                          messages in "to_boat": the staged mission + forecast until the boat reports
                          that mission id, then the start command until the boat reports started.
                          Both are one-shot: once confirmed they are never sent again, so a boat started
                          with --fresh waits for a new "Send mission" instead of getting the old one.
  GET  /api/telemetry   page reads them back: ONE record per minute ("t"). A Starlink record (full)
                        replaces an Iridium one (5 fields) for the same minute; repeats are dropped.
                        last_contact = wall time of the last POST per link.
  POST /api/plan        boat sends {"plan", "prediction"} (one POST, made by the same planner run):
                          plan        the decision (summary + hourly speeds) -> shore/data/plan.json
                                      (history in plans.jsonl)
                          prediction  per minute: battery V, speed, load -> shore/data/prediction.json
  GET  /api/plan        page reads the latest plan ({} if none)
  GET  /api/prediction  page reads the latest prediction ({} if none), plus the planned solar per
                        minute ("solar_w"), worked out here from the forecast shore sent with the mission
                        (data/weather/forecast.csv) with the same rule the boat uses (service/problem.py)
  POST /api/reset       start fresh: telemetry.jsonl is moved to data/archive/, plan.json,
                        prediction.json, plans.jsonl and the staged mission (outbox.json) are deleted
                        (the boat re-sends its plan when it next replans). Safe while the boat runs a
                        mission: Abort falls back to the mission the boat reports.
  GET  /api/missions    missions the operator can send (shore/missions/mission*.json), with a short summary
  POST /api/mission     {"name": ...}: stage that mission + data/weather/forecast.csv for the boat
                        (new mission id each time; data/outbox.json)
  POST /api/mission/start   ask the boat to start the staged mission
  POST /api/mission/abort   ask the boat to drop the staged mission (or, if none, the one it reports):
                        the boat deletes mission, forecast, progress and plan -> WAITING_FOR_MISSION.
                        Not delivered yet -> simply never sent.
  Commands travel on both links: Starlink replies carry mission / start / abort; Iridium replies carry
  wake (Starlink on, a mission is waiting) / start / abort. Both links bring the boat's status.
  GET  /api/mission     the staged mission (no forecast text) + the boat's last reported status
  GET  /                the web page (shore/index.html)

The boat never talks to this server directly: it goes through shore/proxy.py (:8001 Starlink,
:8002 Iridium). Plans only come over Starlink.

Run:
    pip install flask
    python3 shore/server.py            # then open http://127.0.0.1:8000
"""
import json
import math
import sys
import time
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "service"))
import problem  # noqa: E402  service/problem.py: the boat's solar rule (forecast -> W per minute)

DATA = HERE / "data" / "telemetry.jsonl"
PLAN = HERE / "data" / "plan.json"
PLANS = HERE / "data" / "plans.jsonl"
PREDICTION = HERE / "data" / "prediction.json"
OUTBOX = HERE / "data" / "outbox.json"        # mission staged for the boat
BOAT = HERE / "data" / "boat.json"            # boat's last reported status
MISSIONS_DIR = HERE / "missions"            # operator missions (what "Send mission" offers)
FORECAST = HERE.parent / "data" / "weather" / "forecast.csv"
DATA.parent.mkdir(exist_ok=True)

app = Flask(__name__)
last_contact = {"starlink": None, "iridium": None}   # wall time of the last POST per link
RANK = {"iridium": 0, "starlink": 1}                    # higher wins for the same minute


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def write_json(path, obj):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(obj))
    tmp.replace(path)


@app.post("/api/telemetry")
def receive():
    body = request.get_json(force=True)
    if isinstance(body, dict):                       # {"link", "boat": status, "records"}; reply carries to_boat
        records, boat = body.get("records", []), body.get("boat") or {}
        link = body.get("link", "starlink")
        write_json(BOAT, dict(boat, seen_t=time.time(), via=link))
    else:                                            # old Iridium format: plain list, no status
        records, boat, link = body, None, (body[0].get("link", "iridium") if body else "iridium")
    with open(DATA, "a") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")
    last_contact[link] = time.time()
    return jsonify(ok=True, received=len(records), to_boat=to_boat(boat, link))


def to_boat(boat, link):
    """What shore still has to tell the boat. Confirmation comes from the boat's own status (from either
    link), so a lost reply is simply sent again on the next contact; once confirmed, never again.

    Starlink can carry everything. Iridium only short commands (an SBD message to the boat is ~270 B):
    instead of the mission it sends "wake" = switch Starlink on, a mission is waiting."""
    out = read_json(OUTBOX)
    if not boat or out is None:
        return {}
    has_it = boat.get("mission_id") == out["mission_id"]
    changed = False
    if has_it and not out.get("delivered"):
        out["delivered"], changed = True, True
    if has_it and boat.get("started") and not out.get("started"):
        out["started"], changed = True, True
    if out.get("abort") and not has_it and not out.get("aborted"):
        out["aborted"], changed = True, True       # boat no longer holds it (or never got it)
        PLAN.unlink(missing_ok=True)                 # the page stops showing the aborted plan
        PREDICTION.unlink(missing_ok=True)           # (history stays in plans.jsonl)
    if changed:
        write_json(OUTBOX, out)
    if out.get("abort"):
        return {"abort": out["mission_id"]} if has_it else {}
    if not out.get("delivered"):
        if link == "iridium":
            return {"wake": out["mission_id"]}
        return {"mission": {k: out[k] for k in ("mission_id", "name", "mission", "forecast_csv")}}
    if has_it and out.get("start") and not out.get("started"):
        return {"start": out["mission_id"]}
    return {}


def route_km(wps):
    R, km = 6371.0, 0.0
    for a, b in zip(wps, wps[1:]):
        p1, p2 = math.radians(a["lat"]), math.radians(b["lat"])
        dp, dl = p2 - p1, math.radians(b["lon"] - a["lon"])
        h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        km += 2 * R * math.asin(math.sqrt(h))
    return km


@app.get("/api/missions")
def missions():
    out = []
    for f in sorted(MISSIONS_DIR.glob("mission*.json")):
        m = json.loads(f.read_text())
        out.append({"name": f.stem, "waypoints": len(m["waypoints"]),
                    "route_km": round(route_km(m["waypoints"]), 1),
                    "arrive_by": m.get("arrive_by"), "reserve_v": m.get("reserve_v"),
                    "speed_ms": m.get("speed_ms")})
    return jsonify(missions=out)


@app.post("/api/mission")
def stage_mission():
    name = (request.get_json(force=True) or {}).get("name", "")
    f = MISSIONS_DIR / f"{name}.json"
    if not name.startswith("mission") or "/" in name or not f.exists():
        return jsonify(ok=False, error=f"no mission {name!r}"), 404
    out = {"mission_id": f"{name}-{time.strftime('%Y%m%dT%H%M%S')}", "name": name,
           "mission": json.loads(f.read_text()), "forecast_csv": FORECAST.read_text(),
           "staged_t": time.time(), "start": False, "delivered": False, "started": False}
    write_json(OUTBOX, out)
    return jsonify(ok=True, mission_id=out["mission_id"])


@app.post("/api/mission/start")
def start_mission():
    out = read_json(OUTBOX)
    if out is None or out.get("abort"):
        return jsonify(ok=False, error="no mission staged"), 409
    out["start"] = True
    write_json(OUTBOX, out)
    return jsonify(ok=True, mission_id=out["mission_id"])


@app.post("/api/mission/abort")
def abort_mission():
    """Abort the boat's mission: the staged one, or (if nothing is staged, e.g. shore data was deleted)
    whatever mission the boat last reported. Sent over both links until the boat reports it gone."""
    out = read_json(OUTBOX)
    boat = read_json(BOAT) or {}
    if out is None or out.get("aborted"):
        if not boat.get("mission_id"):
            return jsonify(ok=False, error="no mission to abort"), 409
        out = {"mission_id": boat["mission_id"], "name": boat.get("name"), "staged_t": time.time(),
               "start": False, "delivered": True, "started": bool(boat.get("started"))}
    out["abort"] = True
    write_json(OUTBOX, out)
    return jsonify(ok=True, mission_id=out["mission_id"])


@app.get("/api/mission")
def mission_status():
    out = read_json(OUTBOX)
    if out is not None:
        out = {k: v for k, v in out.items() if k not in ("forecast_csv", "mission")}
    return jsonify(staged=out, boat=read_json(BOAT))


@app.get("/api/telemetry")
def telemetry():
    by_t = {}
    if DATA.exists():
        with open(DATA) as f:
            for line in f:
                if not line.strip():
                    continue
                r = json.loads(line)
                r.setdefault("link", "starlink")
                old = by_t.get(r["t"])
                if old is None or RANK.get(r["link"], 0) >= RANK.get(old["link"], 0):
                    by_t[r["t"]] = r
    records = sorted(by_t.values(), key=lambda r: r["t"])
    return jsonify(records=records, last_contact=last_contact)


@app.post("/api/plan")
def receive_plan():
    msg = request.get_json(force=True)
    plan, prediction = msg["plan"], msg["prediction"]
    write_json(PREDICTION, prediction)
    write_json(PLAN, plan)
    with open(PLANS, "a") as f:
        f.write(json.dumps(plan) + "\n")
    last_contact["starlink"] = time.time()
    return jsonify(ok=True, plan_id=plan.get("plan_id"))


@app.get("/api/plan")
def latest_plan():
    return jsonify(read_json(PLAN, {}))


_solar_cache = {}   # plan_id -> planned solar (W per minute)


def planned_solar(plan, n):
    """Planned solar for the plan's first n minutes: the boat's rule (service/problem.py) on our forecast."""
    key = plan.get("plan_id")
    if key not in _solar_cache:
        try:
            p_solar = problem.Problem(plan["mission"], problem.read_forecast(FORECAST)).p_solar
            _solar_cache[key] = [round(float(x), 1) for x in p_solar]
        except (ValueError, KeyError, FileNotFoundError) as e:
            print(f"planned solar: not available ({e})", flush=True)
            _solar_cache[key] = None
    sol = _solar_cache[key]
    return None if sol is None else sol[:n]


@app.get("/api/prediction")
def latest_prediction():
    pred, plan = read_json(PREDICTION, {}), read_json(PLAN, {})
    if pred and plan and pred.get("plan_id") == plan.get("plan_id"):
        pred["series"]["solar_w"] = planned_solar(plan, len(pred["series"]["t"]))
    return jsonify(pred)


@app.post("/api/reset")
def reset():
    deleted = [p.name for p in (PLAN, PLANS, PREDICTION, OUTBOX) if p.exists()]
    for p in (PLAN, PLANS, PREDICTION, OUTBOX):
        p.unlink(missing_ok=True)
    if not DATA.exists():
        return jsonify(ok=True, archived=None, deleted=deleted)
    archive = DATA.parent / "archive"
    archive.mkdir(exist_ok=True)
    dest = archive / f"telemetry-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    DATA.replace(dest)              # archived, not deleted: an accidental click loses nothing
    return jsonify(ok=True, archived=str(dest.relative_to(HERE)), deleted=deleted)


@app.get("/")
def index():
    resp = send_from_directory(HERE, "index.html")
    resp.headers["Cache-Control"] = "no-store"     # always the current page, never a stale cached copy
    return resp


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8000)
