"""Shore server: mailbox for boat telemetry + the web page.

  POST /api/telemetry   boat sends a JSON list of records (appended to shore/data/telemetry.jsonl)
  GET  /api/telemetry   page reads them back
  POST /api/plan        boat sends its latest plan (shore/data/plan.json; history in plans.jsonl)
  GET  /api/plan        page reads the latest plan ({} if none)
  POST /api/reset       start fresh: telemetry.jsonl is moved to data/archive/ (the plan is kept;
                        the next plan from the boat replaces it anyway)
  GET  /                the web page (shore/index.html)

Run:
    pip install flask
    python3 shore/server.py            # then open http://127.0.0.1:8000
"""
import json
import time
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

HERE = Path(__file__).parent
DATA = HERE / "data" / "telemetry.jsonl"
PLAN = HERE / "data" / "plan.json"
PLANS = HERE / "data" / "plans.jsonl"
DATA.parent.mkdir(exist_ok=True)

app = Flask(__name__)
last_contact = {"wall": None}


@app.post("/api/telemetry")
def receive():
    records = request.get_json(force=True)
    with open(DATA, "a") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")
    last_contact["wall"] = time.time()
    return jsonify(ok=True, received=len(records))


@app.get("/api/telemetry")
def telemetry():
    records = []
    if DATA.exists():
        with open(DATA) as f:
            records = [json.loads(line) for line in f if line.strip()]
    records.sort(key=lambda r: r["t"])
    return jsonify(records=records, last_contact=last_contact["wall"])


@app.post("/api/plan")
def receive_plan():
    plan = request.get_json(force=True)
    tmp = PLAN.with_suffix(".tmp")
    tmp.write_text(json.dumps(plan))
    tmp.replace(PLAN)
    with open(PLANS, "a") as f:
        f.write(json.dumps(plan) + "\n")
    last_contact["wall"] = time.time()
    return jsonify(ok=True, plan_id=plan.get("plan_id"))


@app.get("/api/plan")
def latest_plan():
    return jsonify(json.loads(PLAN.read_text()) if PLAN.exists() else {})


@app.post("/api/reset")
def reset():
    if not DATA.exists():
        return jsonify(ok=True, archived=None)
    archive = DATA.parent / "archive"
    archive.mkdir(exist_ok=True)
    dest = archive / f"telemetry-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    DATA.replace(dest)              # archived, not deleted: an accidental click loses nothing
    return jsonify(ok=True, archived=str(dest.relative_to(HERE)))


@app.get("/")
def index():
    return send_from_directory(HERE, "index.html")


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8000)
