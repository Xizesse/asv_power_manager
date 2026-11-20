"""Shore server: mailbox for boat telemetry + the web page.

  POST /api/telemetry   boat sends a JSON list of records (appended to shore/data/telemetry.jsonl)
  GET  /api/telemetry   page reads them back
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


@app.get("/")
def index():
    return send_from_directory(HERE, "index.html")


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8000)
