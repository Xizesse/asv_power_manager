#!/usr/bin/env bash
# Start the whole simulator: ArduPilot SITL Rover (boat) + the environment sim (env_sim.py).
# Must match fetch_weather.py (same lat/lon and start date). Ctrl+C stops both.
#
#   ./run_sitl.sh -w            first run, or after editing scout.parm (wipes saved params)
#   ./run_sitl.sh               new run: scenario start time/place, env sim battery reset
#   SPEEDUP=1 ./run_sitl.sh     real time
#   ./run_sitl.sh --resume      carry on after a kill: time, place and battery from
#                               sim/state/world.json (env_sim.py writes it every 10 s sim time).
#                               SITL starts RESUME_GAP_S (default 10) after the saved time, so the
#                               clock never goes back; the gap + boot is integrated by the env sim
#                               with whatever SITL reports (disarmed = motor off) if under 120 s.
#                               Never wipes eeprom, so the mission and params survive.
#
# Env sim output goes to sim/logs/env_sim.out  (tail -f it to watch).
set -euo pipefail

LAT=60.5
LON=1.5
START="2025-07-14T11:00:00Z"
SPEEDUP=${SPEEDUP:-10}   # target only; this laptop achieved ~20x
RESUME_GAP_S=${RESUME_GAP_S:-10}   # >= env_sim SAVE_EVERY_S, so resumed time is never before the kill

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_DIR="$SIM_DIR/sitl_run"          # eeprom, logs, mav.tlog live here (gitignore it)
# MAVProxy outputs: 14550 = QGC (default), 14551 = our service (service/bridge.py)
ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
VENV=${VENV:-$HOME/venv-ardupilot}
ENV_PY=${ENV_PY:-$SIM_DIR/../.venv/bin/python}   # env sim needs pandas (project venv)

STATE="$SIM_DIR/state/world.json"

ARGS=()
RESUME=0
for a in "$@"; do
  case "$a" in
    --resume) RESUME=1 ;;
    *) ARGS+=("$a") ;;
  esac
done

START_UNIX=$(date -u -d "$START" +%s)
if [[ $RESUME == 1 ]]; then
  [[ -f "$STATE" ]] || { echo "run_sitl: --resume but no $STATE" >&2; exit 1; }
  FILTERED=()
  for a in "${ARGS[@]}"; do
    if [[ "$a" == "-w" || "$a" == "--wipe-eeprom" ]]; then
      echo "run_sitl: ignoring $a on --resume (it would wipe the mission)" >&2
    else
      FILTERED+=("$a")
    fi
  done
  ARGS=("${FILTERED[@]}")
  read -r START_UNIX LAT LON < <(python3 -c '
import json, sys
s = json.load(open(sys.argv[1]))
lat = s["lat"] if s.get("lat") is not None else sys.argv[2]
lon = s["lon"] if s.get("lon") is not None else sys.argv[3]
print(int(s["t_unix"]) + int(sys.argv[4]), lat, lon)' "$STATE" "$LAT" "$LON" "$RESUME_GAP_S")
  echo "run_sitl: resuming at $(date -u -d "@$START_UNIX" +%FT%TZ)  $LAT,$LON"
fi
mkdir -p "$RUN_DIR" "$SIM_DIR/logs"

# env sim first: it retries until SITL is listening on tcp:5762
ENV_ARGS=()
[[ $RESUME == 1 ]] || ENV_ARGS=(--fresh)
(cd "$SIM_DIR/.." && exec "$ENV_PY" -u sim/env_sim.py "${ENV_ARGS[@]}") \
  >> "$SIM_DIR/logs/env_sim.out" 2>&1 &
ENV_PID=$!
trap 'kill $ENV_PID 2>/dev/null || true' EXIT
echo "run_sitl: env sim started (pid $ENV_PID${ENV_ARGS:+, ${ENV_ARGS[*]}}), log sim/logs/env_sim.out"

source "$VENV/bin/activate"
cd "$ARDUPILOT_DIR/Rover"
../Tools/autotest/sim_vehicle.py -v Rover -f motorboat \
  -l "$LAT,$LON,0,0" \
  --speedup "$SPEEDUP" \
  --use-dir "$RUN_DIR" \
  --add-param-file "$SIM_DIR/scout.parm" \
  -A "--start-time $START_UNIX" \
  --out "udp:127.0.0.1:14551" \
  --console --map "${ARGS[@]}"
