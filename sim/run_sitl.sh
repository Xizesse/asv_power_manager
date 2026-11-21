#!/usr/bin/env bash
# Start ArduPilot SITL Rover (boat) at the scenario place and time.
# Must match fetch_weather.py (same lat/lon and start date).
#
#   ./run_sitl.sh -w            first run, or after editing scout.parm (wipes saved params)
#   ./run_sitl.sh               normal run (50x target)
#   SPEEDUP=1 ./run_sitl.sh     real time
set -euo pipefail

LAT=60.5
LON=1.5
START="2025-07-14T11:00:00Z"
SPEEDUP=${SPEEDUP:-1}   # target only; this laptop achieved ~20x

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_DIR="$SIM_DIR/sitl_run"          # eeprom, logs, mav.tlog live here (gitignore it)
# MAVProxy outputs: 14550 = QGC (default), 14551 = our service (service/bridge.py)
ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
VENV=${VENV:-$HOME/venv-ardupilot}

START_UNIX=$(date -u -d "$START" +%s)
mkdir -p "$RUN_DIR"

source "$VENV/bin/activate"
cd "$ARDUPILOT_DIR/Rover"
exec ../Tools/autotest/sim_vehicle.py -v Rover -f motorboat \
  -l "$LAT,$LON,0,0" \
  --speedup "$SPEEDUP" \
  --use-dir "$RUN_DIR" \
  --add-param-file "$SIM_DIR/scout.parm" \
  -A "--start-time $START_UNIX" \
  --out "udp:127.0.0.1:14551" \
  --console --map "$@"
