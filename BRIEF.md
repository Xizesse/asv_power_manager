# Low-power mission mode for Scout

**Background.** Scout is a 2.4 m solar-electric robot boat that stays at sea for weeks. It runs SentryOS on a Linux computer alongside ArduPilot Rover. Today an operator watches the battery and adjusts speed, schedules and payloads by hand. We want a mode an operator can set and forget: given a mission (waypoints and/or a station to hold), Scout forecasts its energy, chooses how to operate, and arrives or keeps holding without running flat.

**Data.** `scout_telemetry_2025-07.csv` is the raw telemetry from a 17-day Scout deployment in the northern North Sea: position, speed over ground, autopilot mode and waypoint, and voltage and current for the battery, thruster and solar input. `DATA.md` lists the columns. Weather and ocean models are public: Copernicus Marine for currents, ERA5 or Open-Meteo for wind and cloud. Positions and dates in the data have been shifted, so public weather and ocean data won't match it: use them for realistic conditions, or infer what you can from the data itself.

**The task.**

1. Build a model of Scout's energy from the data: what it draws, what solar provides, and how wind and current move it. Your forecast must run from the command line from any start time in the data, e.g. `forecast --from 2025-07-23T00:00Z --hours 24`.
2. Build the low-power mode as a service that runs on the boat's Linux computer and talks MAVLink to ArduPilot. The operator sets waypoints or a station to hold, plus an arrival time or a minimum reserve, then leaves it. The service forecasts energy and decides how the boat operates. It must keep working when the link to shore is down and after the autopilot restarts.
3. Run the service against ArduPilot SITL (Rover), with your own simulation of solar input, wind and current drift, and thruster load. Add a simple web page that shows the forecast against what happened.
4. Package it as systemd services, unit files included, so it installs and runs with `systemctl`. Any Linux machine with systemd will do.
5. Prove it: one automated test of the decision logic, and one failure test (kill the autopilot mid-mission, and drop the link) with `journalctl` output showing the service recovered. Say how you'd roll it out to one boat first, and how you'd roll it back.

**Hand back** a repository we can run and a one-page write-up. Then a 45-minute call: you demo it running under systemd, we break something while it runs, and we ask it to forecast a period we choose. In the write-up and the call, cover:

- **Where the energy goes:** a breakdown by consumer, with numbers, and how you got each one from the data.
- **What surprised you in the data,** and what you did about it.
- **How good the forecast is:** its error on data you didn't fit to, and what you would and wouldn't trust it for.
- **What the mode decides:** what it changes, when, and why.
- **Where each part runs** (boat, shore, browser) and why, your backend and frontend choices, and what happens when the link or the autopilot drops.
- **What you chose not to build,** and what you'd want measured on a real boat next.

Use any tools you like, including AI. We care more about your judgement than the amount you build.
