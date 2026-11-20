# scout_telemetry_2025-07.csv

Raw telemetry from a Scout in the northern North Sea, 14–30 July 2025, as stored by Tether. Positions and times are shifted from the real deployment. One row per message received.

| Column | Meaning |
| --- | --- |
| `created_at` | Time Tether received the message (UTC) |
| `communication_mode` | Link it arrived over: `direct` (Starlink), `imt` (Iridium), `globalstar` (backup tracker) |
| `latitude_deg`, `longitude_deg` | GPS position |
| `heading_deg`, `course_deg` | Heading and course over ground (degrees) |
| `groundspeed_ms` | Speed over ground (m/s) |
| `custom_mode` | ArduPilot Rover flight mode number |
| `custom0_arming` | 1 = armed, 0 = disarmed |
| `wp_num`, `wp_latitude_deg`, `wp_longitude_deg` | Active mission waypoint |
| `voltage_battery1_v`, `current_battery1_a` | Battery (V, A) |
| `voltage_battery2_v`, `current_battery2_a` | Labelled "thruster" in Tether (V, A) |
| `voltage_battery3_v`, `current_battery3_a` | Solar input (V, A) |
| `temperature_c` | Internal temperature (°C) |
| `battery_pct` | Battery state of charge as shown in Tether (%) |

`null` means the field wasn't in that message.
