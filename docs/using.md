# Screens

The screen is portrait: a **status bar** on top, the **radar** in the middle and
four **tabs** at the bottom.

## Status bar

- **Wi-Fi** state and signal strength
- **Feed**: server, protocol and messages per second (or the last error)
- **Aircraft count**: with position / total tracked
- **Clock**: local time with zone abbreviation, UTC underneath (or just UTC
  when the timezone is UTC)

## Radar

| Action | How |
|---|---|
| Select an aircraft | Tap it (opens the Details tab) |
| Deselect | Tap empty map |
| Pan | Drag |
| Zoom | `+` / `−` (5 to 400 nm) |
| Re-centre on the receiver | Location button |
| Labels on/off | `Aa` button |
| Trails on/off | Shuffle button |
| Map on/off | Picture button |

Aircraft are drawn as arrows pointing along their track, coloured by altitude
(legend at the bottom: orange near the ground through green and blue to magenta
above 40,000 ft; grey on the ground). Labels show callsign (or registration/hex),
altitude or flight level, and a climb/descent arrow.

- **Trails** keep about 20 minutes of history, in the aircraft's altitude
  colour, fading with age. The selected aircraft's trail is bright and thick.
- **Military** aircraft (by ICAO address block) get a green ring.
- **Emergencies** get a red ring and a red banner across the top of the radar
  (tap it to select the aircraft). An emergency also wakes the screen saver.

## Aircraft tab

A table of up to 150 aircraft: flight, registration (or ICAO hex), type,
altitude, ground speed, vertical speed, distance and squawk. Sort by distance,
altitude, callsign, speed or most recent message. Tap a row to open its details.

## Details tab

The header shows callsign, registration, type, operator, route and a photo (when
online lookup finds one). Below it, everything known about the aircraft,
grouped into sections:

| Section | Contents |
|---|---|
| Identity | ICAO address, callsign, registration, type, manufacturer, owner/operator, country, emitter category, military flag, data source, ADS-B version, squawk and its meaning |
| Route | Airline, origin and destination airports, distance flown / remaining, time to go at current speed |
| Position | Latitude/longitude, age, distance and bearing from the receiver, barometric and GNSS altitude, on ground |
| Motion | Ground speed, track, heading (true/magnetic), TAS, IAS, Mach, vertical rates, roll, track rate |
| Autopilot / intent | Selected altitude (MCP/FCU or FMS), altimeter setting, selected heading, engaged modes (AP, VNAV, ALT HOLD, APPR, LNAV, TCAS) |
| Status | Emergency state, alert/SPI, active TCAS resolution advisory |
| Derived | Outside air temperature (from TAS and Mach), wind (when heading is true) |
| Signal / quality | RSSI, message count and rate, first/last seen, NACp, NACv, SIL, SDA, NIC baro, GVA, Comm-B registers seen |

Rows only appear when the aircraft actually transmits that data — what you see
depends on the aircraft's equipment and which interrogations your receiver
overhears. See [What is decoded](decoding.md).

## Stats tab

Message-rate chart, totals, per-downlink-format counts, maximum range seen, feed
and Wi-Fi status, online lookup counters, uptime, free memory and firmware
version.

## Settings tab

See the [Settings reference](settings.md).
