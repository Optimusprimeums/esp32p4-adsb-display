# What is decoded

With a Beast or raw AVR feed the display receives every Mode S frame your
receiver hears and decodes them itself (`main/modes.c`). CRC is checked for all
frames; replies whose address is hidden in the parity are only accepted for
aircraft already seen with a verified address.

## Downlink formats

| DF | Content | Used for |
|---|---|---|
| 0 | Short air-air surveillance (ACAS) | Altitude, on-ground |
| 4 / 20 | Surveillance / Comm-B altitude reply | Altitude, alert, SPI, ground state |
| 5 / 21 | Surveillance / Comm-B identity reply | Squawk |
| 11 | All-call reply | Discovers aircraft, capability |
| 16 | Long air-air surveillance (ACAS) | Altitude, resolution advisory |
| 17 | ADS-B extended squitter | Everything below |
| 18 | TIS-B / ADS-R / non-transponder ADS-B | Same message types, marked by source |

## ADS-B message types (DF17/18)

| Type code | Content |
|---|---|
| 1–4 | Callsign, emitter category (light, heavy, rotorcraft, UAV, …) |
| 5–8 | Surface position, ground speed, track |
| 9–18 | Airborne position (CPR), barometric altitude, surveillance status |
| 19 | Velocity: ground speed + track, or heading + IAS/TAS; vertical rate (baro or GNSS); GNSS–baro difference; NACv |
| 20–22 | Airborne position with GNSS height |
| 28 | Emergency/priority status with squawk; TCAS resolution advisory broadcast |
| 29 | Target state: selected altitude (MCP/FCU or FMS), altimeter setting, selected heading, autopilot modes, NACp, SIL, NIC baro |
| 31 | Operational status: ADS-B version, NACp, SIL, SDA, GVA, NIC baro, TCAS capability, 1090ES/UAT receive, heading reference |

Positions use CPR decoding: global decoding from an even/odd pair, then local
decoding against the last known position. Surface positions are resolved
against the receiver location. Implausible jumps are rejected.

## Comm-B (DF20/21)

Secondary radar interrogates aircraft for "BDS registers". Their replies don't
say which register they contain, so the decoder infers it from the bit patterns
and checks the result against the aircraft's own ADS-B state:

| Register | Content |
|---|---|
| BDS 1,0 | Data link capability (recognised only) |
| BDS 2,0 | Aircraft identification (callsign) |
| BDS 3,0 | ACAS active resolution advisory |
| BDS 4,0 | Selected vertical intention: MCP/FCU altitude, FMS altitude, altimeter setting, VNAV/ALT HOLD/APPR modes |
| BDS 5,0 | Track and turn: roll angle, true track, ground speed, track rate, TAS |
| BDS 6,0 | Heading and speed: magnetic heading, IAS, Mach, barometric and inertial vertical rate |

How many Comm-B replies you receive depends on secondary radar coverage at your
location.

## Derived values

| Value | From |
|---|---|
| Distance and bearing | Aircraft position and receiver location |
| Outside air temperature | TAS ÷ Mach gives the speed of sound, which depends only on temperature |
| Wind | Ground vector minus air vector — only when the heading is known to be true (magnetic headings would need a declination model) |
| Country and military flag | ICAO 24-bit address block allocations |
| Distance flown / remaining, time to go | Position and route airports from the online lookup |

## Online enrichment

With **Online lookup** on, aircraft are looked up (selected aircraft first,
then nearest) at [adsbdb.com](https://www.adsbdb.com/): registration, type,
manufacturer, owner/operator, airline, origin and destination. The selected
aircraft's photo thumbnail comes from airport-data.com. Results are cached and
requests are rate-limited to about one per second.

## Self-test

At boot the decoder is checked against published reference frames ("The
1090 MHz Riddle" and the pyModeS test suite) — identification, CPR, both
velocity subtypes, BDS 2,0/4,0/5,0/6,0 and CRC rejection. The result is
logged (`decoder self-test: all passed`) and is part of the OTA verification.
