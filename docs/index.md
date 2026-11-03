# ESP32-P4 ADS-B Display

Firmware that turns the **Guition JC8012P4A1** — a 10.1" 800×1280 touch display
built around an ESP32-P4 with an ESP32-C6 Wi-Fi co-processor — into a standalone
aircraft radar.

It connects over Wi-Fi to the ADS-B receiver you already run (dump1090,
dump1090-fa, readsb, PiAware, an FR24/ADS-B Exchange feeder container, …),
decodes the raw Mode S messages itself, and shows everything that can be learned
about each aircraft.

## Features

- **Radar view**: range rings centred on your receiver, an offline map
  (coastlines, runways, airport codes), aircraft coloured by altitude and pointing
  along their track, fading altitude-coloured trails (~20 min), labels, and
  military/emergency highlighting. Tap to select, drag to pan, zoom in steps.
- **Aircraft list**: sortable by distance, altitude, callsign, speed or recency.
- **Details page** with everything known about the selected aircraft:
  identity, registration, type, operator, route (origin → destination with
  distance flown/remaining), position, speeds (GS/IAS/TAS/Mach), heading, roll,
  vertical rates, autopilot targets and modes, emergency/TCAS status, data
  quality (NACp/NACv/SIL/…), signal strength, outside air temperature and a photo.
- **Own Mode S decoder**: DF0/4/5/11/16/17/18/20/21, all ADS-B message types,
  CPR position decoding, and Comm-B (BDS 2,0 / 4,0 / 5,0 / 6,0) inference.
- **Emergency banner** for 7500/7600/7700 and ADS-B emergency status.
- **Settings on the touchscreen**: Wi-Fi scan, server, protocol, receiver
  location, units, timezone, brightness, screen orientation, screen saver.
- **Firmware updates over Wi-Fi** with A/B slots and automatic rollback.
- **ESP32-C6 firmware update** pushed from the P4, no extra wiring.

## Quick start

1. [Check your hardware](hardware.md) (board and chip revision).
2. [Build and flash](getting-started.md) once over USB.
3. [Expose your receiver's Beast port](feeds.md) on your network.
4. On the display: **Settings** → Wi-Fi, server, location → **Save & apply**.

After that, updates go [over Wi-Fi](ota.md).

!!! note "Status"
    Developed and tested on a JC8012P4A1 with an ESP32-P4 **rev v3.2** and the
    newer ("V2") LCD panel, with ESP-IDF 6.1.
