# ESP32-P4 ADS-B Display

Standalone ADS-B radar firmware for the **Guition JC8012P4A1** — a 10.1"
800×1280 touch display with an ESP32-P4 and an ESP32-C6 Wi-Fi co-processor.

It connects over Wi-Fi to your existing ADS-B receiver (dump1090, readsb,
PiAware, feeder containers…), decodes Mode S / ADS-B / Comm-B itself, and shows
a radar view with an offline map and trails, a sortable aircraft list, and a
details page with everything known about each aircraft — down to autopilot
targets, roll angle and outside air temperature.

**📖 Documentation: <https://optimusprimeums.github.io/esp32p4-adsb-display/>**

## Highlights

- Own Mode S decoder: DF0/4/5/11/16/17/18/20/21, all ADS-B message types, CPR,
  Comm-B BDS 2,0 / 4,0 / 5,0 / 6,0 inference, verified by a boot-time self-test
- Radar with offline coastline/runway/airport map, altitude-coloured trails,
  emergency and military highlighting
- Registration, type, operator, route and photo via adsbdb.com (optional)
- Touchscreen settings: Wi-Fi scan, server, location, units, timezone,
  orientation, screen saver
- Over-the-air updates with A/B slots and automatic rollback
- ESP32-C6 co-processor firmware update pushed from the P4

## Quick start

```bash
git clone https://github.com/Optimusprimeums/esp32p4-adsb-display.git
cd esp32p4-adsb-display
idf.py build
idf.py -p <port> flash monitor
```

Requires ESP-IDF 6.1. Then set Wi-Fi, server and location in the **Settings**
tab. See [Build and flash](https://optimusprimeums.github.io/esp32p4-adsb-display/getting-started/)
for details, older board revisions and private build defaults.

## Documentation

| | |
|---|---|
| [Hardware](docs/hardware.md) | Board, pins, chip and panel revisions |
| [Build and flash](docs/getting-started.md) | Toolchain, build, first setup |
| [Connecting a feed](docs/feeds.md) | Beast/AVR/SBS, dump1090, Docker |
| [Screens](docs/using.md) · [Settings](docs/settings.md) | Using the display |
| [Offline map](docs/offline-map.md) | Generating the map for your area |
| [Firmware updates](docs/ota.md) | OTA, rollback, recovery |
| [Wi-Fi co-processor](docs/coprocessor.md) | ESP32-C6 / ESP-Hosted |
| [Troubleshooting](docs/troubleshooting.md) | Stripes, touch, ghosting, Wi-Fi |
| [What is decoded](docs/decoding.md) · [Architecture](docs/development.md) | Internals |
| [Credits and licences](docs/credits.md) | Third-party code and data |

## Licence

GPL-3.0-or-later — see [LICENSE](LICENSE). Third-party code and data keep
their own licences; see [Credits and licences](docs/credits.md).
