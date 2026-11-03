# Settings reference

Settings live on the **Settings** tab and are stored in flash (NVS), so they
survive reboots and firmware updates. Most apply when you tap **Save & apply**;
the ones marked *live* take effect immediately (and are saved with the rest).

| Setting | Values | Default | Notes |
|---|---|---|---|
| Wi-Fi network | text, or pick from **Scan** | — | 2.4 GHz networks (ESP32-C6) |
| Wi-Fi password | text | — | |
| ADS-B server | host/IP and port | — / 30005 | See [Connecting a feed](feeds.md) |
| Feed protocol | Beast, Raw AVR, SBS | Beast | Changing it updates a default port automatically |
| Receiver lat / lon | decimal degrees | unset | Needed for rings, distances and surface positions |
| Units | Aviation (ft, kt, nm), Metric (m, km/h, km) | Aviation | |
| Timezone | 18 zones incl. UTC | UTC | DST switches automatically |
| Screen orientation | Normal, Flipped 180° | Normal | *live*; flips touch too |
| Online lookup | on / off | on | Registration, type, route and photo from adsbdb.com |
| Brightness | 5–100 % | 80 % | *live* |
| Screen saver | on/off + 1, 5, 10, 15, 30 min, 1 h, 2 h | on, 10 min | *live*; backlight off after no touch; first touch only wakes |
| Panel VCOM | 0x10–0x80 | 0x44 | *live*; tune only if old images "ghost" — see [Troubleshooting](troubleshooting.md#ghosting) |

The radar range you last used is remembered too.

## Firmware section

| Control | Purpose |
|---|---|
| Firmware line | Running version, slot, build date, previous version |
| **Allow OTA update** | Opens a 10-minute window for uploads — see [Firmware updates](ota.md) |
| **Revert to previous** | Boots the firmware in the other slot (asks first) |
| **Update Wi-Fi chip** | Pushes the bundled ESP-Hosted firmware to the ESP32-C6 — see [Wi-Fi co-processor](coprocessor.md) |

## Privacy

With **Online lookup** on, the display sends the ICAO addresses and callsigns of
aircraft it sees to api.adsbdb.com (and fetches photos from airport-data.com).
Nothing else leaves your network. Turn it off to keep the display fully local.
