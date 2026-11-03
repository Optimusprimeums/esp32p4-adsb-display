# Credits and licences

This project is licensed under the **GNU General Public License v3.0 or later**
(GPL-3.0-or-later). Third-party parts keep their own licences, listed below.

## Third-party code and data in this repository

| Item | Source | Licence |
|---|---|---|
| GSL3680 touch driver (`components/esp_lcd_touch_gsl3680`) | Guition demo package for the JC8012P4A1; buffer-overflow fixes added here | Vendor code; `gsl_point_id.c` (Silead touch algorithm) is **GPL-2.0-or-later** |
| JD9365 V2 panel init sequence (`main/panel_jc8012_v2.h`) | [ESPHome](https://github.com/esphome/esphome) `mipi_dsi/models/guition.py`, model `JC8012P4A1-V2` | MIT |
| ICAO address allocations and military ranges (`main/icao_db.c`) | Generated from the allocation table in [dump1090-fa](https://github.com/flightaware/dump1090) (ICAO Annex 10 Vol III) and [tar1090-db](https://github.com/wiedehopf/tar1090-db) military ranges | factual data; credited |
| ESP-Hosted co-processor project (`c6_coprocessor/`) | Based on the ESP-Hosted `coprocessor_ota` example by Espressif | Apache-2.0 |
| Decoder reference vectors (`main/selftest.c`) | *The 1090 MHz Riddle* (Junzi Sun) and the [pyModeS](https://github.com/junzis/pyModeS) test suite | test data |

`gsl_point_id.c` is GPL-2.0-or-later, which is compatible with this project's
GPL-3.0-or-later licence.

## Libraries fetched at build time

ESP-IDF, LVGL, ESP-Hosted, esp_wifi_remote, esp_lcd_jd9365, esp_lcd_touch and
cJSON are downloaded by the ESP-IDF component manager under their own licences
(Apache-2.0 / MIT).

## Map and lookup data

- Coastlines: [Natural Earth](https://www.naturalearthdata.com/) (public domain)
  or [OpenStreetMap](https://www.openstreetmap.org/copyright) © OpenStreetMap
  contributors (ODbL), when generated with `--osm-coast`.
- Airports and runways: [OurAirports](https://ourairports.com/data/) (public domain).
- Aircraft and route lookups at run time: [adsbdb.com](https://www.adsbdb.com/);
  photos from airport-data.com.

## Thanks

- [jtenniswood/espcontrol](https://github.com/jtenniswood/espcontrol) for
  documenting the JC8012P4A1 panel revisions and the P4 v3 board variant.
- [profi-max/JC8012P4A1_BSP_ESP32P4](https://github.com/profi-max/JC8012P4A1_BSP_ESP32P4)
  for collecting Guition's board files.
- The dump1090/readsb, tar1090 and pyModeS projects for the Mode S knowledge
  this decoder builds on.
