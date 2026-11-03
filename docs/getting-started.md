# Build and flash

## Requirements

- **ESP-IDF 6.1** (installed with Espressif's EIM installer or manually)
- Python, git
- A USB-C cable for the first flash

Components (LVGL 9.5, ESP-Hosted, esp_wifi_remote, the JD9365 driver, cJSON…) are
downloaded automatically by the IDF component manager on the first build.

## Get the source

```bash
git clone https://github.com/Optimusprimeums/esp32p4-adsb-display.git
cd esp32p4-adsb-display
```

## Optional: your own build-time defaults

Everything can be set on the device later, but you can bake in defaults so a
freshly flashed board works straight away. Copy the example and edit it:

```bash
cp sdkconfig.defaults.local.example sdkconfig.defaults.local
```

`sdkconfig.defaults.local` is ignored by git, so your Wi-Fi password, server
address and home location never end up in a commit. The build picks it up
automatically.

| Option | Meaning |
|---|---|
| `CONFIG_ADSB_WIFI_SSID` / `_PASSWORD` | Wi-Fi network |
| `CONFIG_ADSB_SERVER_HOST` / `_PORT` | Your ADS-B receiver |
| `CONFIG_ADSB_RX_LAT` / `_LON` | Receiver location, decimal degrees |
| `CONFIG_ADSB_DEFAULT_TZ` | Timezone index (0 = UTC; order as in Settings) |
| `CONFIG_LWIP_LOCAL_HOSTNAME` | Name the display gives your router |

Settings saved on the device always take precedence over these defaults.

## Optional: offline map

To get coastlines, runways and airport codes on the radar, generate a map for
your location before building — see [Offline map](offline-map.md). Without one,
the radar shows rings and aircraft only.

## Build

Open a terminal with the ESP-IDF 6.1 environment activated, then:

```bash
idf.py build
```

## Flash (first time, over USB)

Connect the board's USB-C port and find its serial port (e.g. `COM5` on
Windows, `/dev/ttyUSB0` / `/dev/ttyACM0` on Linux):

```bash
idf.py -p COM5 flash monitor
```

The first USB flash installs the bootloader and partition table that enable
[over-the-air updates](ota.md); after that USB is only needed for recovery.

A healthy first boot logs, among other things:

```text
I (1163) selftest: decoder self-test: all passed
I (1364) jd9365: LCD ID: 93 65 04
I (2658) ui: UI ready
I (8938) wifi: got IP 192.168.1.123
I (9006) feed: connected to 192.168.1.50:30005 (Beast)
```

## First-time setup on the device

1. Open the **Settings** tab.
2. **Scan** for Wi-Fi, pick your network, enter the password.
3. Enter your receiver's **IP address**, port **30005**, protocol **Beast**.
4. Enter your **receiver latitude / longitude** — needed for range rings,
   distances, and decoding positions of aircraft on the ground.
5. Choose units, timezone, orientation and screen saver to taste.
6. Tap **Save & apply**.

## Older boards (ESP32-P4 rev 1.x)

```bash
idf.py -B build_rev1 -D SDKCONFIG=build_rev1/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.rev1" build
idf.py -B build_rev1 -D SDKCONFIG=build_rev1/sdkconfig -p COM5 flash monitor
```

(Add `;sdkconfig.defaults.local` to the list if you use local defaults.)
Older boards may also have the original LCD panel, which needs the older
JD9365 init sequence — see [Troubleshooting](troubleshooting.md#full-screen-stripes).
