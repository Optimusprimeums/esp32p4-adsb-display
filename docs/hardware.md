# Hardware

## The board

**Guition JC8012P4A1** (sold as JC8012P4A1C with capacitive touch):

| Part | Detail |
|---|---|
| Main MCU | ESP32-P4 (dual-core RISC-V, 32 MB PSRAM, 16 MB flash) |
| Wi-Fi / BT | ESP32-C6 co-processor, connected to the P4 over SDIO |
| Display | 10.1" IPS, 800×1280 portrait, JD9365 controller, 2-lane MIPI-DSI |
| Touch | GSL3680 capacitive controller over I²C (**not** a GT911) |
| Power | 5 V over USB-C |

## Pin map

| Function | GPIO |
|---|---|
| LCD reset | 27 |
| Backlight (PWM) | 23 |
| Touch I²C SDA / SCL | 7 / 8 |
| Touch reset / INT | 22 / 21 |
| C6 SDIO CLK / CMD | 18 / 19 |
| C6 SDIO D0–D3 | 14, 15, 16, 17 |
| C6 reset | 54 |
| Console UART TX / RX | 37 / 38 |

## Revisions — check yours first

These boards have been built with two generations of ESP32-P4 silicon and (at
least) two LCD panels. The firmware defaults to the newer combination.

### ESP32-P4 chip revision

ESP-IDF 6 builds either for P4 revision **≥ 3.0** or for **< 3.0**, never both.
The boot log tells you which you have:

```text
I (30) boot: chip revision: v3.2
```

- **v3.x** — use the default build.
- **v1.x / v0.x** — build with the `sdkconfig.defaults.rev1` overlay (see
  [Build and flash](getting-started.md#older-boards-esp32-p4-rev-1x)).

Flashing the wrong one fails with a "chip revision" error at boot; nothing is
damaged.

### LCD panel

Boards with a v3.x chip carry the newer **"V2" panel**, which needs its own
JD9365 initialisation sequence and timing (70 MHz pixel clock, 840 Mbps DSI
lanes). That's what this firmware uses (`main/panel_jc8012_v2.h`).

If you see full-screen vertical colour stripes instead of the UI, the init
sequence doesn't match your panel — see
[Troubleshooting](troubleshooting.md#full-screen-stripes).
