# Troubleshooting

Most problems are easiest to diagnose from the serial log: connect USB and run
`idf.py -p <port> monitor`.

## Boot fails with a chip revision error

The build targets a different ESP32-P4 silicon revision. Check `boot: chip
revision:` in the log and see [Hardware → chip revision](hardware.md#esp32-p4-chip-revision).

## Full-screen stripes

Vertical (in portrait) bands of pastel colour over the whole screen, while the
log says the LCD came up (`jd9365: LCD ID: 93 65 04`), mean the panel is
**not accepting the video stream** — what you see is leftover panel memory.

The JC8012P4A1 has shipped with different LCD panels that need different JD9365
init sequences and timing. This firmware uses the newer **V2** panel settings
(`main/panel_jc8012_v2.h`, 70 MHz pixel clock, 840 Mbps lanes, vertical porches
10/30). Older units need the original Guition sequence from Guition's demo
package (also collected in
[profi-max/JC8012P4A1_BSP_ESP32P4](https://github.com/profi-max/JC8012P4A1_BSP_ESP32P4))
with 60 MHz pixel clock, 1500 Mbps lanes and vertical porches 8/20. Swap the
table and timing in `lcd_init()` in `main/board.c` to try it.

## Touch is offset or mirrored

The panel and touch orientation are configured under **ADS-B Display → Board
orientation** in `idf.py menuconfig` (LCD mirror X/Y, touch mirror X/Y). For a
180° rotation use **Settings → Screen orientation** instead — it flips both.

## Ghosting

Faint copies of earlier screens that fade slowly are LCD image retention. It
often clears once the panel has warmed up. If it persists, adjust
**Settings → Panel VCOM** a few steps either way (it applies when you lift
your finger) and keep the cleanest value; **Save & apply** keeps it. The screen
saver also helps.

## No Wi-Fi / "C6 not responding"

- Check the log for `esp-hosted fw versions: host=… coprocessor=…`. If the C6
  never answers, its firmware may be missing or incompatible — re-flash it
  through the C6 UART pads with the ESP-Hosted co-processor firmware
  (`c6_coprocessor/`).
- Only 2.4 GHz networks are supported.

## Feed doesn't connect

- The top bar shows the last error (`DNS lookup failed`, `connect: timeout`,
  `Server closed connection`…).
- Test from a computer: `nc -v <receiver-ip> 30005`.
- With Docker feeder images, publish port 30005 and use the host's LAN IP — see
  [Connecting a feed](feeds.md).

## No positions / wrong positions

- Aircraft need both an even and an odd position message (a few seconds) before
  the first position is shown.
- Aircraft on the ground need the **receiver location** set to decode their
  position.
- Positions more than 450 nm from the receiver are rejected as implausible —
  check the receiver location if everything is missing.

## Update problems

See [Firmware updates → Troubleshooting](ota.md#troubleshooting).
