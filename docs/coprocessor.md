# Wi-Fi co-processor (ESP32-C6)

The ESP32-P4 has no radio. Wi-Fi goes through the on-board ESP32-C6, which runs
Espressif's **ESP-Hosted** co-processor firmware and talks to the P4 over SDIO.
The P4 side (`esp_hosted` + `esp_wifi_remote`) makes it look like normal
ESP-IDF Wi-Fi to the application.

## Versions

Host and co-processor should run the same ESP-Hosted release. The boot log
shows both:

```text
I (3035) eh_init_evt: esp-hosted fw versions: host=3.0.9 coprocessor=3.0.9 (match)
```

Boards ship with older co-processor firmware (e.g. 2.3.x). ESP-Hosted 3.x stays
compatible with it — you'll see `major version mismatch — OTA coprocessor from
host` and Wi-Fi still works — but updating is recommended.

## Updating the C6 from the display

The matching co-processor firmware is **bundled inside the display firmware**.
**Settings → Update Wi-Fi chip** shows the running and bundled versions and is
enabled when the C6 is older. It asks for confirmation, sends the image over
SDIO (about a minute) and restarts.

- If the transfer fails or the C6 rejects the image, the C6 keeps its current
  firmware.
- If a new image were to fail *on the C6 itself*, the C6 would need re-flashing
  through its own UART pads — the P4 cannot recover it. That's why this is a
  manual, confirmed action rather than automatic.

## Rebuilding the C6 firmware

`c6_coprocessor/` is an ESP-IDF project for the C6, pinned to the same
ESP-Hosted release as the host side, SDIO transport, 4 MB flash with two OTA
slots.

```bash
cd c6_coprocessor
idf.py set-target esp32c6
idf.py build
cp build/c6_hosted_coprocessor.bin firmware/
```

Then rebuild the display firmware so it embeds the new image, and update
`C6_BUNDLED_MAJOR/MINOR/PATCH` in `main/ota.c` if the ESP-Hosted version
changed. Keep the ESP-Hosted versions in `main/idf_component.yml` and
`c6_coprocessor/main/idf_component.yml` identical.
