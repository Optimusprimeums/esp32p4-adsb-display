# Firmware updates (OTA)

After the first USB flash, new firmware can be installed over Wi-Fi.

## Installing an update

1. Build: `idf.py build` → `build/adsb_display.bin`.
2. On the display: **Settings → Allow OTA update**. Uploads are accepted for
   **10 minutes**; the screen shows the address to use.
3. Upload, either from a browser at `http://<display-ip>/` (choose the `.bin`,
   click **Upload and install**), or from a terminal:

    ```bash
    curl --data-binary @build/adsb_display.bin http://<display-ip>/update
    ```

    On Windows PowerShell use `curl.exe` (plain `curl` is an alias for something
    else).

4. The display reboots into the new firmware and confirms it (see below).
   `http://<display-ip>/status` shows the running version and slot.

## How it stays safe

The flash holds **two application slots** (`ota_0`, `ota_1`). The running
firmware is never touched by an update.

**While uploading**

- Uploads are refused unless the window was opened on the device itself, so
  nobody else on the network can flash it.
- The image is checked before anything is written (ESP image header, same
  project name, right chip) and written to the *inactive* slot.
- At the end the whole image is verified (including its SHA-256) before it is
  marked bootable. An interrupted, truncated or corrupt upload is discarded and
  the current firmware keeps running.

**After rebooting into the new firmware**

The new image starts *on probation* ("pending verify"). It confirms itself only
when, within **5 minutes**:

- the decoder self-test passes,
- the user interface has started, and
- Wi-Fi has connected.

If it crashes, hangs (watchdog reset), fails the self-test or never gets on
Wi-Fi, the bootloader boots the **previous firmware** again on the next reset.

**Manual revert**: **Settings → Revert to previous** switches to the other slot
at any time (only offered when that slot holds a complete, valid image).

## What OTA cannot change

The bootloader and partition table are only written over USB. If a future
release changes them, the release notes will say a USB flash is needed.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `403 … upload window closed` | Tap **Allow OTA update** on the display first (it closes after 10 min or a reboot). |
| `409 … not verified yet` | The running firmware is still on probation; wait until Wi-Fi is up. |
| `image is a different project` | You uploaded the wrong `.bin` (e.g. the bootloader or the C6 firmware). Use `build/adsb_display.bin`. |
| Upload stops part-way and the display restarts (firmware 1.1.3 or older) | Old firmware could trip its task watchdog at full LAN speed. Upload once with `curl --limit-rate 64K …`; 1.1.4 and newer handle full speed. |
| Display is back on the old version | The new one failed its checks and was rolled back. Connect USB and run `idf.py monitor` to see why. |
