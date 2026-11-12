# Airband audio

The **Airband** tab plays live voice from a
[GopherTrunk](https://github.com/MattCheramie/GopherTrunk) scanner through the
board's speaker. It also shows which channel is transmitting and keeps a list
of recent transmissions.

GopherTrunk runs on a separate machine with the SDRs, demodulates the AM air
band and publishes the audio on its HTTP API. The display plays the audio and
can tune the scanner (see [Tuning](#tuning)).

```
SDR(s) -> GopherTrunk (AM channels) --HTTP--> display: ES8311 codec -> speaker
```

## What the display uses

| Endpoint | Used for |
|---|---|
| `GET /api/v1/audio/stream` | Live audio: an open-ended WAV stream (16-bit mono PCM). It is only opened while **Listen** is on. `?device=<serial>` narrows it to one channel. |
| `GET /api/v1/events` | Server-Sent Events. `call.start` and `call.end` give the channel label, frequency and duration. |
| `GET /api/v1/scanner`, `POST /api/v1/scanner/manual_tune`, `DELETE /api/v1/scanner/manual_tune/{index}`, `POST /api/v1/scanner/conventional/{index}/lockout`, `.../unlockout`, `.../dwell`, `POST /api/v1/scanner/conventional/resume` | [Tuning and guard](#tuning) only |

Listening uses only read-only `GET`s, which need no credentials. Tuning
changes the scanner's state, so it needs a token or a trusted network (see
below).

## GopherTrunk configuration

1. **Expose the API on the LAN.** It binds to loopback by default:

    ```yaml
    api:
      http_addr: "0.0.0.0:8080"
    ```

2. **Add the air-band channels** as AM conventional channels. Each one needs
   a `label`, which is the name the display shows:

    ```yaml
    scanner:
      conventional:
        - label: "Tower"
          frequency_hz: 118700000    # the actual carrier (118.700 MHz)
          mode: am
          squelch_cn_db: 12
        - label: "Approach"
          frequency_hz: 119100000
          mode: am
    ```

    A wideband SDR can monitor several channels at once through
    `sdr.devices[].conventional`. See GopherTrunk's `config.example.yaml`.

3. **Pick the live sample rate.** 8 kHz is plenty for AM voice and keeps the
   stream at 16 KB/s. The display accepts anything from 4 to 48 kHz.

    ```yaml
    audio:
      sample_rate: 8000
    ```

    If the tab reports **"Live audio off in GopherTrunk (503)"**, the daemon
    has no live-audio publisher running. Check that `audio:` is set up as in
    GopherTrunk's documentation.

Check from any PC on the LAN that it plays:

```bash
curl -N http://<gophertrunk-host>:8080/api/v1/audio/stream --output - | ffplay -
```

## On the display

1. **Settings → GopherTrunk server:** enter the host and port (default 8080),
   then tap **Save**.
2. Open the **Airband** tab and tap **Listen**.

| Control | What it does |
|---|---|
| **Listen / Stop** | Opens or closes the audio stream. When stopped, the speaker amplifier is off. The setting is remembered across reboots. |
| Level bar | Peak level of the audio being played. |
| **Volume**, mute | Codec output volume. Remembered. |
| **Channels** | Every channel heard since boot. Tap one to hear only that channel (GopherTrunk's `?device=` filter). **All channels** removes the filter. A green border means the channel is on air now. |
| Recent transmissions | The last 30 transmissions: time, channel, frequency and length. |

While Listen is on, the radar also shows the channel currently on air, on
every tab.

### Tuning

The **Tune** tile lets the display tune GopherTrunk itself:

| Control | What it does |
|---|---|
| Preset buttons | Tune to that frequency (AM). The tuned preset is highlighted. |
| Frequency box and **Tune** | Type any frequency in MHz (e.g. `118.700`, or `118.005`, which is the 118.000 MHz carrier on 8.33 kHz spacing) and tap Tune. |
| **Guard** | Choose presets to keep watching. Tap Guard, tap the presets (an eye and an amber border mark them), then tap **Done**. A long press on a preset also toggles it. |
| **Scan** | Stop holding and go back to scanning. |

Tuning holds one frequency. Guarded presets stay on GopherTrunk's scan list all the
time, so:

- **Holding:** the scanner visits the tuned frequency and the guarded
  presets, and nothing else. A call on a guard channel still gets through,
  like dual watch on an aircraft radio. Every other channel is locked out
  while you hold.
- **Scanning:** the scanner covers its own scan list plus the guarded
  presets. Channels the display locked out are unlocked again.

With one voice SDR, GopherTrunk visits these channels in turn, so the more you
guard, the longer a call on the held frequency can go unheard at its start.

The display adds the guarded presets and the tuned frequency to the scan list
as temporary AM channels (`POST /api/v1/scanner/manual_tune`). It removes a
channel when you unguard it or tune elsewhere. Tuning also clears any channel
filter and turns **Listen** on, so you hear the result straight away.

The display keeps track of what it added and locked out, and of the tuned
frequency, across restarts. After a reboot it compares that with
`GET /api/v1/scanner`: it removes channels it no longer wants and puts back any
that went missing because GopherTrunk restarted. It only touches channels it
added or locked itself. GopherTrunk renumbers temporary channels when one is
removed, so the display always finds a channel by frequency and label, never
by a saved index. GopherTrunk's manual-tune endpoint refuses to delete
channels from its config file.

What tuning needs on the GopherTrunk side:

- **A conventional scan-list scanner** (`scanner.conventional`) with an SDR
  for it. Without one, GopherTrunk answers 503 and the tile says
  *"GopherTrunk has no scan-list scanner to tune"*.
- **Permission to change things.** On a LAN listener, GopherTrunk requires a
  bearer token for any endpoint that changes state. Either:
    - set `api.auth.token` in GopherTrunk and enter the same token under
      **Settings → GopherTrunk token** (stored on the display, never shown), or
    - add your LAN to `api.auth.trusted_networks` (e.g. `192.168.1.0/24`) and
      leave the token empty.

**Presets** are set in **Settings → Tune presets** as `Label MHz` pairs
separated by `;`, for example:

```
Tower 118.700; Ground 121.900; ATIS 127.250; Guard 121.500
```

Up to 12 are shown. Guards are remembered by frequency, so renaming a preset
keeps its guard.

The build default can be set privately with `CONFIG_ADSB_AB_PRESETS` in
`sdkconfig.defaults.local`. The display follows a changed default after a
firmware update until the presets are edited on the device. From then on it
keeps your list.

### Buffering

Audio starts after about 250 ms of buffering, to absorb Wi-Fi jitter. Between
transmissions the stream is silent, and the buffer refills when the next one
starts. If playback falls more than 1.5 s behind, the display skips ahead.
The Stats tab shows the link states and how much audio has been received.

## Hardware

| Signal | GPIO |
|---|---|
| I2S MCLK | 13 |
| I2S BCLK | 12 |
| I2S LRCK | 10 |
| I2S DOUT (to codec) | 9 |
| Speaker amplifier enable | 20 |
| ES8311 control | I2C on the touch bus (SDA 7 / SCL 8), address 0x18 |

The codec is driven by Espressif's
[esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)
component (Apache-2.0).
