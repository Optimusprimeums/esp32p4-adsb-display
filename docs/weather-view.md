# Weather view build

The weather view build is a separate firmware release. It has two full-screen
views that share the map, and you switch between them with the button in the
top bar:

| View | Map | Lower half |
|---|---|---|
| **ADS-B** | Aircraft, trails, offline map | The usual tabs (Aircraft, Details, Airband, Stats, Settings) |
| **Weather** | The same map and aircraft, with live weather radar over them | The weather station: temperature, wind, humidity, rain, UV, light and the 24 h charts |

Both views keep the same centre and zoom, so the radar lines up with the
aircraft. Tapping an aircraft in the Weather view switches to the ADS-B view
and opens its details. Everything else is the same as the standard firmware.
The Weather tab is hidden in the ADS-B view, because the station now has a
view of its own.

## Radar

The radar comes from [RainViewer](https://www.rainviewer.com/)'s free public
API, which composites NEXRAD and other national radars. The display needs
internet access for it.

- It loads the newest radar frame every 5 minutes. RainViewer publishes a new
  frame every 10 minutes, and the time of the frame on screen is shown at the
  top left, e.g. *Radar 14:20 (RainViewer)*.
- The radar tiles are reprojected onto the map, so the radar follows panning
  and zooming. While you drag the map, the last image moves with it. About
  half a second after you stop, the display redraws it for the new view.
- The **⚡ button** on the map (above the map button) turns the radar layer on
  or off in either view. Each view starts with its usual setting: radar on in
  Weather, off in ADS-B.
- RainViewer's public API stops at zoom level 7 (about 1 km per pixel) and
  serves only its "Universal Blue" colour scheme. At short ranges the radar
  looks blocky, because that is all the detail available. If the view needs
  more than 24 tiles (about 400 nm across), the label asks you to zoom in.

Possible messages:

| Message | Meaning |
|---|---|
| *Radar: no internet* | The display can't reach api.rainviewer.com. |
| *Radar: no radar data here* | No radar covers this area. RainViewer's coverage map shows where there is radar. |
| *Radar: zoom in to show radar* | The view is too wide for the tile cache. |

## Building and installing

```bash
idf.py -B build_wx -D ADSB_VARIANT=wx -D SDKCONFIG=build_wx/sdkconfig build
```

`ADSB_VARIANT=wx` adds `sdkconfig.defaults.wx` on top of `sdkconfig.defaults`
and your private `sdkconfig.defaults.local`. It also gives the firmware the
version `x.y.z-wx`, so **Settings → Firmware** shows which build is running.

Install it like any other update, over the air with
`build_wx/adsb_display.bin` (see [Firmware updates](ota.md)). Both builds use
the same partitions and settings, so you can switch between the standard
build and the weather view build in either direction with OTA. **Revert**
takes you back to whichever build ran before.

The PNG decoder for the radar tiles (LVGL's built-in lodepng) is only
compiled into this build.
