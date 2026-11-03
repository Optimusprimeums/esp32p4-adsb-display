# Connecting a feed

The display is a **network client**: it connects to a TCP port on the machine
that runs your ADS-B decoder. It does not need its own SDR.

## Which protocol

| Protocol | Usual port | Notes |
|---|---|---|
| **Beast binary** (recommended) | 30005 | Every raw Mode S frame plus signal level. Required for Comm-B, autopilot/intent, data-quality fields and RSSI. |
| Raw AVR | 30002 | Raw frames as hex text; no signal level unless the server adds it. |
| SBS / BaseStation | 30003 | Pre-decoded CSV. Position, altitude, speed, squawk, callsign only. |

With Beast or AVR the display decodes the frames itself — see
[What is decoded](decoding.md).

## dump1090 / readsb on a Raspberry Pi

dump1090-fa, readsb and dump1090-mutability all listen on the standard ports
when networking is enabled (`--net`). Point the display at the Pi's IP address,
port 30005, protocol Beast. Check the port is reachable from another machine:

```bash
nc -v <receiver-ip> 30005
```

You should see binary data scroll past (Ctrl+C to stop).

## Docker feeder containers

All-in-one feeder images (for example `thomx/fr24feed-piaware`) run dump1090
inside the container, but by default publish only the web map port. Publish the
Beast port as well. In `docker-compose.yml`:

```yaml
services:
  feeder:
    image: thomx/fr24feed-piaware
    ports:
      - "8080:8080"
      - "30005:30005"   # Beast output for the display
```

Then recreate the container (a restart doesn't apply new port mappings):

```bash
docker compose up -d
```

Use the **Docker host's** LAN IP on the display — not the container's internal
`172.x` address. If the host has a firewall, allow TCP 30005 from your LAN.

!!! warning
    Don't forward port 30005 to the internet. The display only needs LAN access.

## Status on the display

The top bar shows the feed state (`Connected … msg/s` or the last error). The
**Stats** tab shows message counts per downlink format, reconnects and bytes
received. The client reconnects automatically with back-off, and treats 60 s of
silence as a dead connection.
