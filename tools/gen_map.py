#!/usr/bin/env python3
"""
Generate main/map_data.c (offline radar background) for a receiver location.

Data sources (download once, public domain):
  Natural Earth 10m coastline + minor islands (GeoJSON):
    https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_coastline.geojson
    https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_minor_islands_coastline.geojson
  OurAirports:
    https://davidmegginson.github.io/ourairports-data/airports.csv
    https://davidmegginson.github.io/ourairports-data/runways.csv

Optional high-resolution coastline (OpenStreetMap, ODbL - credit "(c) OpenStreetMap
contributors"), fetched once from the Overpass API for a bounding box, e.g.:
  curl -A "your-app" -o osm_coast.json --data-urlencode     'data=[out:json][timeout:180];way["natural"="coastline"](S,W,N,E);out geom;'     https://overpass-api.de/api/interpreter

Usage:
  python tools/gen_map.py --lat 51.4700 --lon -0.4543 --radius 400 --data <dir> [--osm-coast osm_coast.json]
"""
import argparse
import csv
import json
import math
import os

NM_PER_DEG = 60.0


def dist_nm(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * 3440.065 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def simplify(pts, tol):
    """Douglas-Peucker on (lon, lat) in degrees (fine for small tolerances)."""
    if len(pts) < 3:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        ax, ay = pts[a]
        bx, by = pts[b]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        best, idx = 0.0, -1
        for i in range(a + 1, b):
            px, py = pts[i]
            if norm < 1e-12:
                # closed ring (first == last point): distance from that point
                d = math.hypot(px - ax, py - ay)
            else:
                d = abs(dy * (px - ax) - dx * (py - ay)) / norm
            if d > best:
                best, idx = d, i
        if best > tol and idx > 0:
            keep[idx] = True
            stack.append((a, idx))
            stack.append((idx, b))
    return [p for p, k in zip(pts, keep) if k]


def load_lines(path):
    with open(path, encoding='utf-8') as f:
        gj = json.load(f)
    for feat in gj['features']:
        g = feat['geometry']
        if not g:
            continue
        if g['type'] == 'LineString':
            yield g['coordinates']
        elif g['type'] == 'MultiLineString':
            for c in g['coordinates']:
                yield c
        elif g['type'] == 'Polygon':
            for c in g['coordinates']:
                yield c
        elif g['type'] == 'MultiPolygon':
            for poly in g['coordinates']:
                for c in poly:
                    yield c


def load_osm_ways(path):
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    for el in d.get('elements', []):
        geom = el.get('geometry')
        if el.get('type') == 'way' and geom:
            yield [(g['lon'], g['lat']) for g in geom]


def clip_line(coords, lat0, lon0, radius):
    """Split a polyline into runs of points inside the radius."""
    run = []
    for lon, lat in coords:
        if dist_nm(lat0, lon0, lat, lon) <= radius:
            run.append((lon, lat))
        else:
            if len(run) >= 2:
                yield run
            run = []
    if len(run) >= 2:
        yield run


def c_str(s, n):
    s = (s or '').encode('ascii', 'replace').decode('ascii')
    s = s.replace('\\', '/').replace('"', "'")
    return s[:n - 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lat', type=float, required=True)
    ap.add_argument('--lon', type=float, required=True)
    ap.add_argument('--radius', type=float, default=400, help='nm')
    ap.add_argument('--tol', type=float, default=0.0003, help='simplify tolerance, degrees')
    ap.add_argument('--data', required=True)
    ap.add_argument('--osm-coast', help='Overpass JSON of natural=coastline ways (replaces Natural Earth)')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'main', 'map_data.c'))
    a = ap.parse_args()

    # --- coastlines ---
    lines = []
    if a.osm_coast:
        sources = [load_osm_ways(a.osm_coast)]
        coast_credit = 'Coastlines: (c) OpenStreetMap contributors (ODbL).'
    else:
        sources = [load_lines(os.path.join(a.data, n))
                   for n in ('ne_10m_coastline.geojson', 'ne_10m_minor_islands_coastline.geojson')
                   if os.path.exists(os.path.join(a.data, n))]
        coast_credit = 'Coastlines: Natural Earth (public domain).'
    for src in sources:
        for coords in src:
            for run in clip_line(coords, a.lat, a.lon, a.radius):
                s = simplify(run, a.tol)
                if len(s) >= 2:
                    lines.append(s)
    npts = sum(len(l) for l in lines)

    # --- airports ---
    kinds = {'large_airport': 0, 'medium_airport': 1, 'small_airport': 2}
    airports = {}
    with open(os.path.join(a.data, 'airports.csv'), encoding='utf-8') as f:
        for r in csv.DictReader(f):
            k = kinds.get(r['type'])
            if k is None:
                continue
            lat, lon = float(r['latitude_deg']), float(r['longitude_deg'])
            if dist_nm(a.lat, a.lon, lat, lon) > a.radius:
                continue
            code = r['icao_code'] or r['gps_code'] or r['ident']
            airports[r['ident']] = (k, code, r['iata_code'], r['name'], lat, lon)

    # --- runways (only for airports we kept, both ends known, not closed) ---
    runways = []
    with open(os.path.join(a.data, 'runways.csv'), encoding='utf-8') as f:
        for r in csv.DictReader(f):
            apt = airports.get(r['airport_ident'])
            if not apt or r['closed'] == '1':
                continue
            try:
                la1, lo1 = float(r['le_latitude_deg']), float(r['le_longitude_deg'])
                la2, lo2 = float(r['he_latitude_deg']), float(r['he_longitude_deg'])
            except ValueError:
                continue
            length = int(float(r['length_ft'] or 0))
            runways.append((apt[0], la1, lo1, la2, lo2, length))

    # --- emit C ---
    out = []
    out.append('/* Generated by tools/gen_map.py - do not edit.')
    out.append(' * Centre %.1f, %.1f (rounded), radius %.0f nm.' % (a.lat, a.lon, a.radius))
    out.append(' * %s Airports/runways: OurAirports (public domain). */' % coast_credit)
    out.append('#include "map_data.h"')
    out.append('')
    out.append('const float map_coast_pts[][2] = {  /* lat, lon */')
    for l in lines:
        out.append('    ' + ' '.join('{%.5ff, %.5ff},' % (lat, lon) for lon, lat in l))
    out.append('};')
    out.append('')
    out.append('const map_line_t map_coast_lines[] = {')
    start = 0
    for l in lines:
        lats = [p[1] for p in l]
        lons = [p[0] for p in l]
        out.append('    {%d, %d, %.4ff, %.4ff, %.4ff, %.4ff},' % (start, len(l), min(lats), max(lats), min(lons), max(lons)))
        start += len(l)
    out.append('};')
    out.append('const int map_coast_line_count = %d;' % len(lines))
    out.append('const char map_coast_credit[] = "%s";' % ('(c) OpenStreetMap' if a.osm_coast else ''))
    out.append('')
    out.append('const map_airport_t map_airports[] = {')
    for k, code, iata, name, lat, lon in sorted(airports.values(), key=lambda x: x[0]):
        out.append('    {%d, "%s", "%s", "%s", %.5ff, %.5ff},' % (k, c_str(code, 8), c_str(iata, 4), c_str(name, 40), lat, lon))
    out.append('};')
    out.append('const int map_airport_count = %d;' % len(airports))
    out.append('')
    out.append('const map_runway_t map_runways[] = {')
    for k, la1, lo1, la2, lo2, length in runways:
        out.append('    {%d, %d, %.5ff, %.5ff, %.5ff, %.5ff},' % (k, length, la1, lo1, la2, lo2))
    out.append('};')
    out.append('const int map_runway_count = %d;' % len(runways))
    out.append('')
    with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out))
    print('coast: %d lines / %d points, airports: %d, runways: %d -> %s'
          % (len(lines), npts, len(airports), len(runways), os.path.normpath(a.out)))


if __name__ == '__main__':
    main()
