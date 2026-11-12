#!/usr/bin/env python3
"""
Grab a screenshot from a display running the demo build and save it as PNG.

  python tools/screenshot.py <display-ip> out.png [tab=0..3] [sel=HEX|none] [range=NM] [emerg=0|1]

Tabs: 0 Aircraft, 1 Details, 2 Stats, 3 Settings. Standard library only.
"""
import struct
import sys
import urllib.request
import zlib


def bmp_to_rgb(data):
    if data[:2] != b'BM':
        raise SystemExit('not a BMP: %r' % data[:80])
    off = struct.unpack_from('<I', data, 10)[0]
    w, h = struct.unpack_from('<ii', data, 18)
    bpp = struct.unpack_from('<H', data, 28)[0]
    if bpp != 24:
        raise SystemExit('expected 24-bit BMP')
    row = (w * 3 + 3) & ~3
    rows = []
    for y in range(abs(h)):
        src = h - 1 - y if h > 0 else y            # BMP rows are bottom-up
        r = data[off + src * row: off + src * row + w * 3]
        rows.append(bytes(b for i in range(0, len(r), 3) for b in (r[i + 2], r[i + 1], r[i])))
    return w, abs(h), rows


def write_png(path, w, h, rows):
    raw = b''.join(b'\x00' + r for r in rows)
    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) \
        + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def main():
    ip, out = sys.argv[1], sys.argv[2]
    query = '&'.join(sys.argv[3:])
    url = 'http://%s/screenshot.bmp%s' % (ip, ('?' + query) if query else '')
    with urllib.request.urlopen(url, timeout=60) as r:
        data = r.read()
    w, h, rows = bmp_to_rgb(data)
    write_png(out, w, h, rows)
    print('%s: %dx%d' % (out, w, h))


if __name__ == '__main__':
    main()
