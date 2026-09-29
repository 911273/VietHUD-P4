"""Cut a regional, ALL-ROADS speedmap out of the full nationwide SD dataset so
it fits the ESP32-P4 board's 13.5 MB flash FAT partition (no microSD card).

Tiles are taken whole, nearest-first around a centre point, until the byte
budget is used. Segment ids are renumbered 1..N (seg_names.bin is a headerless
array indexed by segment id, 106 MB nationwide), seg_names.bin is rebuilt for
the new ids, index.bin/metadata.bin (tile/segment counts, CRC32 of index.bin)
are regenerated, the metadata self-test point is disabled. names.bin (global
name ids), cameras.bin, signs.bin and sounds/ are copied unchanged.

usage: python make_region_subset.py SRC_SPEEDMAP_DIR OUT_SPEEDMAP_DIR LAT LON [BUDGET_MB]
"""
import math
import os
import shutil
import struct
import sys
import zlib

SRC, OUT = sys.argv[1], sys.argv[2]
LAT, LON = float(sys.argv[3]), float(sys.argv[4])
BUDGET = float(sys.argv[5]) if len(sys.argv) > 5 else 12.6  # MB of the 13.1 MB usable FAT
SEG = 28
META_FMT = '<4sH16s16sQIIfiihI64sI'

meta_raw = open(os.path.join(SRC, 'metadata.bin'), 'rb').read()
meta = list(struct.unpack_from(META_FMT, meta_raw))
idx = open(os.path.join(SRC, 'index.bin'), 'rb').read()
ents = [struct.unpack_from('<IiiiiII', idx, i) for i in range(0, len(idx) - len(idx) % 28, 28)]

# fixed-size companions copied as-is
copies = ['names.bin', 'cameras.bin', 'signs.bin', 'manifest.txt']
fixed = sum(os.path.getsize(os.path.join(SRC, f)) for f in copies if os.path.exists(os.path.join(SRC, f)))
snd = os.path.join(SRC, 'sounds')
fixed += sum(os.path.getsize(os.path.join(r, x)) for r, _, fs in os.walk(snd) for x in fs)
room = BUDGET * 1e6 - fixed - 64 * 1024  # metadata/index/FAT slack
print(f'fixed files {fixed / 1e6:.2f} MB -> {room / 1e6:.2f} MB for tiles + seg_names')


def dist_km(e):
    clat = (e[1] + e[2]) / 2e7
    clon = (e[3] + e[4]) / 2e7
    return math.hypot((clat - LAT) * 110.54, (clon - LON) * 111.32 * math.cos(math.radians(LAT)))


chosen, used, radius = [], 0, 0
for e in sorted(ents, key=dist_km):
    cost = e[6] // SEG * (SEG + 4)  # segment record + its seg_names entry
    if used + cost > room:
        break
    chosen.append(e)
    used += cost
    radius = dist_km(e)
chosen.sort(key=lambda e: e[0])  # index.bin must be sorted by tileId
print(f'{len(chosen)} tiles within ~{radius:.1f} km of {LAT},{LON}')

os.makedirs(OUT, exist_ok=True)
tiles_src = open(os.path.join(SRC, 'tiles.bin'), 'rb')
segnames_src = open(os.path.join(SRC, 'seg_names.bin'), 'rb')
new_id = {}
out_tiles = bytearray()
out_idx = bytearray()
names_out = []
for (tid, a, b, c, d, off, size) in chosen:
    tiles_src.seek(off)
    blob = bytearray(tiles_src.read(size))
    for k in range(0, len(blob) - len(blob) % SEG, SEG):
        old = struct.unpack_from('<I', blob, k)[0]
        nid = new_id.get(old)
        if nid is None:
            nid = len(new_id) + 1
            new_id[old] = nid
            segnames_src.seek(old * 4)
            names_out.append(segnames_src.read(4) or b'\0\0\0\0')
        struct.pack_into('<I', blob, k, nid)
    out_idx += struct.pack('<IiiiiII', tid, a, b, c, d, len(out_tiles), len(blob))
    out_tiles += blob

seg_names = bytearray(4) + b''.join(names_out)  # id 0 unused
open(os.path.join(OUT, 'tiles.bin'), 'wb').write(out_tiles)
open(os.path.join(OUT, 'index.bin'), 'wb').write(out_idx)
open(os.path.join(OUT, 'seg_names.bin'), 'wb').write(seg_names)

meta[2] = b'VN-LOCAL'.ljust(16, b'\0')
meta[5] = len(chosen)            # tileCount
meta[6] = len(new_id)            # segmentCount
meta[8], meta[9], meta[10], meta[11] = 0, 0, 0, 0  # no self-test point in a regional cut
meta[13] = zlib.crc32(bytes(out_idx)) & 0xFFFFFFFF
open(os.path.join(OUT, 'metadata.bin'), 'wb').write(struct.pack(META_FMT, *meta))

for f in copies:
    if os.path.exists(os.path.join(SRC, f)):
        shutil.copy2(os.path.join(SRC, f), os.path.join(OUT, f))
if os.path.isdir(os.path.join(OUT, 'sounds')):
    shutil.rmtree(os.path.join(OUT, 'sounds'))
shutil.copytree(snd, os.path.join(OUT, 'sounds'))

total = sum(os.path.getsize(os.path.join(r, x)) for r, _, fs in os.walk(OUT) for x in fs)
print(f'segments {len(new_id)}, tiles.bin {len(out_tiles) / 1e6:.2f} MB, seg_names {len(seg_names) / 1e6:.2f} MB, '
      f'total {total / 1e6:.2f} MB')
