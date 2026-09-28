#!/usr/bin/env python3
"""
complete_junction_lights.py — add the missing per-approach traffic lights at
signalized junctions, using the road network to place them.

At many junctions the alert data has a light for one or two approaches only;
the firmware checks a light's direction against the car's travel direction, so
the other approaches never get a warning. For every junction that already has a
DIRECTED light (card signs.bin or the firmware's built-in OSM table):

  1. junction = lights within 40 m of each other; junction nodes = road-graph
     nodes within 30 m of the cluster centre where >= 3 segments meet
  2. approaches = segments leading INTO a junction node (one-way respected)
  3. an approach with no existing light within +-60 deg of its travel bearing
     gets a new light on that approach, 12 m before the junction node, with the
     approach bearing as its direction
  Junctions with an undirected light are skipped (it already warns for every
  approach). Minor roads joining a major-road junction are skipped.

Usage:
    python complete_junction_lights.py <speedmap_dir>            # report only
    python complete_junction_lights.py <speedmap_dir> --write     # append to signs.bin,
                                                                   # update + re-sign manifest
"""
import hashlib, math, re, shutil, struct, subprocess, sys, time, zlib
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CLUSTER_M, NODE_R_M, SETBACK_M, COVER_DEG, MERGE_DEG = 40.0, 30.0, 12.0, 60.0, 30.0
LIGHT = 6
SEGD = np.dtype([('id', '<u4'), ('slat', '<i4'), ('slon', '<i4'), ('elat', '<i4'), ('elon', '<i4'), ('hdg', '<u2'),
                 ('cls', 'u1'), ('dir', 'u1'), ('lim', '<i2'), ('src', 'u1'), ('flags', 'u1')])
IDX = np.dtype([('id', '<u4'), ('a', '<i4'), ('b', '<i4'), ('c', '<i4'), ('d', '<i4'), ('off', '<u4'), ('size', '<u4')])
HDR = struct.Struct('<4sHIHII')      # magic, version, count, reserved, crc32(payload), timestamp
REC = struct.Struct('<IiiHBBBBH')    # id, lat, lon, dir, type, speed, sub, flags, reserved


def angdiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def bearing(la1, lo1, la2, lo2):
    return (math.degrees(math.atan2((lo2 - lo1) * math.cos(math.radians(la1)), la2 - la1)) + 360.0) % 360.0


def dist_m(la1, lo1, la2, lo2):
    return math.hypot((la2 - la1) * 110540, (lo2 - lo1) * 111320 * math.cos(math.radians(la1)))


class Roads:
    def __init__(self, d):
        idx = np.frombuffer((d / 'index.bin').read_bytes(), dtype=IDX)
        self.look = {int(a): (int(b), int(c)) for a, b, c in zip(idx['id'], idx['off'], idx['size'])}
        self.f = open(d / 'tiles.bin', 'rb')
        self.cache = {}

    def tile(self, la, lo):
        k = (la << 16) | lo
        if k not in self.cache:
            e = self.look.get(k)
            if e:
                self.f.seek(e[0])
                self.cache[k] = np.frombuffer(self.f.read(e[1]), dtype=SEGD)
            else:
                self.cache[k] = None
            if len(self.cache) > 3000:
                self.cache.pop(next(iter(self.cache)))
        return self.cache[k]

    def around(self, lat, lon):
        la, lo = int((lat + 90) / 0.01), int((lon + 180) / 0.01)
        ts = [t for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (t := self.tile(la + dy, lo + dx)) is not None]
        return np.concatenate(ts) if ts else None


def load_lights(d):
    raw = (d / 'signs.bin').read_bytes()
    cnt = HDR.unpack_from(raw, 0)[2]
    card = [REC.unpack_from(raw, HDR.size + i * REC.size) for i in range(cnt)]
    lights = [(r[1] / 1e7, r[2] / 1e7, r[3]) for r in card if r[4] == LIGHT]
    src = (ROOT / 'src/map/OsmTrafficSignals.cpp').read_text(encoding='utf-8')
    osm = [(int(a) / 1e7, int(b) / 1e7, int(c)) for a, b, c in re.findall(r'\{(\d+), (\d+), (\d+)\}', src)]
    return raw, card, lights, osm


def clusters(points):
    cell = CLUSTER_M / 110540
    grid = defaultdict(list)
    for i, p in enumerate(points):
        grid[(int(p[0] / cell), int(p[1] / cell))].append(i)
    parent = list(range(len(points)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for (gx, gy), ids in grid.items():
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((gx + dx, gy + dy), ()):
                    for i in ids:
                        if i < j and dist_m(points[i][0], points[i][1], points[j][0], points[j][1]) <= CLUSTER_M:
                            parent[find(i)] = find(j)
    out = defaultdict(list)
    for i in range(len(points)):
        out[find(i)].append(points[i])
    return list(out.values())


def approaches_missing(roads, cl):
    """New (lat, lon, bearing) lights for the uncovered approaches of one junction cluster."""
    if any(p[2] == 0xFFFF for p in cl):
        return []  # an undirected light already warns for every approach
    cla = sum(p[0] for p in cl) / len(cl)
    clo = sum(p[1] for p in cl) / len(cl)
    S = roads.around(cla, clo)
    if S is None:
        return []
    ends = [(S['slat'], S['slon']), (S['elat'], S['elon'])]
    kx = 111320 * math.cos(math.radians(cla))
    # graph nodes near the centre, with degree
    deg = defaultdict(int)
    near = []
    for e, (la, lo) in enumerate(ends):
        d = np.hypot((la / 1e7 - cla) * 110540, (lo / 1e7 - clo) * kx)
        for i in np.nonzero(d <= NODE_R_M)[0]:
            key = (int(la[i]), int(lo[i]))
            deg[key] += 1
            near.append((i, e, key))
    jn = {k for k, v in deg.items() if v >= 3}
    if not jn:
        return []
    appr = []  # (bearing, lat_at_setback, lon_at_setback, class)
    for i, e, key in near:
        if key not in jn:
            continue
        other = 1 - e
        okey = (int(ends[other][0][i]), int(ends[other][1][i]))
        if okey in jn:
            continue  # internal link between junction nodes (dual carriageway)
        s = S[i]
        oneway = bool(s['flags'] & 0x02)
        # travel other -> junction: forward if the junction node is the segment END
        forward = (e == 1)
        if oneway and ((s['dir'] == 2) == forward):
            continue  # this approach is not legal towards the junction
        jla, jlo = key[0] / 1e7, key[1] / 1e7
        ola, olo = okey[0] / 1e7, okey[1] / 1e7
        L = dist_m(jla, jlo, ola, olo)
        if L < 1:
            continue
        t = min(1.0, SETBACK_M / L)
        appr.append((bearing(ola, olo, jla, jlo), jla + (ola - jla) * t, jlo + (olo - jlo) * t, int(s['cls'])))
    if len(appr) < 2:
        return []
    ranks = [c if c in (1, 2, 3) else 3 for *_, c in appr]
    best = min(ranks)
    appr = [a for a, r in zip(appr, ranks) if not (best <= 2 and r == 3)]  # alleys at a major junction
    have = [p[2] for p in cl]
    new = []
    for b, la, lo, _ in sorted(appr):
        if any(angdiff(b, h) <= COVER_DEG for h in have):
            continue
        if any(angdiff(b, n[2]) <= MERGE_DEG for n in new):
            continue
        if not on_road_along(roads, la, lo, b):
            continue  # nearest road at the placed point runs another way (curve / parallel road)
        new.append((la, lo, round(b) % 360))
    return new


def on_road_along(roads, la, lo, b, max_m=3.0, max_deg=30.0):
    """The nearest road segment at (la, lo) is within max_m and runs along bearing b (either way)."""
    S = roads.around(la, lo)
    if S is None:
        return False
    kx = 111320 * math.cos(math.radians(la))
    ax = S['slon'] / 1e7 * kx; ay = S['slat'] / 1e7 * 110540
    ex = S['elon'] / 1e7 * kx - ax; ey = S['elat'] / 1e7 * 110540 - ay
    L = np.maximum(ex * ex + ey * ey, 1e-6)
    t = np.clip(((lo * kx - ax) * ex + (la * 110540 - ay) * ey) / L, 0, 1)
    d = np.hypot(lo * kx - ax - ex * t, la * 110540 - ay - ey * t)
    k = int(np.argmin(d))
    h = (math.degrees(math.atan2(ex[k], ey[k])) + 360) % 360
    return d[k] <= max_m and min(angdiff(h, b), angdiff(h + 180, b)) <= max_deg


def main():
    d = Path(sys.argv[1])
    write = '--write' in sys.argv
    roads = Roads(d)
    raw, card, lights, osm = load_lights(d)
    cls_ = clusters(lights + osm)
    added = []
    for cl in cls_:
        added += approaches_missing(roads, cl)
    def region(pts, b):
        return sum(1 for p in pts if b[0] <= p[0] <= b[1] and b[2] <= p[1] <= b[3])
    R = {'Toan quoc': (0, 90, 0, 180), 'Ha Noi (rong)': (20.95, 21.10, 105.72, 105.92),
         'Noi thanh HN': (20.98, 21.07, 105.78, 105.88), 'TP HCM': (10.70, 10.85, 106.60, 106.75)}
    print(f'junction clusters: {len(cls_)} (from {len(lights)} card + {len(osm)} OSM lights)')
    for n, b in R.items():
        print(f'  {n:14s} new approach lights: {region(added, b)}')
    if not write:
        print('report only (use --write to add them to signs.bin)')
        return added
    stamp = time.strftime('bak_%m%d_%H%M_pre_junction')
    for n in ('signs.bin', 'manifest.txt', 'manifest.txt.sig'):
        shutil.copy2(d / n, d / f'{n}.{stamp}')
    magic, ver, cnt, res, crc, ts = HDR.unpack_from(raw, 0)
    nid = max(r[0] for r in card)
    recs = []
    for la, lo, b in added:
        nid += 1
        recs.append(REC.pack(nid, round(la * 1e7), round(lo * 1e7), b, LIGHT, 0, 0, 0, 0))
    payload = raw[HDR.size:] + b''.join(recs)
    out = HDR.pack(magic, ver, cnt + len(recs), res, zlib.crc32(payload) & 0xFFFFFFFF, int(time.time())) + payload
    (d / 'signs.bin').write_bytes(out)
    man = (d / 'manifest.txt').read_text(encoding='utf-8').splitlines()
    old = man[0].split()[1]
    new = max(time.strftime('%Y.%m.%d.%H%M'), old)
    if new == old:
        y, mo, dd, hm = old.split('.')
        new = f'{y}.{mo}.{dd}.{int(hm) + 1:04d}'
    sha = hashlib.sha256(out).hexdigest()
    man = [f'version {new}'] + [f'signs.bin {len(out)} {sha}' if l.startswith('signs.bin ') else l for l in man[1:]]
    (d / 'manifest.txt').write_text('\n'.join(man) + '\n', encoding='utf-8', newline='\n')
    r = subprocess.run([sys.executable, str(ROOT / 'tools' / 'sign_manifest.py'), str(d / 'manifest.txt')],
                       capture_output=True, text=True)
    print(f'wrote {len(recs)} lights; backup .{stamp}; manifest {old} -> {new}; {r.stdout.strip() or r.stderr.strip()}')
    return added


if __name__ == '__main__':
    main()
