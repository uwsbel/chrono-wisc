#!/usr/bin/env python3
# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
#
# Cut a window out of an SLDEM2015 512 px/deg tile (LOLA and SELENE Terrain
# Camera, ~59 m) into a GeoTIFF the Chrono::Planet DEM stack reads. Only the
# rows of the window are downloaded, streamed from the PDS Geosciences Node.
# Heights are written in meters above the 1737.4 km reference sphere, as the
# LOLA products shipped with Chrono are.
#
# Usage: make_sldem2015_crop.py OUT.tif [--site LON LAT] [--half-width DEG]
# The default is a 10 x 10 degree window around the Apollo 17 landing site,
# written as sldem2015_apollo17.tif (see README.md).
#
# Requires numpy and tifffile.
#
# =============================================================================

import argparse
import math
import sys
import urllib.request

import numpy as np
import tifffile

BASE = "https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/data/sldem2015/tiles/float_img/"
PPD = 512            # pixels per degree
TILE_LAT = 30        # tile height (deg)
TILE_LON = 45        # tile width (deg)
NODATA = -32768.0


def tile_name(lat, lon):
    """File name of the tile holding (lat, lon) in [-60, 60) x [0, 360)."""
    lat0 = math.floor(lat / TILE_LAT) * TILE_LAT
    lon0 = math.floor(lon / TILE_LON) * TILE_LON
    def ns(v):
        return f"{abs(v):02d}{'s' if v < 0 else 'n'}"
    return lat0, lon0, f"sldem2015_512_{ns(lat0)}_{ns(lat0 + TILE_LAT)}_{lon0:03d}_{lon0 + TILE_LON:03d}_float.img"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out")
    ap.add_argument("--site", nargs=2, type=float, default=(30.75, 20.19), metavar=("LON", "LAT"))
    ap.add_argument("--half-width", type=float, default=5.0, help="half the window size (deg)")
    args = ap.parse_args()

    lon, lat = args.site
    lat0, lon0, name = tile_name(lat, lon)
    # Window in whole pixels, clipped to the tile. Rows run south from the tile's northern edge.
    top = lat0 + TILE_LAT
    r0 = max(0, int(math.floor((top - (lat + args.half_width)) * PPD)))
    r1 = min(TILE_LAT * PPD, int(math.ceil((top - (lat - args.half_width)) * PPD)))
    c0 = max(0, int(math.floor((lon - args.half_width - lon0) * PPD)))
    c1 = min(TILE_LON * PPD, int(math.ceil((lon + args.half_width - lon0) * PPD)))
    row_bytes = TILE_LON * PPD * 4
    print(f"{name}: rows {r0}-{r1}, columns {c0}-{c1}, {(r1 - r0) * row_bytes / 1e6:.0f} MB to download")

    req = urllib.request.Request(BASE + name, headers={"Range": f"bytes={r0 * row_bytes}-{r1 * row_bytes - 1}"})
    out = np.empty((r1 - r0, c1 - c0), dtype=np.float32)
    with urllib.request.urlopen(req) as resp:
        if resp.status != 206:
            sys.exit(f"server ignored the byte range (HTTP {resp.status})")
        for r in range(r1 - r0):
            buf = resp.read(row_bytes)
            while len(buf) < row_bytes:
                more = resp.read(row_bytes - len(buf))
                if not more:
                    sys.exit(f"download ended at row {r0 + r}")
                buf += more
            row = np.frombuffer(buf, dtype="<f4")[c0:c1]
            out[r] = row * 1000.0  # km to m
            if r % 512 == 0:
                print(f"  row {r}/{r1 - r0}", flush=True)
    out[~np.isfinite(out)] = NODATA

    # Geographic coordinates on the Moon's 1737.4 km sphere, pixel corners, as in ldem_64_fixed.tif.
    west = lon0 + c0 / PPD
    north = top - r0 / PPD
    geokeys = (1, 1, 0, 10,
               1024, 0, 1, 2,          # GTModelType: geographic
               1025, 0, 1, 1,          # GTRasterType: pixel is area
               2048, 0, 1, 32767,      # GeographicType: user defined
               2049, 34737, 7, 0,      # GeogCitation
               2050, 0, 1, 32767,      # GeogGeodeticDatum: user defined
               2054, 0, 1, 9102,       # GeogAngularUnits: degree
               2056, 0, 1, 32767,      # GeogEllipsoid: user defined
               2057, 34736, 1, 0,      # GeogSemiMajorAxis
               2058, 34736, 1, 1,      # GeogSemiMinorAxis
               2061, 34736, 1, 2)      # GeogPrimeMeridianLong
    tags = [
        (33550, "d", 3, (1.0 / PPD, 1.0 / PPD, 0.0), True),
        (33922, "d", 6, (0.0, 0.0, 0.0, west, north, 0.0), True),
        (34735, "H", len(geokeys), geokeys, True),
        (34736, "d", 3, (1737400.0, 1737400.0, 0.0), True),
        (34737, "s", 0, "Moon|", True),
        (42113, "s", 0, str(int(NODATA)), True),
    ]
    tifffile.imwrite(args.out, out, tile=(256, 256), extratags=tags)
    valid = out[out != NODATA]
    print(f"Wrote {args.out}: {out.shape[1]} x {out.shape[0]}, lon {west:.4f} to {west + out.shape[1] / PPD:.4f}, "
          f"lat {north - out.shape[0] / PPD:.4f} to {north:.4f}, heights {valid.min():.0f} to {valid.max():.0f} m")


if __name__ == "__main__":
    main()
