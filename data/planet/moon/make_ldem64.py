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
# Authors: bgwitt
# =============================================================================
#
# Makes ldem_64_fixed.tif, the Moon preset's GLOBAL elevation data (moon::Dem::GLOBAL), from LOLA's 64 px/deg
# global grid on the PDS Geosciences Node. It is too large for the repository (480 MB).
#
# The PDS product (LRO-L-LOLA-4-GDR-V1.0, LDEM_64.IMG) holds 16-bit heights in half meters above the 1737.4 km
# sphere, from 0 deg east. The file made here holds them as float32 meters, from 180 deg west, in a tiled, deflated
# GeoTIFF, as the module reads it. The download is checked against its SHA-256.
#
#     python3 make_ldem64.py ldem_64_fixed.tif
#
# =============================================================================

import argparse
import hashlib
import os
import sys
import urllib.request

import numpy as np
import tifffile

URL = "https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/data/lola_gdr/cylindrical/img/ldem_64.img"
SHA256 = "1c4958699d4cffd7e777d51421b6044bba300aa0bef286a544309efb082cdbd6"
LINES, SAMPLES = 11520, 23040  # 64 px/deg, 90 deg N to 90 deg S, 0 to 360 deg E
SCALE = 0.5                     # m per DN
RADIUS = 1737400.0              # m, the reference sphere


def fetch(path):
    """Downloads the PDS grid to `path` unless it is there with the right checksum."""
    def digest(p):
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 24), b""):
                h.update(chunk)
        return h.hexdigest()

    if os.path.exists(path) and digest(path) == SHA256:
        return
    part = path + ".part"
    print(f"downloading {URL} (531 MB)")
    with urllib.request.urlopen(URL) as resp, open(part, "wb") as out:
        while True:
            chunk = resp.read(1 << 24)
            if not chunk:
                break
            out.write(chunk)
    if digest(part) != SHA256:
        os.remove(part)
        sys.exit(f"{URL} does not match its checksum")
    os.replace(part, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="the GeoTIFF to write, ldem_64_fixed.tif")
    parser.add_argument("--keep", action="store_true", help="keep the downloaded PDS grid beside the output")
    args = parser.parse_args()

    img = os.path.splitext(args.output)[0] + ".pds.img"
    fetch(img)
    dn = np.fromfile(img, "<i2").reshape(LINES, SAMPLES)
    # Half meters to meters, and 0 deg E at the left to 180 deg W at the left
    heights = np.roll(dn.astype(np.float32) * np.float32(SCALE), SAMPLES // 2, axis=1)
    del dn

    deg = 1.0 / 64
    # GeoTIFF: geographic, pixel corners, on the 1737.4 km sphere, from 180 deg W 90 deg N
    geokeys = (1, 1, 0, 10, 1024, 0, 1, 2, 1025, 0, 1, 1, 2048, 0, 1, 32767, 2049, 34737, 84, 0, 2050, 0, 1, 32767,
               2054, 0, 1, 9102, 2056, 0, 1, 32767, 2057, 34736, 1, 0, 2058, 34736, 1, 1, 2061, 34736, 1, 2)
    ascii = "GCS Name = unknown|Datum = unknown|Ellipsoid = unknown|Primem = Reference meridian||"
    extratags = [
        (33550, "d", 3, (deg, deg, 0.0), True),                    # ModelPixelScale
        (33922, "d", 6, (0.0, 0.0, 0.0, -180.0, 90.0, 0.0), True),  # ModelTiepoint
        (34735, "H", len(geokeys), geokeys, True),                 # GeoKeyDirectory
        (34736, "d", 3, (RADIUS, RADIUS, 0.0), True),              # GeoDoubleParams
        (34737, "s", 0, ascii, True),                              # GeoAsciiParams
        (42113, "s", 0, "-32768", True),                           # GDAL_NODATA
    ]
    part = args.output + ".part"
    tifffile.imwrite(part, heights, tile=(256, 256), compression="zlib", photometric="minisblack", extratags=extratags)
    os.replace(part, args.output)
    if not args.keep:
        os.remove(img)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
