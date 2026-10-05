# Moon elevation data for Chrono::Planet

Elevation models used by the Moon preset (`chrono_planet/planets/moon/ChMoon.h`),
selected in code with the `moon::Dem` enum. All are LRO Lunar Orbiter Laser
Altimeter (LOLA) products (NASA, public domain), as float32 heights in meters
above the 1737.4 km reference sphere, with nodata -32768.

| `moon::Dem` | File | Resolution | Zooms | Shipped |
|---|---|---|---|---|
| `GLOBAL_LOW_RES` | `ldem_4_global.tif` | 4 px/deg (~7.6 km), global | 0-2 | yes |
| `APOLLO17_LANDING_SITE` | `ldem_1024_apollo_region.tif` | 1024 px/deg (~30 m), Apollo 17 region, equirectangular | 5-30 | yes |
| `GLOBAL` | `ldem_64_fixed.tif` | 64 px/deg (~474 m), global | 0-4 | no (480 MB): `make_ldem64.py` |
| `APOLLO17_SLDEM2015` | `sldem2015_apollo17.tif` | 512 px/deg (~59 m), 10 x 10 deg around the Apollo 17 site | 5-7 | no (105 MB) |
| `APOLLO17_NAC_DTM` | `NAC_DTM_APOLLO17.TIF` | 5 m/px, Taurus-Littrow (19.39-21.30 N, 29.91-31.66 E), equirectangular | 8-30 | no (463 MB) |

`ldem_4_global.tif` is `ldem_64_fixed.tif` averaged over 16 x 16 pixel blocks,
with its geotransform origin moved to the center of the first block so each
value sits where the module places it.

`GLOBAL` is too large for the repository. `make_ldem64.py` makes it from LOLA's
64 px/deg global grid (`LDEM_64.IMG`, 531 MB, checked against its SHA-256) on the
PDS Geosciences Node. It turns the grid's 16-bit half meters into float32 meters
and starts it at 180 deg W, as the module reads it:

    python3 make_ldem64.py ldem_64_fixed.tif

Place the result in this directory (or the build tree's copy of it). Any other
DEM can be used directly through `ChGeoTiffSource`.

The LOLA Apollo 17 grid has laser measurements only along LOLA's north-south
ground tracks, which near the equator lie up to a kilometer apart, and is
interpolated between them: shaded under a low Sun it is striped. For views of
the site from altitude, or anything finer than a kilometer, use
`APOLLO17_SLDEM2015` and `APOLLO17_NAC_DTM` in its place. `APOLLO17_SLDEM2015`
is a window of SLDEM2015 (LOLA and SELENE Terrain Camera, Barker et al. 2016)
that `make_sldem2015_crop.py` cuts out, downloading only that window from the
PDS Geosciences Node:

    python3 make_sldem2015_crop.py sldem2015_apollo17.tif

`APOLLO17_NAC_DTM` is the LROC NAC stereo DTM of the site, used as released:

    curl -LO https://pds.lroc.im-ldi.com/data/LRO-L-LROC-5-RDR-V1.0/LROLRC_2001/DATA/SDP/NAC_DTM/APOLLO17/NAC_DTM_APOLLO17.TIF

Both are in meters above the 1737.4 km sphere, as the LOLA products are.
