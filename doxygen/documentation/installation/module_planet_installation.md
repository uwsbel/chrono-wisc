Install the PLANET module {#module_planet_installation}
==========================

[TOC]

Chrono::Planet is an optional module that provides planetary terrain for any
body from GeoTIFF digital elevation models: a stack of DEMs sampled at any
longitude, latitude and level of detail, procedural crater, rock and roughness
layers below the DEM resolution, and a quadtree of terrain tiles that streams
around a moving viewpoint. Nothing in the core is tied to a particular body;
presets for the Moon and Mars live under `chrono_planet/planets`.
Chrono::Vehicle builds two terrain models on it: `PlanetTerrain` (a rigid
collision patch that follows the vehicle) and `PlanetSCMTerrain` (the SCM soil
model over the unbounded surface).

For how to set up a body, its DEMs and its relief, read the
[reference manual](@ref manual_planet).


## Features

- Any spherical body (`chrono::planet::ChPlanetBody`): radius, gravity and the
  geographic coordinate system its DEMs are read in.
- Any raster GDAL can open, geographic or projected, with per-file zoom range,
  band, value scale and offset, radius-to-height conversion and nodata
  handling (`chrono::planet::ChGeoTiffStack`). Several DEMs blend by zoom range
  and feather where one overlays another.
- A deterministic surface (`chrono::planet::ChPlanetSurface`) shared by physics
  and rendering, so wheels and pixels ride one height field.
- Filter chains that transform the sampled heights: procedural relief layers
  (craters, boulders and micro-roughness with configurable size
  distributions), scale, clamp, function and regional filters, preset chains
  per body, and user-defined filters and samplers.
- A local east-north-up frame at a landing site (`chrono::planet::ChSiteFrame`).
- A quadtree of tiles (`chrono::planet::ChPlanetQuadtree`) with a prefetch
  worker and a tile mesh builder, ready for a rendering back end.


## Requirements

- To **build** this module you need:
    - the [GDAL](https://gdal.org) library and headers (any 3.x)
    - optionally OpenMP, which the grid loops use when Chrono is built with it

- To **run** applications based on this module you need:
    - the GDAL shared library
    - optionally, GeoTIFF elevation models. Without any, a surface is its
      procedural relief alone, which is enough for the demos.

Projects that link against an installed Chrono::Planet need the GDAL library
but not its headers: the public headers do not include GDAL.


## Building instructions

1. Install GDAL (on Ubuntu, `libgdal-dev`).

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono)

3. During CMake configuration, set `CH_ENABLE_MODULE_PLANET` to 'on'.
   CMake's own FindGDAL locates the library; set `GDAL_DIR` or `GDAL_ROOT`
   if it is not in a default location.

4. Optional settings:
    - `CH_PLANET_SIMD` selects the instruction set for the terrain grid loops
      (`avx2`, the default, `native`, or `off`). Contraction is always off so
      every build produces the same heights.

5. Press 'Generate' and build as usual.

To use the vehicle terrains, also enable `CH_ENABLE_MODULE_VEHICLE`.

To draw the quadtree terrain, also enable `CH_ENABLE_MODULE_VSG`: this adds
the `Chrono_planet_vsg` library with `ChPlanetVisualizationVSG`, a plugin for
`ChVisualSystemVSG` that streams the tiles around the camera, and the
visual demos.


## Elevation data for the Moon

The Moon preset ships two LOLA elevation models in `data/planet/moon`, and
knows one more that is too large for the repository. Code selects them with
the `moon::Dem` enum, for example
`moon::CreateSurface({moon::Dem::GLOBAL_LOW_RES, moon::Dem::APOLLO17_LANDING_SITE})`:

- `GLOBAL_LOW_RES` (`ldem_4_global.tif`, shipped): global, 4 pixels per degree
- `APOLLO17_LANDING_SITE` (`ldem_1024_apollo_region.tif`, shipped): the
  Apollo 17 region, 1024 pixels per degree
- `GLOBAL` (`ldem_64_fixed.tif`, not shipped): global, 64 pixels per degree.
  To use it, place the file in `data/planet/moon/` before configuring, or in
  the build tree's copy of the data directory.

See `data/planet/moon/README.md` for details. Any other DEM can be passed on
a demo's command line instead.


## Using it from your own project

Build and install Chrono with the module enabled, then in your project's
`CMakeLists.txt` request the component, as in `template_project`:

~~~{.cmake}
find_package(Chrono COMPONENTS Planet Vehicle CONFIG)
target_link_libraries(myapp ${CHRONO_TARGETS})
~~~

`Chrono_DIR` can point to either the Chrono build tree or the install tree.
Your own body presets belong in your project; `planets/moon/ChMoon.cpp` is the
template to copy.


## How to use it

- Read the [reference manual](@ref manual_planet).
- Look at the [API section](group__planet__module.html) of this module for documentation about classes and functions.
- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.

All demos run on the Moon preset. GeoTIFF paths on the command line form the
DEM stack for both physics and rendering. With none, the shipped Moon DEM
resources are used (`GLOBAL_LOW_RES` and `APOLLO17_LANDING_SITE`).

Headless (only `CH_ENABLE_MODULE_PLANET` and `CH_ENABLE_MODULE_VEHICLE`):

- `demo_VEH_PlanetTerrain`: a rigid body settles on a `PlanetTerrain` patch
  that follows it, then a wheel sinks into a `PlanetSCMTerrain`.
- `demo_PLANET_Surface`: a survey of the surface around a landing site. It
  breaks the elevation into its DEM and relief-layer terms across levels of
  detail, samples an elevation grid in the site frame (relief and slope
  statistics, written as CSV and as a Wavefront OBJ), lists the procedural
  boulders by size class, checks the site frame, and streams the quadtree to
  the site to show that its finest mesh and the physics surface agree.

With `CH_ENABLE_MODULE_VSG` (all in `src/demos/planet`). Each shows a "Planet
terrain" panel with a wireframe checkbox and tile counts:

- `demo_PLANET_TerrainVSG`: the minimal setup, a ball dropped onto a
  `PlanetTerrain` patch with the quadtree tiles streamed around the camera.
- `demo_PLANET_Flyover`: a scripted camera descends from a few kilometers up
  onto the site and circles it, with an overlay of the quadtree state. Pass
  `--wireframe` to watch the level of detail change.
- `demo_PLANET_Viper_Rigid`: the VIPER rover drives an S-curve over a
  `PlanetTerrain` patch that is rebuilt as it moves, through a field of
  procedural boulders spawned around it as fixed collision bodies. The overlay
  shows the rover's longitude, latitude and elevation on the planet.
- `demo_PLANET_Viper_SCM`: the same rover on `PlanetSCMTerrain`, with its ruts
  drawn in the terrain tiles through a `ChDeformationFilter` and colored by
  depth, the SCM
  prefetch worker fed the rover's heading, per-wheel sinkage and soil forces in
  the overlay and in a CSV file, and a periodic rut summary. Pass
  `--bulldozing` to enable soil displacement.
- `demo_PLANET_WheeledVehicle` (also needs `CH_ENABLE_MODULE_VEHICLE_MODELS`):
  an HMMWV with rigid tires driven from the keyboard under lunar gravity, using
  the standard Chrono::Vehicle visual system with the terrain plugin attached.
  This shows a `PlanetTerrain` used like any other `ChTerrain`.

The boulder helper the rover and vehicle demos share, `PlanetBoulderField.h`,
shows how to turn the surface's `ChRockLayer` instances into Chrono bodies:
each rock's mesh from `RockMeshes` becomes the visual shape and its coarsest
level of detail the convex collision hull, placed on the rock bed the surface
already carries.
