Chrono::Planet Reference Manual {#manual_planet}
=================================

[TOC]

* [Install and build Chrono::Planet](@ref module_planet_installation)
* [API reference](group__planet__module.html)

Chrono::Planet turns elevation data and procedural relief into terrain on a
spherical body, for physics (through Chrono::Vehicle's `PlanetTerrain` and
`PlanetSCMTerrain`) and for rendering (through a level-of-detail quadtree).
The module has no built-in body. You describe the body, point it at your
DEMs and choose its relief; presets for the Moon and Mars are provided as
ready-made examples of doing exactly that.


# Concepts {#planet_concepts}

| Class | Role |
|---|---|
| `ChPlanetBody` | Name, reference radius, surface gravity and geographic coordinate system of a body. |
| `ChPlanetSurface` | The terrain: a sampler, a fallback where it has no data, and a filter chain. |
| `ChElevationSampler` | A sampler: anything that answers height at a longitude, latitude and zoom. |
| `ChGeoTiffStack` | The usual sampler: GDAL rasters, each serving a range of zooms. |
| `ChSamplerChain` | Several samplers in order; the first with data answers. |
| `ChSurfaceFilter` | A filter: maps the height so far at a point to a new height. |
| `ChFilterChain` | Filters applied in order; itself a filter, so chains nest. |
| `ChReliefLayer` | Procedural relief (craters, rocks, roughness): a filter that adds height. |
| `ChDeformationFilter` | Height changes set at run time, such as soil ruts, reported to the renderer. |
| `ChLevelOfDetail` | What a zoom means: root tile size and vertex spacing per zoom. |
| `ChSiteFrame` | Local east-north-up Cartesian frame at a site, for a Chrono system. |
| `ChPlanetQuadtree` | Streams terrain tiles from a surface around a moving camera. |

Every height in the module is in meters above the body's reference sphere, and
every longitude and latitude is on that sphere. The height at a point is

    height = filter chain( sampler height, or the fallback's where the sampler has no data )

All of these are pure functions of longitude, latitude and level of detail.
That is what lets a physics terrain and a renderer built on the same surface
agree to the millimeter without sharing any state.

**Zoom and spacing.** A *zoom* is a quadtree level: 0 is a root tile (16
degrees by default), and each level halves the tile. The DEM stack picks its
raster by zoom. Filters see a *spacing*: relief features too small
to be resolved at that ground spacing fade out, so a coarse tile and a fine
tile show consistent terrain. Physics queries use the surface's fixed zoom
(`SetZoom`, 15 by default, about 0.2 m on the Moon), whose spacing equals the
vertex spacing of the renderer's mesh at that level.


# Setting up a body {#planet_new_body}

The steps below build a surface for any body. The Moon preset,
`chrono_planet/planets/moon/ChMoon.cpp`, follows the same steps and is the
template to copy for a body of your own, in your own project.

**1. Describe the body.**

~~~{.cpp}
using namespace chrono::planet;
ChPlanetBody body("Europa", 1560800.0, 1.315);  // name, mean radius (m), gravity (m/s^2)
~~~

Bodies are spheres. For a flattened body such as Mars or the Earth, use the
mean radius; elevations referenced to an ellipsoid or a geoid then differ from
heights above the sphere by up to a few kilometers, which `ChGeoTiffSource::offset`
can absorb for a local site.

Projected DEMs are transformed into the body's geographic coordinate system.
By default it is a sphere of the body's radius, which suits most planetary
products. Set another with `SetGeographicSRS`, which takes WKT, a PROJ string
or an authority code such as `IAU_2015:30100` when your PROJ database has it.

**2. Add elevation data.**

~~~{.cpp}
auto surface = chrono_types::make_shared<ChPlanetSurface>(body);
surface->AddGeoTiff({"europa_global.tif", 0, 4});   // coarse global DEM for zooms 0-4
surface->AddGeoTiff({"europa_site.tif", 5, 30});    // regional DEM for zooms 5 and finer
~~~

At each zoom the raster whose range contains it provides the height. Where it
ends or has holes it fades, over 500 m of ground, into the finest coarser
raster that covers the area. `AddGeoTiff` throws `std::runtime_error` with the
reason if a file cannot be used.

Rasters are read through GDAL, so GeoTIFF, PDS, ISIS cubes and the other GDAL
formats all work. `ChGeoTiffSource` describes how to read the values:

| Field | Default | Use |
|---|---|---|
| `path` | | File, absolute or relative to the working directory. |
| `min_zoom`, `max_zoom` | 0, 30 | Zoom range the raster serves. |
| `band` | 1 | Band holding the elevations. |
| `scale`, `offset` | 1, 0 | Convert stored values to meters, for example `scale = 1000` for kilometers. |
| `values_are_radii` | false | Values are distances from the body center; the radius is subtracted. |
| `use_band_scale_offset` | true | Apply the band's own scale/offset metadata first. |
| `nodata` | from file | Override the band's nodata value. |
| `min_valid` | -30000 | Heights below this are treated as nodata. |

Longitudes may run from -180 to 180 or from 0 to 360. A raster with no
coordinate system is read as longitude/latitude on the body. A raster whose
datum radius differs from the body's by more than 1% prints a warning, since
that usually means data from another body or values in the wrong units.

To combine a GeoTIFF stack with samplers of your own, put them in a
`ChSamplerChain` and install it with `ChPlanetSurface::SetSampler`: the first
sampler with data at a point answers.

**3. Choose a fallback.** Where the sampler has no data, the surface uses its
fallback, or 0 m if none is set. `ChNoiseElevation` adds gentle noise so the
ground is never perfectly flat; `ChElevationFunction` wraps any function of
longitude and latitude.

~~~{.cpp}
surface->SetFallback(chrono_types::make_shared<ChNoiseElevation>(1.0, 0.1));  // 1 m, 0.1 cells/degree
~~~

**4. Build the filter chain.** The sampled height passes through the
surface's filter chain, first filter to last. DEMs rarely resolve the
meter-scale features a wheel feels, so the usual chain starts with relief
layers, which add them procedurally. Every layer's default parameters describe
*no* relief, so nothing appears until you configure it:

~~~{.cpp}
ChCraterLayer::Params craters = moon::CraterParams();  // start from a calibrated set
craters.complex_diameter_km = 15.0 * moon::kGravity / body.GetSurfaceGravity();  // ~1/g scaling
surface->AddLayer(chrono_types::make_shared<ChCraterLayer>(body, craters));

ChRockLayer::Params rocks;
rocks.coverage = 0.03;  // Golombek-Rapp k: fraction of the ground covered by rocks
surface->AddLayer(chrono_types::make_shared<ChRockLayer>(body, rocks));
~~~

| Layer | What it adds | Key parameters |
|---|---|---|
| `ChBaseReliefLayer` | Broad swell over the whole body | `frequency`, `amplitude` |
| `ChCraterLayer` | Impact craters with rims, ejecta and central peaks | size classes and densities, depth and rim laws, simple-to-complex transition |
| `ChRockLayer` | The regolith beds boulders sit in; `Query` returns the boulders | `coverage` (k), Golombek-Rapp `qa`, `qb`, size bins |
| `ChRoughnessLayer` | Multi-octave micro-roughness, clods and pits | `slope_rms`, `fine_boost`, wavelengths |

Other filters reshape the terrain:

| Filter | Effect |
|---|---|
| `ChScaleFilter` | `height * scale + offset`: vertical exaggeration or a datum shift |
| `ChClampFilter` | Clamp heights to a range |
| `ChFunctionFilter` | Any function of longitude, latitude, spacing and height |
| `ChRegionFilter` | Apply another filter only inside a longitude/latitude box, feathered at its edges |
| `ChFilterChain` | A sub-chain, for example a preset chain inside your own |

~~~{.cpp}
// Start from the Moon's calibrated chain, then flatten a landing pad and exaggerate the result.
auto chain = moon::FilterChain();
auto flat = chrono_types::make_shared<ChFunctionFilter>([](double, double, double, double) { return -2230.0; });
chain->AddFilter(chrono_types::make_shared<ChRegionFilter>(flat, 30.74, 20.18, 30.76, 20.20, 0.005));
chain->AddFilter(chrono_types::make_shared<ChScaleFilter>(2.0));
surface->SetFilterChain(chain);
~~~

`GetFilterChain()` returns the surface's chain for editing in place
(`AddFilter`, `InsertFilter`, `RemoveFilter`), and `FindFilter<T>()` finds a
filter by type, searching nested chains.

Sizes are in meters and kilometers, not degrees, so the same parameters give
features of the same physical size on any body. Each layer has a `seed` that
varies its realization without changing its statistics. Layer parameters are
validated on construction and throw `std::invalid_argument` if inconsistent.

**5. Place a site and build the terrain.**

~~~{.cpp}
ChSiteFrame site = surface->MakeSiteFrame(lon, lat);  // origin on the surface
sys.SetGravitationalAcceleration(ChVector3d(0, 0, -body.GetSurfaceGravity()));
vehicle::PlanetTerrain terrain(&sys, surface, site);
terrain.Initialize();

auto quadtree = chrono_types::make_shared<ChPlanetQuadtree>(surface, 10);
auto vis_planet = chrono_types::make_shared<ChPlanetVisualizationVSG>(quadtree, site);  // with a "Planet terrain" panel
~~~

Configure the surface fully before sharing it: queries are thread-safe,
configuration is not.

The quadtree refines a tile once the camera comes within one tile width of it
(`ChPlanetQuadtree::SetSplitDistance`, in tile widths), and does not refine
tiles below the camera's horizon (`SetHorizonMargin`, the terrain depth assumed
between them, 1 km by default; negative turns it off). Raise the split distance
for more detail at a distance, at the cost of more tiles.


# Run-time deformation {#planet_deformation}

Filters are normally fixed for the whole run, so physics and rendering can
evaluate them independently. A `ChDeformationFilter` is the exception: it holds
height changes on a grid in a site frame, set while the simulation runs, and it
reports each change so the quadtree rebuilds the tiles fine enough to show it.
Coarse tiles never see it: the changes fade out between two and four grid
spacings of mesh resolution.

Keep it out of the surface physics samples. A soil model takes its undisturbed
ground from the surface, and would count its own ruts twice. Instead, draw a
*view* of the surface: `CreateView` returns a surface that shares the physics
surface's samplers and runs its chain followed by extra filters.

~~~{.cpp}
vehicle::PlanetSCMTerrain terrain(&sys, surface, site);          // SCM samples the undisturbed surface
terrain.Initialize(params, wheels);
auto ruts = terrain.MakeDeformationFilter();                       // site frame and grid of the SCM terrain
auto quadtree = chrono_types::make_shared<ChPlanetQuadtree>(surface->CreateView(ruts), 10);

while (vis->Run()) {
    terrain.PublishDeformation();  // copy the current sinkage into the filter
    vis->Render();                 // the quadtree rebuilds the tiles the ruts touch
    ...
}
~~~

Ruts are usually only a few centimeters deep, which uniform shading hardly
shows. `ChPlanetVisualizationVSG::SetDeformationColoring(ruts, 0.05)` colors
the drawn terrain by depth, like SCM's sinkage plot, and adds a checkbox for it
to the plugin's panel.

Any filter can take part: override `ChSurfaceFilter::GetChanges` to report the
region a change touched. `ChPlanetQuadtree::Invalidate` rebuilds a region on
demand for changes made outside the chain.


# CRM soil under a rover {#planet_crm}

With Chrono::FSI, `vehicle::PlanetCRMWindow` runs CRM soil (SPH continuum
soil) in a window of the site that follows a rover, over the terrain the rest
of the scene draws. The window is seeded from the ground's heights
(`vehicle::PlanetCRMTerrain::ConstructFromHeight`); when the rover nears its
edge, a new window is seeded ahead of it, and the soil of the old one where the
two overlap is carried over as it is, particles with their stress, so the rover
does not sink into fresh soil. The wheels, added with `AddWheel`, stamp their
footprints as ruts into a `ChDeformationFilter`, and `Publish` writes them, with
the berms of soil the wheels push up beside them, so the drawn terrain and the
sensors see them. `UpdateLooseSoil` meshes the soil thrown up or pushed aside
(`ChSparseSdfGrid`, clods of about the particle spacing), and `MeasureEjecta`
reports the soil thrown into free flight at each wheel, for a `vehicle::ChDustField`.

~~~{.cpp}
PlanetCRMWindow soil(sys, surface, site, ruts, spacing, [&](PlanetCRMTerrain& crm) {
    crm.SetCrmSPH(soil_properties);
    crm.SetSPHParameters(sph_parameters);
    for (auto& wheel : wheels)
        crm.AddRigidBody(wheel, wheel_geometry, true);
    crm.SetActiveDomain(ChVector3d(0.5, 0.5, 0.5));
});
soil.SetWindow(5.0, 3.5, 0.25, 1.3);  // length, width, depth, margin (m)
for (auto& wheel : wheels)
    soil.AddWheel(wheel, wheel_radius, wheel_width);
soil.Initialize(chassis);

while (...) {
    soil.Advance(step);           // moves the window when the rover nears its edge
    if (render)
        soil.Publish();           // ruts and berms into the deformation filter
}
~~~

Where other ground takes over part of the site, `ChPlanetVisualMesh::AddHole`,
`ChPlanetVisualizationVSG::AddHole` and `vehicle::PlanetTerrain::AddHole` leave
a rectangle out of the drawn tiles and the collision patch, cutting the faces
that cross its edges along them.

See `demo_PLANET_Viper_CRM` (VSG), and, through Chrono::Sensor with the Hapke
BRDF, `demo_PLANET_Viper_CRM_Sensor`, `demo_PLANET_Viper_CRM_HillClimb` (up a
slope, until the wheels slip and dig in) and
`demo_PLANET_Viper_CRM_CraterFording` (through a small crater). CRM soil is far
slower than real time: at 0.03 m spacing with two cameras, about 0.05x.


# Module layout {#planet_layout}

Each folder of `src/chrono_planet` holds one concern, and depends only on those
listed before it:

| Folder | Concern |
|---|---|
| `core/` | Internal math, noise, hashing, threading and profiling |
| (top level) | The body, site frame, level-of-detail schedule, geographic grid, and the surface |
| `samplers/` | The sampler interface and generic samplers |
| `dem/` | GeoTIFF stacks (GDAL stays inside this folder's sources) |
| `filters/` | The filter interface, filter chains, general filters, deformation |
| `procedural/` | Relief layers and their internal field helpers |
| `geometry/` | Rock shape meshes, for rendering and collision, and meshes of splatted spheres (`ChSparseSdfGrid`) |
| `lod/` | The quadtree and tile mesh building; `ChPlanetQuadtree.h` and `ChTileMesh.h` are its public face |
| `planets/` | Body presets, built only on the public API; the Moon's is split into terrain (`ChMoon.h`) and DEM resources (`ChMoonDem.h`) |
| `visualization/` | The VSG plugins (terrain, and meshes that change from frame to frame), in their own library |

The terrain model never includes rendering code: the surface knows zoom
levels only through `ChLevelOfDetail`.


## Following an elevation query

For developers changing this repository, start in `ChPlanetSurface.cpp`:

1. `GetElevationAtZoom` reads the sampler, uses the fallback if data is
   missing, then applies the filter chain.
2. `GetElevationGrid` follows the same sequence. `SampleElevationGrid` fills
   the grid from the sampler and fills its holes from the fallback;
   `ApplyFilters` then runs the chain, handling longitude wrapping.
3. The renderer can cache `GetStaticElevationGrid` and later call
   `ApplyDynamicFilters`. This is an optimization of the same calculation:
   the split is immediately before the first dynamic filter, and all filters
   after it run again in their original order.

For DEM sampling, `ChGeoTiffStack.cpp` owns source selection, raster
reconstruction and blending. `GetHeight` samples a one-point grid through
`GetHeightGrid`, so both use the same reconstruction. Its private `Impl`
holds GDAL resources; `GeoTIFFLoader` handles reading raster files.
`reconstructGrid` follows three steps: `mapToRaster` converts geographic
coordinates to pixel coordinates, `prepareAxisSamples` prepares the four
cubic B-spline taps for each axis, and `interpolateHeights` runs horizontal
and vertical passes. `AxisSamples` groups pixel indices, weights and validity.
A failed mapping returns `nullopt`; a mapped sample without usable data has
zero blend weight. Only the final public height grid represents missing data
as NaN.

`CoordinateTraits<Spherical>::elevationGrid` samples mesh heights before
conversion to planet-centered Cartesian positions, including the optional
cache of static heights.

Users linking Chrono from an external project need only the public surface
and sampler APIs above; the caching and GDAL implementation stay internal.


## Cartesian terrain tiles

The coordinate templates support both `Spherical` and `Cartesian`.
Cartesian bounds and camera positions use local east/north/up meters in a
`ChSiteFrame`. The same planet surface supplies the heights; z is elevation
minus the site's origin elevation. This is a local site approximation, not a
second global projection, and grids must stay outside the polar caps.

For developers using the tile-building API:

~~~{.cpp}
#include "chrono_planet/lod/CartesianCoordinates.h"
#include "chrono_planet/lod/QuadtreeTile.h"

ChSiteFrame site = surface->MakeSiteFrame(30.75, 20.19);
Cartesian::Boundary bounds{0, 0, 100, 100, site};  // 200 m square
TileMeshBuilder<Cartesian> builder(surface);
auto mesh = builder.build(bounds, 5);

QuadtreeTile<Cartesian> tile(bounds, surface);
LodParams lod{200, -1, surface->GetBody().GetRadius()};
tile.updateLOD({0, 0, 50}, lod);  // local camera; no spherical horizon culling
~~~

Splits build all four child meshes synchronously. `invalidate` takes a local
x/y rectangle and a spacing threshold in meters; call it when the surface
changes. Static height caching, parent interpolation and skirts are shared
with spherical tiles. Cartesian sampling currently evaluates geographic
points individually, so large procedural grids can cost more than spherical
grid sampling.

`ChPlanetQuadtree` and `ChPlanetVisualizationVSG` remain the global spherical
streamer and renderer. A Cartesian consumer uses the template tile API and
interprets mesh centers and vertex positions directly in its site frame.

## On-demand sampling

Both spherical and Cartesian LOD tiles are built when `Update` or `updateLOD`
needs them; there is no background prefetch queue or camera extrapolation.
A split is fully built before the update returns. This simplifies ownership
and shutdown, but entering detailed terrain can make an update take longer.
SCM likewise samples previously unseen nodes on demand and retains its height
cache. Its deformation-statistics worker is independent and remains available.

External users should remove calls to `SetPrefetchBudget`, `GetPendingMeshes`,
and `PlanetSCMTerrain::SetPrefetchPose`, and stop setting SCM's `prefetch`,
`prefetch_lookahead`, and `prefetch_half_width` parameters. `TileBuildWorker`
and `PlanetSCMHeightFunctor::Warm` have been removed.

## Removed shading APIs

The unused baked normal/height maps and standalone Planet Hapke model have
been removed. External users must drop the third `shading_bakes` constructor
argument from `ChPlanetQuadtree`; `ChTileBake`, `ChTileMesh::bake`, and
`ChMoonPhotometry.h` are no longer provided. VSG uses its existing PBR material.


# Extending the module {#planet_extending}

**Your own sampler.** Derive from `ChElevationSampler` and implement
`GetHeight(lon, lat, zoom)`, returning `std::nullopt` where you have no data.
Override `GetHeightGrid` if you can evaluate a whole grid faster than point by
point; it marks samples with no data as NaN. Install it with `ChPlanetSurface::SetSampler`, add it to a
`ChSamplerChain`, or use it as the fallback.

**Your own filter.** Derive from `ChSurfaceFilter` and implement
`Apply(lon, lat, spacing, height)`, returning the new height. It must be
deterministic and thread-safe. Override `ApplyGrid` for speed; it must match
`Apply` to rounding. For a filter that only adds height, derive from
`ChReliefLayer` instead.

**Your own relief layer.** Derive from `ChReliefLayer` and implement
`GetHeight(lon, lat, spacing)`. It must be deterministic and thread-safe, and
should fade out features smaller than a few `spacing`s. Override `AddToGrid`
for speed; it must match `GetHeight` to rounding.

**Finding filters.** `ChPlanetSurface::FindFilter<T>()` returns the first
filter of a type, for example to query boulders from the surface's `ChRockLayer`.


# Presets {#planet_presets}

Presets live in `chrono_planet/planets` and use only the public API.

- `chrono::planet::moon`: radius 1737.4 km, gravity 1.62 m/s^2, crater
  classes from 80 km down to 1.25 m with the Pike (1977) lunar depth and rim
  laws, a 2% boulder cover, regolith roughness, and meter-scale fallback noise.
  `moon::FilterChain()` returns the preset chain. LOLA elevation models are
  resources selected with the `moon::Dem` enum:

  | `moon::Dem` | Model | Zooms | Shipped |
  |---|---|---|---|
  | `GLOBAL_LOW_RES` | global, 4 px/deg (~7.6 km) | 0-2 | yes |
  | `GLOBAL` | global, 64 px/deg (~474 m) | 0-4 | no, 480 MB |
  | `APOLLO17_LANDING_SITE` | Apollo 17 region, 1024 px/deg (~30 m) | 5-30 | yes |

  ~~~{.cpp}
  auto surface = moon::CreateSurface({moon::Dem::GLOBAL_LOW_RES, moon::Dem::APOLLO17_LANDING_SITE});
  moon::AddDem(*other_surface, moon::Dem::GLOBAL);  // once ldem_64_fixed.tif is installed
  ~~~

  Resources live in `data/planet/moon` (see its README). `moon::CreateSurface`
  also accepts any `ChGeoTiffSource` list, or none.
- `chrono::planet::mars`: radius 3389.5 km, gravity 3.721 m/s^2, with
  `mars::FilterChain()`. Its relief
  parameters are **not calibrated**: they reuse the lunar crater shapes with
  the transition diameter scaled by 1/gravity and a 5% boulder cover, as a
  starting point to tune against site data.


# Known limitations {#planet_limitations}

- Bodies are spheres; there is no ellipsoid.
- Each raster pixel's value is placed at the pixel's geotransform corner,
  regardless of the file's `AREA_OR_POINT` registration. Area-registered
  rasters, the GeoTIFF default, therefore read half a pixel to the north-west.
- A global raster is not bridged across its own wrap edge, so its last pixel
  column before that edge reads as no data (and takes the fallback).
- `vehicle::PlanetCRMWindow` reseeds its whole window when it moves: the soil
  model restarts from the particles of the overlap, and the soil left behind
  keeps only the ruts and berms written into the deformation filter.
- The quadtree's tile skirt depths are fixed in meters and were tuned on the
  Moon. On a much larger body, cracks between tiles of different levels may
  show at coarse levels.
