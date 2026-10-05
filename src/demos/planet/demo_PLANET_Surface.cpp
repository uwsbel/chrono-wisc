// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: bgwitt
// =============================================================================
//
// Headless survey of a planetary surface, with no visualization. Around a lunar
// landing site this demo
//   - shows how the level of detail changes the sampled elevation (each zoom
//     resolves finer crater octaves),
//   - samples an elevation grid in the site frame, reports relief and slope
//     statistics and writes it out as CSV and as a Wavefront OBJ mesh,
//   - lists the procedural boulders in the same area by size class,
//   - checks the site frame round trip and the ground distance of a traverse,
//   - and, if the DEM stack can be opened, streams the quadtree to the site and
//     compares its finest resident mesh against the physics surface, which is
//     the agreement PlanetTerrain and the renderer rely on.
//
// Command line: GeoTIFF paths form the DEM stack; with none, the Moon DEM
// resources shipped with Chrono (global low resolution plus the Apollo 17
// landing site) are used.
//
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/lod/ChPlanetQuadtree.h"
#include "chrono_planet/planets/moon/ChMoon.h"

#include "PlanetDemoSetup.h"

using namespace chrono;
using namespace chrono::planet;

// Landing site (Apollo 17 region) and the quadtree zoom the physics surface is sampled at.
const double site_lon = 30.75;
const double site_lat = 20.19;
const int zoom = 15;

// Survey grid in the site frame: n x n samples at this spacing, centered on the site.
const int grid_n = 201;
const double grid_step = 0.5;

// Quadtree root tile size (deg) and ring half-width (tiles).
const double root_tile_deg = 16.0;
const int view_range_tiles = 10;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::vector<ChGeoTiffSource> dems;
    for (int i = 1; i < argc; ++i)
        dems.push_back({argv[i], 0, 30});

    const std::string out_dir = GetChronoOutputPath() + "PLANET_Surface";
    if (!CreateOutputDirectory(out_dir)) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::cout << std::fixed;

    // ---------------------------------------------------------------
    // 1. Elevation at the site across levels of detail
    // ---------------------------------------------------------------
    std::cout << "\n--- Site " << site_lon << " E, " << site_lat << " N ---\n";
    std::cout << "DEM stack:\n";
    for (const auto& d : dems)
        std::cout << "  " << d.path << "  (zooms " << d.min_zoom << "-" << d.max_zoom << ")\n";
    if (dems.empty())
        for (auto r : {moon::Dem::GLOBAL_LOW_RES, moon::Dem::APOLLO17_LANDING_SITE}) {
            const ChGeoTiffSource d = moon::GetDem(r);
            std::cout << "  " << moon::GetDemDescription(r) << ": " << d.path << "  (zooms " << d.min_zoom << "-"
                      << d.max_zoom << ")\n";
        }
    // One surface at the physics zoom, shared with the quadtree below. Its layers are queried one by one to
    // show how each term changes with zoom.
    auto surface = CreateDemoSurface(dems, zoom, root_tile_deg);
    if (!surface)
        return 1;
    const auto base_layer = surface->FindFilter<ChBaseReliefLayer>();
    const auto crater_layer = surface->FindFilter<ChCraterLayer>();
    const auto rock_layer = surface->FindFilter<ChRockLayer>();
    const auto roughness_layer = surface->FindFilter<ChRoughnessLayer>();

    std::cout << "\nElevation terms at the site by sampling zoom. The DEM stack serves each zoom from its\n"
                 "finest source for that zoom; the procedural layers only keep the features a mesh at that\n"
                 "zoom resolves, so finer zooms add finer craters, rock beds and roughness (spacing is the\n"
                 "tile vertex spacing):\n";
    std::cout << std::setprecision(4);
    const double base = base_layer->GetHeight(site_lon, site_lat, 0);
    for (int z = 8; z <= 17; ++z) {
        const double spacing = surface->GetSampleSpacingAtZoom(z);
        const auto dem = surface->GetDataElevation(site_lon, site_lat, z);
        const double craters = crater_layer->GetHeight(site_lon, site_lat, spacing);
        const double beds = rock_layer->GetHeight(site_lon, site_lat, spacing);
        const double rough = roughness_layer->GetHeight(site_lon, site_lat, spacing);
        std::cout << "  zoom " << std::setw(2) << z << ": spacing " << std::setw(9)
                  << spacing * surface->GetBody().GetMetersPerDegree() << " m, DEM " << std::setw(11)
                  << (dem ? *dem : 0.0) << (dem ? " m" : " m (none: procedural fallback)") << ", base relief "
                  << std::setw(8) << base << " m, craters " << std::setw(8) << craters << " m, rock beds "
                  << std::setw(7) << beds << " m, roughness " << std::setw(7) << rough << " m\n";
    }

    const ChSiteFrame site = surface->MakeSiteFrame(site_lon, site_lat);
    std::cout << "\nUsing zoom " << zoom << ": site origin elevation " << site.GetOriginElevation()
              << " m (DEM + every layer at that zoom)\n";
    std::cout << "Procedural roughness RMS slope resolved at this zoom: "
              << roughness_layer->GetRmsSlope(surface->GetSampleSpacing()) << " (rise/run)\n";

    // ---------------------------------------------------------------
    // 2. Elevation grid in the site frame: statistics, CSV and OBJ
    // ---------------------------------------------------------------
    std::cout << "\n--- Survey grid: " << grid_n << " x " << grid_n << " at " << grid_step << " m ---\n";
    const double half = 0.5 * (grid_n - 1) * grid_step;
    // Grid steps in degrees so the batch sampler can be used; x/y of each node come from the site frame.
    double lon0, lat0, lon1, lat1;
    site.ToLonLat(-half, -half, lon0, lat0);
    site.ToLonLat(half, half, lon1, lat1);
    const double step_lon = (lon1 - lon0) / (grid_n - 1);
    const double step_lat = (lat1 - lat0) / (grid_n - 1);

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<double> elev;
    surface->GetElevationGrid(lon0, lat0, step_lon, step_lat, grid_n, elev);
    const double sample_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    std::vector<ChVector3d> nodes(elev.size());
    double zmin = 1e300, zmax = -1e300, zsum = 0;
    for (int j = 0; j < grid_n; ++j)
        for (int i = 0; i < grid_n; ++i) {
            const double z = elev[j * grid_n + i] - site.GetOriginElevation();
            nodes[j * grid_n + i] = site.ToLocal(lon0 + i * step_lon, lat0 + j * step_lat, elev[j * grid_n + i]);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
            zsum += z;
        }
    // Slopes by central differences, in the interior
    double slope_sum2 = 0, slope_max = 0;
    int slope_count = 0;
    for (int j = 1; j < grid_n - 1; ++j)
        for (int i = 1; i < grid_n - 1; ++i) {
            const double dzdx = (nodes[j * grid_n + i + 1].z() - nodes[j * grid_n + i - 1].z()) /
                                (nodes[j * grid_n + i + 1].x() - nodes[j * grid_n + i - 1].x());
            const double dzdy = (nodes[(j + 1) * grid_n + i].z() - nodes[(j - 1) * grid_n + i].z()) /
                                (nodes[(j + 1) * grid_n + i].y() - nodes[(j - 1) * grid_n + i].y());
            const double slope = std::sqrt(dzdx * dzdx + dzdy * dzdy);
            slope_sum2 += slope * slope;
            slope_max = std::max(slope_max, slope);
            ++slope_count;
        }
    std::cout << std::setprecision(3);
    std::cout << "  sampled " << elev.size() << " nodes in " << sample_ms << " ms\n";
    std::cout << "  relief about the site: min " << zmin << " m, max " << zmax << " m, mean " << zsum / elev.size()
              << " m\n";
    std::cout << "  slope: RMS " << std::sqrt(slope_sum2 / slope_count) << " (" 
              << std::atan(std::sqrt(slope_sum2 / slope_count)) * CH_RAD_TO_DEG << " deg), max " << slope_max << " ("
              << std::atan(slope_max) * CH_RAD_TO_DEG << " deg)\n";

    {
        ChWriterCSV csv(",");
        csv << "x,y,z,lon,lat,elevation" << std::endl;
        for (int j = 0; j < grid_n; ++j)
            for (int i = 0; i < grid_n; ++i) {
                const auto& p = nodes[j * grid_n + i];
                csv << p.x() << p.y() << p.z() << lon0 + i * step_lon << lat0 + j * step_lat << elev[j * grid_n + i]
                    << std::endl;
            }
        csv.WriteToFile(out_dir + "/heightmap.csv");
        std::cout << "  wrote " << out_dir << "/heightmap.csv\n";
    }
    {
        ChTriangleMeshConnected mesh;
        mesh.GetCoordsVertices() = nodes;
        auto& tris = mesh.GetIndicesVertices();
        for (int j = 0; j < grid_n - 1; ++j)
            for (int i = 0; i < grid_n - 1; ++i) {
                const int a = j * grid_n + i, b = a + 1, c = a + grid_n, d = c + 1;
                tris.push_back(ChVector3i(a, b, d));
                tris.push_back(ChVector3i(a, d, c));
            }
        ChTriangleMeshConnected::WriteWavefront(out_dir + "/patch.obj", {mesh});
        std::cout << "  wrote " << out_dir << "/patch.obj (" << tris.size() << " triangles)\n";
    }

    // ---------------------------------------------------------------
    // 3. Boulders in the survey area
    // ---------------------------------------------------------------
    std::cout << "\n--- Boulders in the survey area (" << 2 * half << " m square) ---\n";
    const auto rocks = rock_layer->Query(lon0, lat0, lon1, lat1, 0.0);
    std::vector<int> per_class(rock_layer->GetNumClasses(), 0);
    double covered = 0;
    for (const auto& r : rocks) {
        ++per_class[r.sizeClass];
        covered += CH_PI * r.radiusM * r.radiusM;
    }
    const double area = (2 * half) * (2 * half);
    std::cout << "  " << rocks.size() << " rocks, covering " << 100.0 * covered / area << " % of the area\n";
    for (int c = 0; c < rock_layer->GetNumClasses(); ++c)
        std::cout << "  class " << c << " (diameter up to " << std::setw(6) << rock_layer->GetClassDiameter(c)
                  << " m): " << std::setw(6) << per_class[c] << "  (" << std::setw(8)
                  << per_class[c] / area * 1e4 << " per hectare)\n";
    {
        ChWriterCSV csv(",");
        csv << "id,x,y,z_ground,radius,size_class,yaw_deg,tilt_deg,bury_frac,mesh_id" << std::endl;
        for (const auto& r : rocks) {
            const ChVector3d p = site.ToLocal(r.lonDeg, r.latDeg, surface->GetElevation(r.lonDeg, r.latDeg));
            csv << r.id << p.x() << p.y() << p.z() << r.radiusM << static_cast<int>(r.sizeClass)
                << r.yawRad * CH_RAD_TO_DEG << r.tiltRad * CH_RAD_TO_DEG << r.buryFrac << r.meshId << std::endl;
        }
        csv.WriteToFile(out_dir + "/rocks.csv");
        std::cout << "  wrote " << out_dir << "/rocks.csv\n";
    }

    // ---------------------------------------------------------------
    // 4. Site frame: round trip and a 1 km traverse
    // ---------------------------------------------------------------
    std::cout << "\n--- Site frame ---\n";
    double worst = 0;
    for (double x : {-1000.0, -50.0, 0.0, 300.0, 1000.0})
        for (double y : {-1000.0, 0.0, 700.0}) {
            double lon, lat;
            site.ToLonLat(x, y, lon, lat);
            const ChVector3d back = site.ToLocal(lon, lat, site.GetOriginElevation());
            worst = std::max(worst, (back - ChVector3d(x, y, 0)).Length());
        }
    std::cout << std::setprecision(9) << "  round-trip error over +-1 km: " << worst << " m\n" << std::setprecision(3);
    {
        // Ground distance of a 1 km eastward line, accumulated over the surface it crosses
        const int n = 2000;
        double length = 0, climb = 0, descent = 0;
        ChVector3d prev(0, 0, 0);
        for (int i = 0; i <= n; ++i) {
            const double x = 1000.0 * i / n;
            double lon, lat;
            site.ToLonLat(x, 0, lon, lat);
            const ChVector3d p(x, 0, surface->GetElevation(lon, lat) - site.GetOriginElevation());
            if (i > 0) {
                length += (p - prev).Length();
                const double dz = p.z() - prev.z();
                (dz > 0 ? climb : descent) += std::abs(dz);
            }
            prev = p;
        }
        double lon, lat;
        site.ToLonLat(1000.0, 0, lon, lat);
        std::cout << "  1 km due east ends at " << lon << " E, " << lat << " N, elevation " << prev.z()
                  << " m above the site;\n  path length over the surface " << length << " m, total climb " << climb
                  << " m, total descent " << descent << " m\n";
    }

    // ---------------------------------------------------------------
    // 5. Quadtree agreement: stream tiles to the site and compare heights
    // ---------------------------------------------------------------
    std::cout << "\n--- Quadtree agreement ---\n";
    {
        ChPlanetQuadtree world(surface, view_range_tiles);
        // One update builds all terrain required at this camera position.
        const ChVector3d cam = surface->GetBody().ToCartesian(site_lon, site_lat, site.GetOriginElevation() + 2.0);
        world.Update(cam, 0.0);
        std::cout << "  " << world.GetTick() << " LOD ticks, " << world.GetNumRootTiles() << " root tiles, "
                  << world.GetMeshes().size() << " leaf meshes resident\n";
        std::cout << "  camera on the tree: " << world.GetCameraLonLat().x() << " E, "
                  << world.GetCameraLonLat().y() << " N, " << world.GetCameraElevation()
                  << " m\n";
        double worst_diff = 0;
        int compared = 0;
        for (double x : {-20.0, -5.0, 0.0, 5.0, 20.0})
            for (double y : {-20.0, 0.0, 20.0}) {
                double lon, lat;
                site.ToLonLat(x, y, lon, lat);
                if (auto h = world.GetElevation(lon, lat, zoom)) {
                    worst_diff = std::max(worst_diff, std::abs(*h - surface->GetElevation(lon, lat)));
                    ++compared;
                }
            }
        std::cout << std::setprecision(6) << "  " << compared << " points compared at zoom " << zoom
                  << ", largest |quadtree - surface| = " << worst_diff << " m\n";
    }

    return 0;
}
