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
// Procedural boulder field with a Golombek-Rapp size-frequency distribution.
//
// =============================================================================

#ifndef CH_ROCK_LAYER_H
#define CH_ROCK_LAYER_H

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "chrono_planet/ChPlanetBody.h"
#include "chrono_planet/procedural/ChCraterLayer.h"
#include "chrono_planet/procedural/ChExposureField.h"
#include "chrono_planet/procedural/ChReliefLayer.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// One boulder placement, deterministic from its grid cell.
struct CH_PLANET_API ChRockInstance {
    std::uint64_t id = 0;                   ///< stable and unique per rock
    double lonDeg = 0.0, latDeg = 0.0;      ///< center (degrees)
    float radiusM = 0.f;                    ///< half the longest axis (m)
    float yawRad = 0.f;                     ///< spin about the local up
    float tiltRad = 0.f, tiltAzRad = 0.f;   ///< lean off vertical, and its azimuth
    float buryFrac = 0.f;                   ///< fraction of the height below ground
    std::uint16_t meshId = 0;               ///< shape index, taken modulo RockMeshes::kCount
    std::uint8_t sizeClass = 0;             ///< 0 = finest class
};

/// Boulders scattered over the body, a pure function of longitude and latitude.
/// Rock counts follow the Golombek and Rapp (1997) model, where the cumulative fractional area
/// covered by rocks of diameter D or larger is F(D) = k exp(-q(k) D), with q(k) = qa + qb / k.
/// Rocks are binned into size classes (one per octave of diameter). As a relief layer, each rock
/// contributes the shallow bed of regolith it sits in; the rocks themselves are separate bodies or
/// meshes that callers place with Query(), GetOrientation() and GetCenterRise().
///
/// Rock abundance can vary from place to place (coverage_spread): k then rises and falls in patches about its
/// given value, and each patch has the sizes of its own k, so a rocky patch holds the larger rocks too.
///
/// The default parameters describe no rocks (k = 0).
class CH_PLANET_API ChRockLayer : public ChReliefLayer {
  public:
    struct Params {
        double coverage = 0;  ///< k, the cumulative fractional area covered by rocks (e.g. 0.02)
        double qa = 1.79;     ///< Golombek-Rapp q(k) = qa + qb / k, per meter of diameter
        double qb = 0.152;    ///< Golombek-Rapp q(k) = qa + qb / k, per meter of diameter
        /// Class diameter bin edges (m), strictly increasing; one class per bin.
        std::vector<double> bin_edges_m = {0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0};
        /// Patchiness. k varies from coverage / coverage_spread to coverage * coverage_spread, in patches about
        /// patch_size (m) wide. 1, the default, is the same k everywhere. k is held under 0.5.
        double coverage_spread = 1;
        double patch_size = 200;
        /// How much of the range from the poorest k to the richest the patches span by themselves, from the poorest
        /// up: 1, the default, all of it. Less leaves the upper part to where bedrock shows or craters threw rocks
        /// out, so the ground away from those is poor in rocks throughout.
        double patch_share = 1;
        /// Where bedrock shows, if given: rocks break out of it, so with patchy cover (coverage_spread over 1) the
        /// ground it says is bedrock is rockier, up to halfway from its patch's k to the richest.
        std::shared_ptr<const ChExposureField> bedrock;
        /// The craters that throw rocks out, if given. A crater that digs through the soil to rock under it leaves
        /// blocks on its rim and round it, thickest at the rim and thinning outward, and they are broken down in
        /// time: few are left round an old one. With patchy cover, the ground within `ejecta_reach` radii of the
        /// center of a crater `ejecta_diameter` (m) across or more is rockier: up to the richest k at the rim of a
        /// newly made one, falling to its patch's own at the reach, and to nothing for a crater whose `freshness`
        /// is 0.7 or less. 0 for the diameter, the default: none.
        /// After Watkins et al., "Boulder distributions around young, small lunar impact craters and implications
        /// for regolith production rates and landing site safety", JGR Planets 124, 2019: the densest at the rim,
        /// the larger within 2 to 4 radii, few left after some hundred million years. The shares are not theirs.
        std::shared_ptr<const ChCraterLayer> craters;
        double ejecta_diameter = 0;
        double ejecta_reach = 3;
        int seed = 0;  ///< varies the realization; 0 reproduces the reference field
    };

    /// Construct the layer for a body. Throws std::invalid_argument on inconsistent parameters.
    ChRockLayer(const ChPlanetBody& body, const Params& params);
    ~ChRockLayer();

    const Params& GetParams() const;

    /// Number of size classes.
    int GetNumClasses() const;
    /// Largest diameter (m) of a class, 0 outside the range.
    double GetClassDiameter(int size_class) const;
    /// Largest radius (m) any rock can have.
    double GetMaxRadius() const;
    /// The cumulative fractional area k at a point. The same everywhere unless coverage_spread is over 1.
    double GetCoverage(double lon_deg, double lat_deg) const;

    /// Rocks with radius >= min_radius (m) whose centers lie in a longitude/latitude rectangle (degrees),
    /// half-open on the max edges so adjacent queries neither repeat nor drop a rock.
    std::vector<ChRockInstance> Query(double min_lon,
                                      double min_lat,
                                      double max_lon,
                                      double max_lat,
                                      double min_radius) const;

    /// Unit quaternion (x, y, z, w) of a rock's yaw and lean, in its local east-north-up frame.
    static std::array<double, 4> GetOrientation(const ChRockInstance& rock);

    /// Height of the rock mesh origin above the ground once buried, for a mesh whose lowest point is
    /// `bottom_extent` below its origin and whose height is `total_height` (both m, after orientation).
    static double GetCenterRise(const ChRockInstance& rock, double bottom_extent, double total_height);

    /// Relief (m) of the rock beds at a point.
    virtual double GetHeight(double lon_deg, double lat_deg, double spacing_deg) const override;
    virtual void AddToGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;

    struct Model;

  private:
    std::unique_ptr<const Model> m_model;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
