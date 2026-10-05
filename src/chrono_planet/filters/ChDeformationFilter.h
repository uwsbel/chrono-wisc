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
// A run-time filter holding height changes on a grid in a site frame, for
// example the ruts a deformable soil model leaves.
//
// =============================================================================

#ifndef CH_DEFORMATION_FILTER_H
#define CH_DEFORMATION_FILTER_H

#include <algorithm>
#include <cstdint>
#include <deque>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "chrono/core/ChVector2.h"

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChSiteFrame.h"
#include "chrono_planet/filters/ChSurfaceFilter.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Height changes (m) on a regular grid in a site frame, added to the terrain and updated at run time.
/// Node (i, j) sits at site coordinates (i * spacing, j * spacing); heights between nodes are bilinear,
/// and nodes never set count as zero. vehicle::PlanetSCMTerrain publishes its soil deformation into one.
///
/// Unlike other filters, this one changes while a simulation runs, and it reports what changed through
/// GetChanges, so a ChPlanetQuadtree drawing a surface with it rebuilds the affected tiles. Keep it out of
/// the chain a physics model samples its undisturbed terrain from (see ChPlanetSurface::CreateView): a
/// soil model that saw its own ruts in its reference surface would count them twice.
///
/// The changes fade out on meshes too coarse to show them: fully visible at a relief spacing up to twice
/// the node spacing, and gone at four times, so coarse tiles never alias a rut into a spike.
///
/// A node can also carry its height after the change (SetNodes). Where the ground has been lowered, the
/// terrain then follows those heights rather than the incoming terrain moved down, so relief finer than the
/// grid, which a wheel presses flat, fades out of compacted ground (see SetFlattenDepth).
/// All methods are thread-safe.
class CH_PLANET_API ChDeformationFilter : public ChSurfaceFilter {
  public:
    /// A grid of the given node spacing (m) in a site frame.
    ChDeformationFilter(const ChSiteFrame& site, double spacing);

    /// A changed node: its height change (m) and its height after the change, above the reference sphere (m),
    /// or NaN if unknown.
    struct Node {
        ChVector2i index;
        double delta;
        double height;
    };

    const ChSiteFrame& GetSiteFrame() const { return m_site; }
    double GetSpacing() const { return m_spacing; }

    /// Set the height change of grid nodes (m). Nodes whose value moves by no more than `tolerance`
    /// are left alone. Returns the number of nodes changed; if any, records a change for GetChanges.
    size_t SetDeltas(const std::vector<std::pair<ChVector2i, double>>& deltas, double tolerance = 1e-3);

    /// Set the height change of grid nodes, with their heights after the change. As SetDeltas otherwise.
    size_t SetNodes(const std::vector<Node>& nodes, double tolerance = 1e-3);

    /// Lowering (m) over which compacted ground goes from the incoming terrain moved down to the nodes' own
    /// heights, where nodes carry them (default: 0.01). 0 keeps the incoming terrain's relief everywhere.
    void SetFlattenDepth(double depth) { m_flatten_depth = depth; }
    double GetFlattenDepth() const { return m_flatten_depth; }

    /// How much of the incoming terrain's finer relief compacted ground loses, from 0 (none) to 1 (all, the default):
    /// a wheel smooths the regolith it presses without polishing it.
    void SetFlattenStrength(double strength) { m_flatten_strength = std::clamp(strength, 0.0, 1.0); }
    double GetFlattenStrength() const { return m_flatten_strength; }

    /// Remove every change, recording one covering them all.
    void Clear();

    /// Height change of a node (0 if never set).
    double GetDelta(const ChVector2i& node) const;

    /// Height change (m) at a longitude and latitude (degrees), as seen at a relief spacing (degrees): bilinear
    /// between the nodes, faded out on coarse meshes as the terrain has it.
    double GetDelta(double lon_deg, double lat_deg, double spacing_deg) const;

    /// Number of nodes holding a change.
    size_t GetNumNodes() const;

    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;
    virtual void ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;
    virtual std::uint64_t GetChanges(std::uint64_t since, ChGeoRegion& region) const override;
    virtual bool IsDynamic() const override { return true; }

  private:
    static std::int64_t Key(int i, int j) {
        return (static_cast<std::int64_t>(i) << 32) | static_cast<std::uint32_t>(j);
    }
    // Visibility at a relief spacing (degrees): 1 up to two node spacings, 0 from four.
    double Weight(double spacing_deg) const;
    // Bilinear height change at site coordinates; caller holds the lock.
    double DeltaAt(double x, double y) const;
    // The incoming height at site coordinates with the change applied, at visibility w; caller holds the lock.
    double HeightAt(double x, double y, double w, double height) const;
    // Lon/lat rectangle of a node box, padded one node for the bilinear reach.
    ChGeoRegion RegionOf(int i0, int j0, int i1, int j1) const;

    ChSiteFrame m_site;
    double m_spacing;
    double m_meters_per_degree;

    mutable std::shared_mutex m_mutex;
    struct Value {
        double delta;
        double height;  // after the change, NaN if unknown
    };
    std::unordered_map<std::int64_t, Value> m_deltas;
    double m_flatten_depth = 0.01;
    double m_flatten_strength = 1.0;
    int m_i0, m_j0, m_i1, m_j1;  // bounding box of the nodes ever set, in node indices

    struct Change {
        std::uint64_t version;
        ChGeoRegion region;
    };
    std::deque<Change> m_log;     // recent changes, oldest first
    std::uint64_t m_dropped = 0;  // latest version dropped from the log, 0 if none
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
