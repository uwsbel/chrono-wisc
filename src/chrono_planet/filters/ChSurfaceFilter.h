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
// Surface filters: height transforms applied in order after the samplers, and
// chains of them.
//
// =============================================================================

#ifndef CH_SURFACE_FILTER_H
#define CH_SURFACE_FILTER_H

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "chrono_planet/ChApiPlanet.h"
#include "chrono_planet/ChGeoGrid.h"

namespace chrono {
namespace planet {

/// @addtogroup planet_module
/// @{

/// Base class for surface filters. A filter maps the height computed so far at a point to a new height.
/// A surface runs its samplers, then passes the result through its filter chain. Procedural relief
/// (craters, rocks, roughness) is one kind of filter, see ChReliefLayer; others reshape the terrain.
///
/// Filters must be deterministic and thread-safe, and see only the point, the relief spacing and the
/// incoming height, so that physics and rendering, which evaluate the chain independently, agree.
/// `spacing_deg` is the angular resolution being sampled: filters that add detail should fade out
/// features smaller than a few spacings.
class CH_PLANET_API ChSurfaceFilter {
  public:
    virtual ~ChSurfaceFilter();

    /// New height (m) at a longitude and latitude (degrees) given the incoming height.
    /// The longitude is wrapped to [-180, 180).
    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const = 0;

    /// Apply the filter in place to an n x n row-major grid of heights, at grid.spacing.
    /// The grid never crosses the dateline; its lon0 is wrapped to [-180, 180). The result must match
    /// Apply at every sample to within rounding. The default calls Apply per sample; override it when a
    /// grid pass is cheaper.
    virtual void ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const;

    /// Change tracking, for filters whose output varies at run time (see ChDeformationFilter).
    /// Grow `region` to include everything this filter changed after `since`, and return the version of
    /// its latest change (0 if it never changed). Versions come from NextVersion(), a counter shared by all
    /// filters, so one number tracks a whole chain. Static filters, the default, never change.
    /// Consumers such as ChPlanetQuadtree poll this to rebuild what a change touched.
    virtual std::uint64_t GetChanges(std::uint64_t since, ChGeoRegion& region) const { return 0; }

    /// The next change version, for GetChanges implementations. Thread-safe; never returns 0.
    static std::uint64_t NextVersion();

    /// True for filters whose output changes at run time. A surface evaluates everything before its first
    /// dynamic filter once per tile and caches it, so a change re-runs only the dynamic filter and those
    /// after it. Filters that override GetChanges should return true.
    virtual bool IsDynamic() const { return false; }
};

/// An ordered chain of filters, applied first to last. A chain is itself a filter, so chains nest:
/// a preset chain (for example moon::FilterChain()) can be extended, or appended to another chain.
class CH_PLANET_API ChFilterChain : public ChSurfaceFilter {
  public:
    ChFilterChain() = default;
    explicit ChFilterChain(std::vector<std::shared_ptr<ChSurfaceFilter>> filters);

    /// Append a filter at the end of the chain.
    void AddFilter(std::shared_ptr<ChSurfaceFilter> filter);
    /// Insert a filter before position `index` (0 is the front; GetNumFilters() appends).
    void InsertFilter(size_t index, std::shared_ptr<ChSurfaceFilter> filter);
    /// Remove a filter from this chain (not from nested chains). Returns false if it is not here.
    bool RemoveFilter(const std::shared_ptr<ChSurfaceFilter>& filter);
    /// Remove every filter.
    void Clear() { m_filters.clear(); }

    size_t GetNumFilters() const { return m_filters.size(); }
    const std::vector<std::shared_ptr<ChSurfaceFilter>>& GetFilters() const { return m_filters; }

    /// First filter of the given type, searching nested chains depth-first, or null.
    template <class T>
    std::shared_ptr<T> Find() const {
        for (const auto& f : m_filters) {
            if (auto typed = std::dynamic_pointer_cast<T>(f))
                return typed;
            if (auto chain = std::dynamic_pointer_cast<ChFilterChain>(f))
                if (auto found = chain->Find<T>())
                    return found;
        }
        return nullptr;
    }

    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;
    virtual void ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;
    virtual std::uint64_t GetChanges(std::uint64_t since, ChGeoRegion& region) const override;
    virtual bool IsDynamic() const override;

    /// The filters this chain applies, in order, with nested chains expanded.
    void Flatten(std::vector<const ChSurfaceFilter*>& out) const;

  private:
    std::vector<std::shared_ptr<ChSurfaceFilter>> m_filters;
};

/// Scale and offset: height * scale + offset. Use it for vertical exaggeration or a datum shift.
class CH_PLANET_API ChScaleFilter : public ChSurfaceFilter {
  public:
    explicit ChScaleFilter(double scale, double offset = 0);
    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;

  private:
    double m_scale;
    double m_offset;
};

/// Clamp heights to [min, max], for example to flatten a basin floor or cap peaks.
class CH_PLANET_API ChClampFilter : public ChSurfaceFilter {
  public:
    ChClampFilter(double min_height, double max_height);
    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;

  private:
    double m_min;
    double m_max;
};

/// A filter from a user function (lon_deg, lat_deg, spacing_deg, height) -> height.
/// The function must be deterministic and safe to call from several threads.
class CH_PLANET_API ChFunctionFilter : public ChSurfaceFilter {
  public:
    using Function = std::function<double(double lon_deg, double lat_deg, double spacing_deg, double height)>;
    explicit ChFunctionFilter(Function f);
    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;

  private:
    Function m_function;
};

/// Apply another filter only inside a longitude/latitude rectangle, blending smoothly to the unfiltered
/// height over a feather band outside it. Use it to add site-specific detail, or to flatten a landing pad.
/// The rectangle may cross the dateline (min_lon > max_lon).
class CH_PLANET_API ChRegionFilter : public ChSurfaceFilter {
  public:
    ChRegionFilter(std::shared_ptr<ChSurfaceFilter> filter,
                   double min_lon,
                   double min_lat,
                   double max_lon,
                   double max_lat,
                   double feather_deg = 0);

    /// Weight of the inner filter at a point: 1 inside the rectangle, 0 beyond the feather band.
    double GetWeight(double lon_deg, double lat_deg) const;

    const std::shared_ptr<ChSurfaceFilter>& GetFilter() const { return m_filter; }

    virtual double Apply(double lon_deg, double lat_deg, double spacing_deg, double height) const override;
    virtual void ApplyGrid(const ChGeoGrid& grid, std::vector<double>& heights) const override;
    virtual std::uint64_t GetChanges(std::uint64_t since, ChGeoRegion& region) const override;
    virtual bool IsDynamic() const override { return m_filter->IsDynamic(); }

  private:
    std::shared_ptr<ChSurfaceFilter> m_filter;
    double m_center_lon, m_half_lon, m_min_lat, m_max_lat, m_feather;
};

/// @} planet_module

}  // namespace planet
}  // namespace chrono

#endif
