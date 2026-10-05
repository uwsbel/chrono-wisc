#ifndef CH_PLANET_FIELDGRID_H
#define CH_PLANET_FIELDGRID_H

// Grid walks shared by the procedural fields: the jittered-cell neighborhood of a point, a feature's
// footprint on a grid, and parallel row bands. The grid must not cross the dateline.

#include <algorithm>
#include <cmath>

#include "chrono_planet/ChGeoGrid.h"
#include "chrono_planet/core/FieldHash.h"
#include "chrono_planet/core/Parallel.h"

namespace chrono {
namespace planet {
namespace field {

// Half-open row range: 0 <= begin <= end <= grid.n.
struct RowRange {
    int begin;
    int end;
};

// Calls addRows(RowRange) over bands of an n-row grid, in parallel: one band on the calling thread for
// small grids, otherwise two per thread (for dynamic balance) and at least 16 rows each.
template <class AddRows>
void forEachRowBand(int n, AddRows&& addRows) {
    constexpr int kBandsPerThread = 2, kMinBandRows = 16;
    if (n <= 0) {
        return;
    }
    int rows = n;
    if (util::parallelGrid(static_cast<size_t>(n) * n)) {
        const int bands = kBandsPerThread * util::parallelThreads();
        rows = std::max(kMinBandRows, (n + bands - 1) / bands);
    }
    const int count = (n + rows - 1) / rows;
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(dynamic) if (count > 1)
    for (int b = 0; b < count; ++b) {
        addRows(RowRange{b * rows, std::min(n, (b + 1) * rows)});
    }
}

// Calls visit(cx, cy), rows outer, for the lattice cell holding (x, y) and each neighbor whose near edge is
// within `reach` cells of it. A cell's feature must stay within `reach` of the cell to be found this way.
template <class Visit>
inline void forEachCellNear(double x, double y, double reach, Visit&& visit) {
    const int ix = fastFloor(x), iy = fastFloor(y);
    const double fx = x - ix, fy = y - iy;
    const int cx0 = fx < reach ? ix - 1 : ix, cx1 = fx > 1.0 - reach ? ix + 1 : ix;
    const int cy0 = fy < reach ? iy - 1 : iy, cy1 = fy > 1.0 - reach ? iy + 1 : iy;
    for (int cy = cy0; cy <= cy1; ++cy) {
        for (int cx = cx0; cx <= cx1; ++cx) {
            visit(cx, cy);
        }
    }
}

// Scatters a feature centered at (centerLonDeg, centerLatDeg) onto rows `rows` of a grid. It reaches
// reachLatDeg north and south, and halfWidthDeg(j) east and west on row j (rows where that is not
// positive are skipped). Calls addSpan(j, i0, i1) for the samples i0..i1 of each row it covers.
template <class HalfWidth, class AddSpan>
inline void scatterFootprint(const ChGeoGrid& grid, RowRange rows, double centerLonDeg, double centerLatDeg,
                             double reachLatDeg, HalfWidth&& halfWidthDeg, AddSpan&& addSpan) {
    const double invStepLon = 1.0 / grid.step_lon, invStepLat = 1.0 / grid.step_lat;
    const int j0 = std::max(rows.begin, static_cast<int>(std::ceil((centerLatDeg - reachLatDeg - grid.lat0) * invStepLat)));
    const int j1 = std::min(rows.end - 1, static_cast<int>(std::floor((centerLatDeg + reachLatDeg - grid.lat0) * invStepLat)));
    for (int j = j0; j <= j1; ++j) {
        const double halfLonDeg = halfWidthDeg(j);
        if (!(halfLonDeg > 0.0)) {
            continue;
        }
        const int i0 = std::max(0, static_cast<int>(std::ceil((centerLonDeg - halfLonDeg - grid.lon0) * invStepLon)));
        const int i1 = std::min(grid.n - 1, static_cast<int>(std::floor((centerLonDeg + halfLonDeg - grid.lon0) * invStepLon)));
        addSpan(j, i0, i1);
    }
}

}  // namespace field
}  // namespace planet
}  // namespace chrono

#endif  // CH_PLANET_FIELDGRID_H
