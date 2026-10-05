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

#include "chrono_planet/dem/ChGeoTiffStack.h"
#include "chrono_planet/dem/GeoTIFFLoader.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "chrono_planet/core/MathUtil.h"
#include "chrono_planet/core/Parallel.h"
#include "chrono_planet/core/SphereMath.h"

namespace chrono {
namespace planet {

namespace {

// Overlay blend ramp on the ground, and its bounds in pixels: the B-spline support at least, one byte at most.
constexpr double kFeatherM = 500.0;
constexpr double kMinFeatherPx = 4.0, kMaxFeatherPx = 255.0;
// Chamfer distance transform steps in thirds of a pixel, and the value of a pixel it has not reached.
constexpr int kChamferAxial = 3, kChamferDiagonal = 4, kChamferUnreached = 100000;
// The B-spline footprint is 4x4, so a sample this far inside valid data is fully trusted.
constexpr double kFullSupportPx = 2.0;
// Geotransforms with a smaller determinant are degenerate.
constexpr double kMinGeoTransformDet = 1e-12;
// Raster axis skew below this counts as axis-aligned.
constexpr double kMaxAxisSkew = 1e-12;
// Accept cross-axis projection drift below one millionth of the sampled extent (dataset units).
constexpr double kProjectionRelativeTolerance = 1e-6;
// Cubic reconstruction samples floor(pixel) + {-1, 0, 1, 2} along each axis.
constexpr int kSplineTaps = 4;

struct RasterCoordinates {
    std::vector<double> columns;
    std::vector<double> rows;
};

// Prepared samples for one axis. Arrays stay separate for the interpolation loops.
struct AxisSamples {
    std::vector<std::array<int, kSplineTaps>> pixels;
    std::vector<std::array<double, kSplineTaps>> weights;
    std::vector<unsigned char> valid;
    int firstPixel = INT_MAX;
    int lastPixel = INT_MIN;

    bool hasCoverage() const { return firstPixel <= lastPixel; }
};

// Cubic B-spline weights of the four taps around a sample at fractional offset t in [0, 1).
std::array<double, kSplineTaps> bsplineWeights(double t) {
    const double t2 = t * t, t3 = t2 * t;
    return {(1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0, (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0,
            (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0, t3 / 6.0};
}


// Prepare both axes identically: locate the sample, calculate its four weights, and clamp its taps.
AxisSamples prepareAxisSamples(const std::vector<double>& coordinates, int rasterSize) {
    AxisSamples samples;
    samples.pixels.resize(coordinates.size());
    samples.weights.resize(coordinates.size());
    samples.valid.resize(coordinates.size(), 0);
    for (size_t i = 0; i < coordinates.size(); ++i) {
        const double pixel = coordinates[i];
        if (!(pixel >= 0.0) || pixel >= rasterSize - 1)
            continue;
        const int basePixel = static_cast<int>(std::floor(pixel));
        samples.valid[i] = 1;
        samples.weights[i] = bsplineWeights(pixel - basePixel);
        for (int tap = 0; tap < kSplineTaps; ++tap)
            samples.pixels[i][tap] = std::clamp(basePixel - 1 + tap, 0, rasterSize - 1);
        samples.firstPixel = std::min(samples.firstPixel, samples.pixels[i].front());
        samples.lastPixel = std::max(samples.lastPixel, samples.pixels[i].back());
    }
    return samples;
}

// Raster (col, row) of a dataset-space point.
struct GeoTransformInverse {
    std::array<double, 6> gt;
    double det;
    explicit GeoTransformInverse(const std::array<double, 6>& g) : gt(g), det(g[1] * g[5] - g[2] * g[4]) {}
    bool valid() const { return std::abs(det) >= kMinGeoTransformDet; }
    double col(double x, double y) const { return ((x - gt[0]) * gt[5] - (y - gt[3]) * gt[2]) / det; }
    double row(double x, double y) const { return (-(x - gt[0]) * gt[4] + (y - gt[3]) * gt[1]) / det; }
};

// GDAL's errors kept off the console while in scope: a sample outside a projection's domain is expected.
struct QuietErrors {
    QuietErrors() { CPLPushErrorHandler(CPLQuietErrorHandler); }
    ~QuietErrors() { CPLPopErrorHandler(); }
    QuietErrors(const QuietErrors&) = delete;
    QuietErrors& operator=(const QuietErrors&) = delete;
};

// Reports once a geographic source whose raster axes are not aligned with lon/lat. It is treated as not covering.
void warnNonseparableRaster() {
    static std::once_flag warned;
    std::call_once(warned, [] {
        std::cerr << "[ChGeoTiffStack] a geographic raster is rotated against lon/lat; it is not read\n";
    });
}

}   // namespace

struct ChGeoTiffStack::Impl {
    // Sets up the body's lon/lat SRS that projected sources are transformed to.
    // Throws std::runtime_error if the body's SRS string does not parse.
    explicit Impl(const ChPlanetBody& body);

    // Loads a raster for zooms [min_zoom, max_zoom]. Throws std::runtime_error on failure.
    void addSource(const ChGeoTiffSource& source);

    // OGR transforms must be released through GDAL, not delete.
    struct TransformDeleter {
        void operator()(OGRCoordinateTransformation* transform) const { OGRCoordinateTransformation::DestroyCT(transform); }
    };
    using TransformPtr = std::unique_ptr<OGRCoordinateTransformation, TransformDeleter>;

    // Where an overlay's data can be trusted, as a per-pixel distance to nodata or the raster edge.
    struct ValidityMask {
        std::vector<unsigned char> distancePx;   // whole pixels, saturating at 255, empty for a complete source
        float featherPx = 0.f;                   // blend ramp width in pixels
        bool complete() const { return distancePx.empty(); }
    };

    // One registered raster with its footprint, zoom range and validity mask.
    struct Source {
        std::shared_ptr<GeoTIFFLoader> loader;
        int minZoom, maxZoom;
        double minLon, maxLon, minLat, maxLat;   // degrees
        bool isGeographic;
        TransformPtr toDataset;   // lon/lat -> ds, projected sources only
        ValidityMask mask;        // complete for the base source
    };

    // Finest source valid at zoom over a rect and the coarser fallback under it. Either may be null.
    struct SelectedSources {
        const Source* fine = nullptr;
        const Source* base = nullptr;
    };
    SelectedSources selectSources(double minLon, double maxLon, double minLat, double maxLat, int zoom) const;
    // Does a lon/lat rect meet a source's footprint, at any wrap of longitude.
    [[nodiscard]] static bool meets(const Source& source, double minLon, double maxLon, double minLat, double maxLat);
    // Seamless compositing: the base the zoom picks, then every overlay the rect meets, in the order added.
    void seamlessGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const;
    bool seamless_ = false;
    // Validity mask of a raster, feathered over a fixed ground distance. edgesInvalid fades the
    // raster's own border too, which overlays need and a global base must not have.
    [[nodiscard]] static ValidityMask validityMaskOf(const GeoTIFFLoader& raster, double pixelMetres, double minValid, bool edgesInvalid,
                                                     double featherM);
    // Blend weight in [0, 1] from distance to an edge or nodata hole at a fractional pixel.
    [[nodiscard]] static double edgeBlendWeight(const Source& source, double pixelColumn, double pixelRow);
    // A successfully mapped raster. A zero weight means no usable data at that sample;
    // its elevation is a placeholder and must not contribute to blending.
    struct RasterSamples {
        std::vector<double> elevations;
        std::vector<double> blendWeights;
    };
    // nullopt means the coordinate transform failed or cannot be sampled as separate axes.
    // A successful result can still have no coverage (all blend weights zero).
    std::optional<RasterSamples> reconstructGrid(const Source& source, const ChGeoGrid& grid) const;
    // A projected raster whose axes do not run along lon/lat over the grid (polar stereographic, say): every sample
    // transformed and interpolated on its own. The same spline and blend weights as the two-pass path.
    RasterSamples reconstructPointwise(const Source& source, const ChGeoGrid& grid) const;
    std::optional<RasterCoordinates> mapToRaster(const Source& source, const ChGeoGrid& grid) const;
    RasterSamples interpolateHeights(const Source& source, const RasterCoordinates& coordinates,
                                  const AxisSamples& columns, const AxisSamples& rows) const;

    std::vector<Source> sources_;
    double radiusM_;
    OGRSpatialReference bodyLonLat_;
    mutable std::mutex transformMutex_;   // OGR transforms are not thread-safe
};

ChGeoTiffStack::ChGeoTiffStack(const ChPlanetBody& body)
    : m_body(body), m_impl(std::make_unique<Impl>(body)) {}

ChGeoTiffStack::~ChGeoTiffStack() = default;

void ChGeoTiffStack::AddSource(const ChGeoTiffSource& source) {
    m_impl->addSource(source);
}

int ChGeoTiffStack::GetNumSources() const {
    return static_cast<int>(m_impl->sources_.size());
}

void ChGeoTiffStack::SetSeamless(bool seamless) {
    m_impl->seamless_ = seamless;
}

bool ChGeoTiffStack::IsSeamless() const {
    return m_impl->seamless_;
}

bool ChGeoTiffStack::Impl::meets(const Source& source, double minLon, double maxLon, double minLat, double maxLat) {
    if (maxLat < source.minLat || minLat > source.maxLat) {
        return false;
    }
    for (double shift : {0.0, 360.0, -360.0}) {
        if (!(maxLon + shift < source.minLon || minLon + shift > source.maxLon)) {
            return true;
        }
    }
    return false;
}

void ChGeoTiffStack::Impl::seamlessGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const {
    const auto [lon0, lat0, stepLon, stepLat, side, spacing] = grid;
    const size_t n = static_cast<size_t>(side) * side;
    const double lon1 = lon0 + (side - 1) * stepLon, lat1 = lat0 + (side - 1) * stepLat;
    const double minLon = std::min(lon0, lon1), maxLon = std::max(lon0, lon1), minLat = std::min(lat0, lat1), maxLat = std::max(lat0, lat1);

    // The base: of the whole-body rasters, the narrowest whose zooms hold this one, else the finest.
    const Source* base = nullptr;
    int smallestZoomRange = std::numeric_limits<int>::max(), finestBaseZoom = std::numeric_limits<int>::min();
    bool holdsZoom = false;
    for (const auto& source : sources_) {
        if (source.minZoom != 0 || !meets(source, minLon, maxLon, minLat, maxLat)) {
            continue;
        }
        if (zoom <= source.maxZoom) {
            if (!holdsZoom || source.maxZoom - source.minZoom < smallestZoomRange) {
                smallestZoomRange = source.maxZoom - source.minZoom;
                base = &source;
                holdsZoom = true;
            }
        } else if (!holdsZoom && source.maxZoom > finestBaseZoom) {
            finestBaseZoom = source.maxZoom;
            base = &source;
        }
    }
    if (base) {
        if (const auto under = reconstructGrid(*base, grid)) {
            for (size_t i = 0; i < n; ++i) {
                if (under->blendWeights[i] > 0.0) {
                    out[i] = under->elevations[i];
                }
            }
        }
    }
    // Each overlay over what is under it, by its own weight: 1 well inside it, 0 at its edges and holes.
    for (const auto& source : sources_) {
        if (source.minZoom == 0 || !meets(source, minLon, maxLon, minLat, maxLat)) {
            continue;
        }
        const auto over = reconstructGrid(source, grid);
        if (!over) {
            continue;
        }
        for (size_t i = 0; i < n; ++i) {
            const double weight = over->blendWeights[i];
            if (weight > 0.0) {
                out[i] = std::isnan(out[i]) ? over->elevations[i] : out[i] * (1.0 - weight) + over->elevations[i] * weight;
            }
        }
    }
}

void ChGeoTiffStack::GetHeightGrid(const ChGeoGrid& grid, int zoom, std::vector<double>& out) const {
    const auto [lon0, lat0, stepLon, stepLat, side, spacing] = grid;
    if (side <= 0) {
        throw std::invalid_argument("ChGeoTiffStack::GetHeightGrid: n must be positive");
    }
    const size_t n = static_cast<size_t>(side) * side;
    out.assign(n, std::numeric_limits<double>::quiet_NaN());
    if (m_impl->seamless_) {
        m_impl->seamlessGrid(grid, zoom, out);
        return;
    }

    // Select the active raster and its coarser base, then reconstruct both on the same grid.
    const double lon1 = lon0 + (side - 1) * stepLon, lat1 = lat0 + (side - 1) * stepLat;
    const auto [fine, base] = m_impl->selectSources(std::min(lon0, lon1), std::max(lon0, lon1), std::min(lat0, lat1), std::max(lat0, lat1), zoom);
    const auto baseGrid = base ? m_impl->reconstructGrid(*base, grid) : std::nullopt;
    if (base && !baseGrid) {
        // Preserve the stack policy: failure to map the selected base invalidates the whole query.
        return;
    }
    const auto fineGrid = fine ? m_impl->reconstructGrid(*fine, grid) : std::nullopt;
    // The fine source fades to the base where both have data, and stands alone where only it does.
    // Even if the fine source failed, the base grid keeps its own holes.
    for (size_t i = 0; i < n; ++i) {
        const double baseWeight = baseGrid ? baseGrid->blendWeights[i] : 0.0;
        const double fineWeight = fineGrid ? fineGrid->blendWeights[i] : 0.0;
        if (fineWeight > 0.0) {
            const double weight = baseWeight > 0.0 ? fineWeight : 1.0;
            const double under = baseGrid ? baseGrid->elevations[i] : 0.0;
            out[i] = under * (1.0 - weight) + fineGrid->elevations[i] * weight;
        } else if (baseWeight > 0.0) {
            out[i] = baseGrid->elevations[i];
        }
    }
}

std::optional<double> ChGeoTiffStack::GetHeight(double lonDeg, double latDeg, int zoom) const {
    std::vector<double> h;
    GetHeightGrid(ChGeoGrid{lonDeg, latDeg, 1e-7, 1e-7, 1, 0}, zoom, h);
    if (std::isnan(h[0])) {
        return std::nullopt;
    }
    return h[0];
}

// GDAL resource setup and raster reconstruction.

ChGeoTiffStack::Impl::Impl(const ChPlanetBody& body) : radiusM_(body.GetRadius()) {
    if (body.GetGeographicSRS().empty()) {
        const std::string& name = body.GetName();
        bodyLonLat_.SetGeogCS(("GCS_" + name).c_str(), ("D_" + name).c_str(), (name + "_Sphere").c_str(), radiusM_, 0.0);
    } else {
        OGRSpatialReference user;
        if (user.SetFromUserInput(body.GetGeographicSRS().c_str()) != OGRERR_NONE) {
            throw std::runtime_error("cannot parse the geographic SRS of body '" + body.GetName() +
                                     "': " + body.GetGeographicSRS());
        }
        if (user.IsGeographic()) {
            bodyLonLat_ = user;
        } else {
            std::unique_ptr<OGRSpatialReference> geog(user.CloneGeogCS());
            if (!geog) {
                throw std::runtime_error("the SRS of body '" + body.GetName() + "' has no geographic base");
            }
            bodyLonLat_ = *geog;
        }
    }
    bodyLonLat_.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
}

void ChGeoTiffStack::Impl::addSource(const ChGeoTiffSource& opt) {
    const std::string& file = opt.path;
    if (opt.min_zoom > opt.max_zoom) {
        throw std::runtime_error("min_zoom exceeds max_zoom for " + file);
    }
    auto ldr = std::make_shared<GeoTIFFLoader>();
    ldr->load(file, opt.band);   // throws with the reason
    if (opt.nodata) {
        ldr->setNodata(*opt.nodata);
    }

    // Convert stored values to heights above the reference sphere. Nodata pixels keep their sentinel.
    const double bandScale = opt.use_band_scale_offset ? ldr->bandScale() : 1.0;
    const double bandOffset = opt.use_band_scale_offset ? ldr->bandOffset() : 0.0;
    const bool identity = bandScale == 1.0 && bandOffset == 0.0 && opt.scale == 1.0 && opt.offset == 0.0 && !opt.values_are_radii;
    if (!identity) {
        const bool hasNodata = ldr->hasNodata();
        const double nodata = ldr->nodata();
        const double radiusShift = opt.values_are_radii ? radiusM_ : 0.0;
        for (double& v : ldr->elevation()) {
            if (!std::isfinite(v) || (hasNodata && v == nodata)) {
                continue;
            }
            v = (v * bandScale + bandOffset) * opt.scale + opt.offset - radiusShift;
        }
    }

    const bool dsIsGeo = ldr->isGeographic();
    TransformPtr toDataset, toLonLat;
    if (!dsIsGeo) {
        const OGRSpatialReference& dsSRS = ldr->srs();
        toDataset.reset(OGRCreateCoordinateTransformation(&bodyLonLat_, &dsSRS));
        toLonLat.reset(OGRCreateCoordinateTransformation(&dsSRS, &bodyLonLat_));
        if (!toDataset || !toLonLat) {
            throw std::runtime_error("cannot transform between the body's geographic SRS and the SRS of " + file);
        }
    }
    // A datum far from the body's radius almost always means data from another body, or heights in the wrong units.
    if (ldr->hasSrs()) {
        OGRErr err = OGRERR_NONE;
        const double semiMajor = ldr->srs().GetSemiMajor(&err);
        if (err == OGRERR_NONE && std::abs(semiMajor - radiusM_) > 0.01 * radiusM_) {
            std::cerr << "[ChGeoTiffStack] warning: " << file << " is on a datum of radius " << semiMajor
                      << " m, but the body radius is " << radiusM_ << " m\n";
        }
    }

    // Lon/lat extent from the raster corners.
    const auto& gt = ldr->geoTransform();
    const double w = ldr->width() - 1, h = ldr->height() - 1;
    double lon[4] = {gt[0], gt[0] + w * gt[1], gt[0] + h * gt[2], gt[0] + w * gt[1] + h * gt[2]};
    double lat[4] = {gt[3], gt[3] + w * gt[4], gt[3] + h * gt[5], gt[3] + w * gt[4] + h * gt[5]};
    if (toLonLat) {
        toLonLat->Transform(4, lon, lat);
    }

    Source src{ldr, opt.min_zoom, opt.max_zoom,
               *std::min_element(lon, lon + 4), *std::max_element(lon, lon + 4),
               *std::min_element(lat, lat + 4), *std::max_element(lat, lat + 4),
               dsIsGeo, std::move(toDataset)};
    // Overlays fade to what is under them at their edges and holes. A base source is used to its edges,
    // and gets a mask only if it has holes, so they read as no data instead of the nodata sentinel.
    const double pixelMetres = std::abs(gt[1]) * (dsIsGeo ? util::metresPerDegLat(radiusM_) : 1.0);
    if (opt.min_zoom > 0) {
        src.mask = validityMaskOf(*ldr, pixelMetres, opt.min_valid, true, opt.feather_m > 0 ? opt.feather_m : kFeatherM);
    } else {
        const bool hasNodata = ldr->hasNodata();
        const double nodata = ldr->nodata();
        const auto& d = ldr->elevation();
        const bool holes = std::any_of(d.begin(), d.end(), [&](double v) {
            return !std::isfinite(v) || (hasNodata && v == nodata) || v < opt.min_valid;
        });
        if (holes) {
            src.mask = validityMaskOf(*ldr, pixelMetres, opt.min_valid, false, kFeatherM);
        }
    }
    sources_.push_back(std::move(src));

    std::cout << "[ChGeoTiffStack] added source: " << file
              << (ldr->hasNodata() ? "  (nodata " + std::to_string(ldr->nodata()) + ")" : "") << '\n';
}

// Chamfer distance transform to the nearest invalid pixel or raster edge, in whole pixels.
ChGeoTiffStack::Impl::ValidityMask ChGeoTiffStack::Impl::validityMaskOf(const GeoTIFFLoader& raster, double pixelMetres,
                                                                      double minValid, bool edgesInvalid, double featherM) {
    const int width = raster.width(), height = raster.height();
    const auto& rasterHeights = raster.elevation();
    const bool hasNodata = raster.hasNodata();
    const double nodata = raster.nodata();
    ValidityMask mask;
    mask.featherPx = static_cast<float>(std::clamp(featherM / std::max(pixelMetres, 1e-3), kMinFeatherPx, kMaxFeatherPx));

    std::vector<int> distanceThirdPixels(static_cast<size_t>(width) * height);   // thirds of a pixel
    auto distanceAt = [&](int x, int y) -> int& { return distanceThirdPixels[static_cast<size_t>(y) * width + x]; };
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const double heightValue = rasterHeights[static_cast<size_t>(y) * width + x];
            const bool invalid = !std::isfinite(heightValue) || (hasNodata && heightValue == nodata) || heightValue < minValid;
            const bool edge = edgesInvalid && (x == 0 || y == 0 || x == width - 1 || y == height - 1);
            if (invalid)
                distanceAt(x, y) = 0;
            else if (edge)
                distanceAt(x, y) = kChamferAxial;
            else
                distanceAt(x, y) = kChamferUnreached;
        }
    }
    // Forward sweep propagates distances from the left and preceding row.
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int& distance = distanceAt(x, y);
            if (x > 0) {
                distance = std::min(distance, distanceAt(x - 1, y) + kChamferAxial);
            }
            if (y > 0) {
                distance = std::min(distance, distanceAt(x, y - 1) + kChamferAxial);
            }
            if (x > 0 && y > 0) {
                distance = std::min(distance, distanceAt(x - 1, y - 1) + kChamferDiagonal);
            }
            if (x + 1 < width && y > 0) {
                distance = std::min(distance, distanceAt(x + 1, y - 1) + kChamferDiagonal);
            }
        }
    }
    // Reverse sweep includes the right and following row, completing the distance estimate.
    for (int y = height - 1; y >= 0; --y) {
        for (int x = width - 1; x >= 0; --x) {
            int& distance = distanceAt(x, y);
            if (x + 1 < width) {
                distance = std::min(distance, distanceAt(x + 1, y) + kChamferAxial);
            }
            if (y + 1 < height) {
                distance = std::min(distance, distanceAt(x, y + 1) + kChamferAxial);
            }
            if (x + 1 < width && y + 1 < height) {
                distance = std::min(distance, distanceAt(x + 1, y + 1) + kChamferDiagonal);
            }
            if (x > 0 && y + 1 < height) {
                distance = std::min(distance, distanceAt(x - 1, y + 1) + kChamferDiagonal);
            }
        }
    }
    mask.distancePx.resize(distanceThirdPixels.size());
    size_t invalidCount = 0;
    for (size_t i = 0; i < distanceThirdPixels.size(); ++i) {
        mask.distancePx[i] = static_cast<unsigned char>(std::min(255, distanceThirdPixels[i] / kChamferAxial));
        if (distanceThirdPixels[i] == 0) {
            ++invalidCount;
        }
    }
    std::cout << "[ChGeoTiffStack]   validity mask: " << width << "x" << height << ", " << (100.0 * invalidCount / distanceThirdPixels.size())
              << "% nodata, feather " << mask.featherPx << " px\n";
    return mask;
}

double ChGeoTiffStack::Impl::edgeBlendWeight(const Source& source, double pixelColumn, double pixelRow) {
    if (source.mask.complete()) {
        return 1.0;
    }
    const int width = source.loader->width(), height = source.loader->height();
    if (!(pixelColumn >= 0.0) || !(pixelRow >= 0.0) || pixelColumn >= width - 1 || pixelRow >= height - 1) {
        return 0.0;
    }
    const int c0 = static_cast<int>(pixelColumn), r0 = static_cast<int>(pixelRow);
    const double tx = pixelColumn - c0, ty = pixelRow - r0;
    auto maskDistance = [&](int c, int r) { return static_cast<double>(source.mask.distancePx[static_cast<size_t>(r) * width + c]); };
    const double distancePixels = (1 - tx) * (1 - ty) * maskDistance(c0, r0) + tx * (1 - ty) * maskDistance(c0 + 1, r0) +
                                  (1 - tx) * ty * maskDistance(c0, r0 + 1) + tx * ty * maskDistance(c0 + 1, r0 + 1);
    // Exclude the two-pixel spline support near holes, then smoothly ramp to full trust.
    // smoothstep01(t) = t*t*(3 - 2*t): zero slope at both ends of the blend.
    const double blendFraction = std::clamp((distancePixels - kFullSupportPx) / std::max(static_cast<double>(source.mask.featherPx) - kFullSupportPx, 1.0), 0.0, 1.0);
    return util::smoothstep01(blendFraction);
}

ChGeoTiffStack::Impl::SelectedSources ChGeoTiffStack::Impl::selectSources(double minLon, double maxLon, double minLat, double maxLat, int zoom) const {
    SelectedSources selected;
    auto& [fine, base] = selected;
    int smallestZoomRange = std::numeric_limits<int>::max(), finestBaseZoom = std::numeric_limits<int>::min();
    for (const auto& source : sources_) {
        bool intersectsGrid = false, coversGrid = false;
        if (!(maxLat < source.minLat || minLat > source.maxLat)) {
            for (double shift : {0.0, 360.0, -360.0}) {
                const double shiftedMinLon = minLon + shift, shiftedMaxLon = maxLon + shift;
                if (!(shiftedMaxLon < source.minLon || shiftedMinLon > source.maxLon)) {
                    intersectsGrid = true;
                }
                if (shiftedMinLon >= source.minLon && shiftedMaxLon <= source.maxLon && minLat >= source.minLat && maxLat <= source.maxLat) {
                    coversGrid = true;
                }
            }
        }
        if (zoom >= source.minZoom && zoom <= source.maxZoom) {
            if (intersectsGrid && source.maxZoom - source.minZoom < smallestZoomRange) {
                smallestZoomRange = source.maxZoom - source.minZoom;
                fine = &source;
            }
        } else if (source.maxZoom < zoom && coversGrid && source.maxZoom > finestBaseZoom) {
            finestBaseZoom = source.maxZoom;
            base = &source;
        }
    }
    // A complete fine source has nothing to fade to, so treat it as base and sample once.
    if (fine && fine->mask.complete()) {
        base = fine;
        fine = nullptr;
    }
    return selected;
}

// Map geographic coordinates, prepare each axis, then interpolate the raster in two passes.
std::optional<ChGeoTiffStack::Impl::RasterSamples> ChGeoTiffStack::Impl::reconstructGrid(const Source& source, const ChGeoGrid& grid) const {
    const auto coordinates = mapToRaster(source, grid);
    if (!coordinates) {
        if (source.isGeographic)
            return std::nullopt;
        return reconstructPointwise(source, grid);
    }
    const auto columns = prepareAxisSamples(coordinates->columns, source.loader->width());
    const auto rows = prepareAxisSamples(coordinates->rows, source.loader->height());
    return interpolateHeights(source, *coordinates, columns, rows);
}

std::optional<RasterCoordinates> ChGeoTiffStack::Impl::mapToRaster(const Source& source, const ChGeoGrid& grid) const {
    const auto [lon0, lat0, stepLon, stepLat, side, spacing] = grid;
    const GeoTransformInverse inverse(source.loader->geoTransform());
    if (!inverse.valid()) {
        return std::nullopt;
    }

    double longitudeShift = 0.0;
    if (lon0 < source.minLon) {
        longitudeShift = 360.0;
    } else if (lon0 > source.maxLon) {
        longitudeShift = -360.0;
    }
    std::vector<double> longitudes(side), latitudes(side);
    for (int i = 0; i < side; ++i) {
        longitudes[i] = lon0 + longitudeShift + i * stepLon;
    }
    for (int j = 0; j < side; ++j) {
        latitudes[j] = lat0 + j * stepLat;
    }

    // Raster x of every grid column, raster y of every grid row.
    std::vector<double> pixelColumns(side), pixelRows(side);
    if (source.isGeographic) {
        const auto& gt = inverse.gt;
        if (std::abs(gt[2]) > kMaxAxisSkew || std::abs(gt[4]) > kMaxAxisSkew) {
            warnNonseparableRaster();
            return std::nullopt;   // rotated raster
        }
        for (int i = 0; i < side; ++i) {
            pixelColumns[i] = inverse.col(longitudes[i], latitudes[0]);
        }
        for (int j = 0; j < side; ++j) {
            pixelRows[j] = inverse.row(longitudes[0], latitudes[j]);
        }
    } else {
        // Transform the first row and column, then check the far corners agree.
        std::vector<double> firstRowX(longitudes), firstRowY(side, latitudes[0]);
        std::vector<double> firstColumnX(side, longitudes[0]), firstColumnY(latitudes);
        double cornerX[2] = {longitudes[side - 1], longitudes[0]}, cornerY[2] = {latitudes[side - 1], latitudes[side - 1]};
        {
            // A point outside the projection's domain is no error here: the samples are then taken one by one
            std::lock_guard<std::mutex> lock(transformMutex_);
            const QuietErrors quiet;
            if (!source.toDataset->Transform(side, firstRowX.data(), firstRowY.data())) {
                return std::nullopt;
            }
            if (!source.toDataset->Transform(side, firstColumnX.data(), firstColumnY.data())) {
                return std::nullopt;
            }
            if (!source.toDataset->Transform(2, cornerX, cornerY)) {
                return std::nullopt;
            }
        }
        const double tolerance = kProjectionRelativeTolerance * (std::abs(firstRowX[side - 1] - firstRowX[0]) + std::abs(firstColumnY[side - 1] - firstColumnY[0]) + 1.0);
        const bool separable = std::abs(cornerX[0] - firstRowX[side - 1]) < tolerance && std::abs(cornerY[0] - firstColumnY[side - 1]) < tolerance &&
                               std::abs(cornerX[1] - firstRowX[0]) < tolerance && std::abs(cornerY[1] - firstColumnY[side - 1]) < tolerance;
        if (!separable) {
            return std::nullopt;   // sampled point by point instead
        }
        for (int i = 0; i < side; ++i) {
            pixelColumns[i] = inverse.col(firstRowX[i], firstRowY[0]);
        }
        for (int j = 0; j < side; ++j) {
            pixelRows[j] = inverse.row(firstColumnX[0], firstColumnY[j]);
        }
    }

    return RasterCoordinates{std::move(pixelColumns), std::move(pixelRows)};
}

ChGeoTiffStack::Impl::RasterSamples ChGeoTiffStack::Impl::reconstructPointwise(const Source& source, const ChGeoGrid& grid) const {
    const auto [lon0, lat0, stepLon, stepLat, side, spacing] = grid;
    const size_t count = static_cast<size_t>(side) * side;
    RasterSamples result{std::vector<double>(count, 0.0), std::vector<double>(count, 0.0)};
    const GeoTransformInverse inverse(source.loader->geoTransform());
    if (!inverse.valid()) {
        return result;
    }
    // Every sample into the raster's projection. One past a pole is in no projection's domain: left without data.
    std::vector<double> x(count), y(count);
    std::vector<int> transformed(count, 0);
    for (int j = 0; j < side; ++j) {
        for (int i = 0; i < side; ++i) {
            const size_t k = static_cast<size_t>(j) * side + i;
            x[k] = lon0 + i * stepLon;
            y[k] = std::clamp(lat0 + j * stepLat, -90.0, 90.0);
        }
    }
    {
        std::lock_guard<std::mutex> lock(transformMutex_);
        const QuietErrors quiet;
        source.toDataset->Transform(static_cast<int>(count), x.data(), y.data(), nullptr, transformed.data());
    }
    const int width = source.loader->width(), height = source.loader->height();
    const auto& rasterHeights = source.loader->elevation();
    const bool parallel = util::parallelGrid(count);
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(static) if (parallel)
    for (int j = 0; j < side; ++j) {
        const double latitude = lat0 + j * stepLat;
        if (latitude < -90.0 || latitude > 90.0)
            continue;
        for (int i = 0; i < side; ++i) {
            const size_t k = static_cast<size_t>(j) * side + i;
            if (!transformed[k] || !std::isfinite(x[k]) || !std::isfinite(y[k]))
                continue;
            const double column = inverse.col(x[k], y[k]), row = inverse.row(x[k], y[k]);
            if (!(column >= 0.0) || column >= width - 1 || !(row >= 0.0) || row >= height - 1)
                continue;
            const double weight = edgeBlendWeight(source, column, row);
            if (!(weight > 0.0))
                continue;
            const int baseColumn = static_cast<int>(std::floor(column)), baseRow = static_cast<int>(std::floor(row));
            const auto columnWeights = bsplineWeights(column - baseColumn), rowWeights = bsplineWeights(row - baseRow);
            double sum = 0.0;
            for (int rowTap = 0; rowTap < kSplineTaps; ++rowTap) {
                const double* rasterRow = &rasterHeights[static_cast<size_t>(std::clamp(baseRow - 1 + rowTap, 0, height - 1)) * width];
                double along = 0.0;
                for (int columnTap = 0; columnTap < kSplineTaps; ++columnTap)
                    along += columnWeights[columnTap] * rasterRow[std::clamp(baseColumn - 1 + columnTap, 0, width - 1)];
                sum += rowWeights[rowTap] * along;
            }
            result.elevations[k] = sum;
            result.blendWeights[k] = weight;
        }
    }
    return result;
}

ChGeoTiffStack::Impl::RasterSamples ChGeoTiffStack::Impl::interpolateHeights(
    const Source& source, const RasterCoordinates& coordinates, const AxisSamples& columns, const AxisSamples& rows) const {
    const int side = static_cast<int>(coordinates.columns.size());
    const size_t count = static_cast<size_t>(side) * side;
    RasterSamples result{std::vector<double>(count, 0.0), std::vector<double>(count, 0.0)};
    if (!columns.hasCoverage() || !rows.hasCoverage())
        return result;

    // Horizontal pass: reuse each interpolated raster row across all output rows that need it.
    const int rasterWidth = source.loader->width();
    const auto& rasterHeights = source.loader->elevation();
    const int rasterRows = rows.lastPixel - rows.firstPixel + 1;
    std::vector<double> rowInterpolatedHeights(static_cast<size_t>(rasterRows) * side);
    const bool parallelRows = util::parallelGrid(rowInterpolatedHeights.size());
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(static) if (parallelRows)
    for (int row = 0; row < rasterRows; ++row) {
        const double* rasterRow = &rasterHeights[static_cast<size_t>(rows.firstPixel + row) * rasterWidth];
        double* outputRow = &rowInterpolatedHeights[static_cast<size_t>(row) * side];
        for (int column = 0; column < side; ++column) {
            double height = 0.0;
            for (int tap = 0; tap < kSplineTaps; ++tap)
                height += columns.weights[column][tap] * rasterRow[columns.pixels[column][tap]];
            outputRow[column] = columns.valid[column] ? height : 0.0;
        }
    }

    // Vertical pass: combine four intermediate rows, then discard samples unsupported by valid data.
    const bool parallelColumns = util::parallelGrid(count);
#pragma omp parallel for num_threads(util::parallelThreads()) schedule(static) if (parallelColumns)
    for (int row = 0; row < side; ++row) {
        if (!rows.valid[row])
            continue;
        const auto& weights = rows.weights[row];
        std::array<const double*, kSplineTaps> sampledRows;
        for (int tap = 0; tap < kSplineTaps; ++tap)
            sampledRows[tap] = &rowInterpolatedHeights[static_cast<size_t>(rows.pixels[row][tap] - rows.firstPixel) * side];
        double* outputHeights = &result.elevations[static_cast<size_t>(row) * side];
        double* blendWeights = &result.blendWeights[static_cast<size_t>(row) * side];
        // Keep validity separate from interpolation so the arithmetic loop can vectorize.
        for (int column = 0; column < side; ++column) {
            if (columns.valid[column])
                blendWeights[column] = edgeBlendWeight(source, coordinates.columns[column], coordinates.rows[row]);
        }
        for (int column = 0; column < side; ++column) {
            const double height = weights[0] * sampledRows[0][column] + weights[1] * sampledRows[1][column] +
                                  weights[2] * sampledRows[2][column] + weights[3] * sampledRows[3][column];
            outputHeights[column] = blendWeights[column] > 0.0 ? height : 0.0;
        }
    }
    return result;
}

}  // namespace planet
}  // namespace chrono
