#include "chrono_planet/core/SphereMath.h"

#include <algorithm>
#include <cmath>

namespace chrono {
namespace planet {

namespace util {

double wrapLongitude(double lonDeg) {
    double shifted = std::fmod(lonDeg + 180.0, 360.0);
    if (shifted < 0.0) {
        shifted += 360.0;
    }
    return shifted - 180.0;
}

double cosLatClamped(double latDeg) { return std::max(kMinCosLat, std::cos(deg2rad(latDeg))); }

double metresPerDegLon(double radiusM, double latDeg) { return metresPerDegLat(radiusM) * std::cos(deg2rad(latDeg)); }

LonLat lonLatOf(double x, double y, double z) {
    const double r = radiusOf(x, y, z);
    const double s = r > 0.0 ? std::clamp(z / r, -1.0, 1.0) : 0.0;
    return {rad2deg(std::atan2(y, x)), rad2deg(std::asin(s))};
}

Vec3 pointOnSphere(double lonDeg, double latDeg, double radius) {
    const double lam = deg2rad(lonDeg), phi = deg2rad(latDeg);
    const double rcp = radius * std::cos(phi);
    return {rcp * std::cos(lam), rcp * std::sin(lam), radius * std::sin(phi)};
}

EnuFrame enuAt(double lonDeg, double latDeg) {
    const double lam = deg2rad(lonDeg), phi = deg2rad(latDeg);
    const double sl = std::sin(lam), cl = std::cos(lam);
    const double sp = std::sin(phi), cp = std::cos(phi);
    return {{-sl, cl, 0.0}, {-sp * cl, -sp * sl, cp}, {cp * cl, cp * sl, sp}};
}

EnuFrame enuAlong(const Vec3& upDir) {
    const Vec3 up = normalize(upDir);
    Vec3 east = cross(Vec3{0.0, 0.0, 1.0}, up);
    east = dot(east, east) > 1e-8 ? normalize(east) : Vec3{1.0, 0.0, 0.0};
    return {east, cross(up, east), up};
}

Vec3 dirFromLonLat(double lonDeg, double latDeg) { return enuAt(lonDeg, latDeg).up; }

}  // namespace util

}  // namespace planet
}  // namespace chrono
