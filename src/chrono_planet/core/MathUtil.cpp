#include "chrono_planet/core/MathUtil.h"

#include <cmath>

namespace chrono {
namespace planet {

namespace util {

double length(const Vec3& v) { return std::sqrt(dot(v, v)); }

Vec3 normalize(const Vec3& v) {
    const double len = length(v);
    return len > 1e-8 ? Vec3{v.x / len, v.y / len, v.z / len} : v;
}

std::array<double, 4> normalized4(const std::array<double, 4>& q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n <= 1e-12) {
        return q;
    }
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Vec3 normalFromGradient(double dhdx, double dhdy) {
    const double len = std::sqrt(dhdx * dhdx + dhdy * dhdy + 1.0);
    return {-dhdx / len, -dhdy / len, 1.0 / len};
}

std::array<unsigned char, 3> packSnorm8(const Vec3& v) {
    return {packSnorm8(v.x), packSnorm8(v.y), packSnorm8(v.z)};
}

std::array<unsigned short, 3> packSnorm16(const Vec3& v) {
    return {packSnorm16(v.x), packSnorm16(v.y), packSnorm16(v.z)};
}

}  // namespace util

}  // namespace planet
}  // namespace chrono
