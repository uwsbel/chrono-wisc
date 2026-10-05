#ifndef CH_PLANET_MATHUTIL_H
#define CH_PLANET_MATHUTIL_H

#include "chrono_planet/ChApiPlanet.h"

#include <array>


namespace chrono {
namespace planet {

// Scalar and small-vector helpers shared by the terrain core.

namespace util {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

// Minimal double 3-vector.
struct CH_PLANET_API Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    // Reads xyz from a raw double triple.
    static Vec3 from(const double* p) { return {p[0], p[1], p[2]}; }
    // Writes xyz into a raw double triple.
    void store(double* p) const { p[0] = x, p[1] = y, p[2] = z; }
    // Component by index, 0, 1, 2 for x, y, z.
    double operator[](int i) const { return i == 0 ? x : i == 1 ? y
                                                                : z; }

    Vec3& operator+=(const Vec3& o) { return x += o.x, y += o.y, z += o.z, *this; }
    Vec3& operator/=(double s) { return x /= s, y /= s, z /= s, *this; }
};

// Componentwise arithmetic, dot and cross.
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Euclidean length.
double length(const Vec3& v);
// Returns v unchanged if it is too short to have a direction.
Vec3 normalize(const Vec3& v);
// Unit quaternion (x, y, z, w) from q, or q unchanged if it is too short to normalize.
[[nodiscard]] std::array<double, 4> normalized4(const std::array<double, 4>& q);
// Unit surface normal from a height gradient, in the gradient's frame. Rise and run share units.
Vec3 normalFromGradient(double dhdx, double dhdy);

// Left unparenthesized on purpose. Folding the constant changes the last bit and moves hashed cells.
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / kPi; }

// Linear interpolation, unclamped.
constexpr double lerp(double a, double b, double t) { return a + t * (b - a); }
constexpr float lerp(float a, float b, float t) { return a + t * (b - a); }

// Hermite and quintic eases on a t already in [0, 1].
constexpr double smoothstep01(double t) { return t * t * (3.0 - 2.0 * t); }
constexpr float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }
constexpr double smootherstep01(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }
constexpr float smootherstep01(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

// Signed [-1, 1] and unsigned [0, 1] to normalized integers. Out-of-range values wrap, so clamp first.
constexpr unsigned char packSnorm8(double v) { return static_cast<unsigned char>((v * 0.5 + 0.5) * 255.0 + 0.5); }
constexpr unsigned short packSnorm16(double v) {
    return static_cast<unsigned short>((v * 0.5 + 0.5) * 65535.0 + 0.5);
}
constexpr unsigned char packUnorm8(double v) { return static_cast<unsigned char>(v * 255.0 + 0.5); }

// The three components packed in xyz order, with the same wrap caveat as the scalar forms.
[[nodiscard]] std::array<unsigned char, 3> packSnorm8(const Vec3& v);
[[nodiscard]] std::array<unsigned short, 3> packSnorm16(const Vec3& v);

}  // namespace util

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_MATHUTIL_H
