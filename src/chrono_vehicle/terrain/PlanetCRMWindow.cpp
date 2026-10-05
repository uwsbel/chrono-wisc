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

#include "chrono_vehicle/terrain/PlanetCRMWindow.h"

#include <algorithm>
#include <limits>
#include <random>
#include <cmath>
#include <stdexcept>

namespace chrono {
namespace vehicle {

PlanetCRMWindow::PlanetCRMWindow(ChSystem& sys,
                                 std::shared_ptr<const planet::ChPlanetSurface> surface,
                                 const planet::ChSiteFrame& site,
                                 std::shared_ptr<planet::ChDeformationFilter> ruts,
                                 double spacing,
                                 Setup setup)
    : m_sys(sys), m_surface(std::move(surface)), m_site(site), m_ruts(std::move(ruts)), m_spacing(spacing), m_setup(std::move(setup)) {
    if (!m_surface || !m_ruts || !m_setup)
        throw std::invalid_argument("PlanetCRMWindow: null surface, ruts or setup");
}

PlanetCRMWindow::~PlanetCRMWindow() {}

void PlanetCRMWindow::SetWindow(double length, double width, double depth, double margin) {
    m_length = length;
    m_width = width;
    m_depth = depth;
    m_margin = std::min(margin, 0.45 * std::min(length, width));
}

double PlanetCRMWindow::GetHeight(double x, double y) const {
    double lon, lat;
    m_site.ToLonLat(x, y, lon, lat);
    return m_surface->GetElevation(lon, lat) - m_site.GetOriginElevation() + m_ruts->GetDelta(lon, lat, 0.0);
}

namespace {
int64_t NodeKey(int i, int j) {
    return (int64_t(i) << 32) ^ int64_t(uint32_t(j));
}
}  // namespace

void PlanetCRMWindow::AddWheel(std::shared_ptr<ChBody> wheel, double radius, double width, const ChVector3d& axis) {
    if (!wheel || !(radius > 0) || !(width > 0) || axis.Length() < 1e-9)
        throw std::invalid_argument("PlanetCRMWindow::AddWheel: null wheel or bad size");
    m_wheels.push_back({std::move(wheel), radius, width, axis.GetNormalized()});
}

double PlanetCRMWindow::GetGround(const ChVector2i& node) {
    const int64_t key = NodeKey(node.x(), node.y());
    auto it = m_ground.find(key);
    if (it != m_ground.end())
        return it->second;
    const double h = m_ruts->GetSpacing();
    double lon, lat;
    m_site.ToLonLat(node.x() * h, node.y() * h, lon, lat);
    const double ground = m_surface->GetElevation(lon, lat) - m_site.GetOriginElevation();
    m_ground.emplace(key, ground);
    return ground;
}

void PlanetCRMWindow::Stamp(const Wheel& wheel) {
    // The wheel's footprint in the horizontal plane: across its axis and along its rolling direction
    const ChFrame<> X = wheel.body->GetFrameRefToAbs();
    const ChVector3d c = X.GetPos();
    const ChVector3d a = X.GetRot().Rotate(wheel.axis);
    const double a_len = std::hypot(a.x(), a.y());
    if (a_len < 0.2)
        return;  // wheel on its side
    const double ax = a.x() / a_len, ay = a.y() / a_len;
    // The rut is as wide as the wheel, flat across it, with walls sloping out from its edges at the soil's angle of
    // repose, up to the ground around it
    const double r = wheel.radius, half = 0.5 * wheel.width;
    const double wall_slope = std::tan(35 * CH_DEG_TO_RAD);
    const double wall_reach = 0.15;  // how far the walls may spread beyond the wheel's edge (m)
    const double h = m_ruts->GetSpacing();
    const double reach = r + half + wall_reach;
    const int i0 = static_cast<int>(std::ceil((c.x() - reach) / h)), i1 = static_cast<int>(std::floor((c.x() + reach) / h));
    const int j0 = static_cast<int>(std::ceil((c.y() - reach) / h)), j1 = static_cast<int>(std::floor((c.y() + reach) / h));
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            const double dx = i * h - c.x(), dy = j * h - c.y();
            const double lateral = std::abs(dx * ax + dy * ay);
            const double along = -dx * ay + dy * ax;
            if (lateral >= half + wall_reach || std::abs(along) >= r)
                continue;
            double bottom = c.z() - std::sqrt(r * r - along * along);
            if (lateral > half)
                bottom += (lateral - half) * wall_slope;
            const ChVector2i node(i, j);
            const double delta = bottom - GetGround(node);
            const int64_t key = NodeKey(i, j);
            auto it = m_pending.find(key);
            const double current = it != m_pending.end() ? it->second.delta : m_ruts->GetDelta(node);
            if (delta < current)
                m_pending[key] = {delta, bottom + m_site.GetOriginElevation()};
        }
}

PlanetCRMWindow::Surface PlanetCRMWindow::SoilSurface(const std::vector<ChVector3d>& positions) const {
    // The top of the soil over each column of the window's lattice: the top of the stack of particles there (loose
    // particles above a gap are left out), half a spacing above the highest one, as the soil is seeded
    const double s = m_spacing;
    Surface surface;
    surface.window = m_terrain->GetWindow();
    const planet::ChSiteRegion& window = surface.window;
    const int nx = surface.nx = static_cast<int>(std::round((window.max_x - window.min_x) / s)) + 1;
    const int ny = surface.ny = static_cast<int>(std::round((window.max_y - window.min_y) / s)) + 1;
    std::vector<std::vector<float>> stacks(size_t(nx) * ny);
    for (const auto& p : positions) {
        const int i = static_cast<int>(std::lround((p.x() - window.min_x) / s));
        const int j = static_cast<int>(std::lround((p.y() - window.min_y) / s));
        if (i >= 0 && i < nx && j >= 0 && j < ny)
            stacks[size_t(i) + size_t(nx) * j].push_back(static_cast<float>(p.z()));
    }
    std::vector<float> top(stacks.size(), std::numeric_limits<float>::quiet_NaN());
    for (size_t c = 0; c < stacks.size(); ++c) {
        auto& z = stacks[c];
        if (z.empty())
            continue;
        std::sort(z.begin(), z.end());
        size_t k = 0;
        while (k + 1 < z.size() && z[k + 1] - z[k] < 1.5 * s)
            ++k;
        top[c] = z[k] + static_cast<float>(0.5 * s);
    }
    // Smoothed over neighboring columns, which differ by the particles' jitter
    surface.top.assign(top.size(), std::numeric_limits<float>::quiet_NaN());
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            double sum = 0;
            int n = 0;
            for (int b = std::max(0, j - 1); b <= std::min(ny - 1, j + 1); ++b)
                for (int a = std::max(0, i - 1); a <= std::min(nx - 1, i + 1); ++a) {
                    const float t = top[size_t(a) + size_t(nx) * b];
                    if (!std::isnan(t))
                        sum += t, ++n;
                }
            if (n > 0)
                surface.top[size_t(i) + size_t(nx) * j] = static_cast<float>(sum / n);
        }
    return surface;
}

bool PlanetCRMWindow::Surface::Sample(double x, double y, double spacing, double& z) const {
    // Bilinear between the columns that have soil, away from the window's sides where the soil sags against the walls
    if (top.empty() || !window.Inset(2 * spacing).Contains(x, y))
        return false;
    const double u = (x - window.min_x) / spacing, v = (y - window.min_y) / spacing;
    const int a = std::clamp(static_cast<int>(u), 0, nx - 2), b = std::clamp(static_cast<int>(v), 0, ny - 2);
    const double fu = std::clamp(u - a, 0.0, 1.0), fv = std::clamp(v - b, 0.0, 1.0);
    double sum = 0, weight = 0;
    for (int db = 0; db <= 1; ++db)
        for (int da = 0; da <= 1; ++da) {
            const float t = top[size_t(a + da) + size_t(nx) * (b + db)];
            const double w = (da ? fu : 1 - fu) * (db ? fv : 1 - fv);
            if (!std::isnan(t))
                sum += w * t, weight += w;
        }
    if (weight <= 1e-6)
        return false;
    z = sum / weight;
    return true;
}

double PlanetCRMWindow::GetSeedHeight(double x, double y) const {
    // Over the old window, the soil as it was
    double z;
    if (m_carried.Sample(x, y, m_spacing, z))
        return z;
    return GetHeight(x, y);
}

void PlanetCRMWindow::Seed(const ChVector2d& center) {
    // The old window's ruts go into the filter; where the windows overlap, away from the old one's walls, its soil is
    // taken over as it is (particles with their stress, so the rover does not sink into fresh soil), and its surface is
    // what the new lattice fills up to; its ground body goes out of the system
    m_carried = Surface();
    m_emitted.clear();
    m_prev_velocities.clear();
    std::vector<PlanetCRMTerrain::ParticleState> carried;
    planet::ChSiteRegion carried_region;
    if (m_terrain) {
        Publish();
        carried = m_terrain->GetParticleStates();
        std::vector<ChVector3d> positions;
        for (const auto& q : carried)
            positions.push_back(q.pos);
        m_carried = SoilSurface(positions);
        carried_region = m_terrain->GetWindow().Inset(2 * m_spacing);
        m_terrain.reset();
    }
    m_ground.clear();
    const planet::ChSiteRegion window(center.x() - 0.5 * m_length, center.y() - 0.5 * m_width, center.x() + 0.5 * m_length,
                                      center.y() + 0.5 * m_width);
    m_terrain = std::make_unique<PlanetCRMTerrain>(m_sys, m_spacing);
    m_setup(*m_terrain);
    m_terrain->SetForceScale(m_force_scale);
    if (!carried.empty())
        m_terrain->SetCarriedParticles(std::move(carried), carried_region);
    m_terrain->ConstructFromHeight([this](double x, double y) { return GetSeedHeight(x, y); }, window, m_depth);
    m_carried = Surface();
    // The surface under the window, for telling loose soil from the ground
    m_columns_origin = ChVector2d(window.min_x, window.min_y);
    m_columns_nx = static_cast<int>(std::round((window.max_x - window.min_x) / m_spacing)) + 1;
    m_columns_ny = static_cast<int>(std::round((window.max_y - window.min_y) / m_spacing)) + 1;
    m_columns.resize(size_t(m_columns_nx) * m_columns_ny);
    for (int j = 0; j < m_columns_ny; ++j)
        for (int i = 0; i < m_columns_nx; ++i) {
            double lon, lat;
            m_site.ToLonLat(window.min_x + i * m_spacing, window.min_y + j * m_spacing, lon, lat);
            m_columns[size_t(i) + size_t(m_columns_nx) * j] = static_cast<float>(m_surface->GetElevation(lon, lat) - m_site.GetOriginElevation());
        }
    m_terrain->Initialize();
    if (m_wheels.empty())
        m_terrain->PublishToDeformation(*m_ruts);  // records where the soil starts
}

void PlanetCRMWindow::SetForceScale(double scale) {
    m_force_scale = scale;
    if (m_terrain)
        m_terrain->SetForceScale(scale);
}

void PlanetCRMWindow::Initialize(std::shared_ptr<ChBody> follow) {
    if (!follow)
        throw std::invalid_argument("PlanetCRMWindow::Initialize: null body to follow");
    m_follow = std::move(follow);
    const ChVector3d p = m_follow->GetPos();
    Seed(ChVector2d(p.x(), p.y()));
}

void PlanetCRMWindow::Advance(double step) {
    // Move the window ahead of the body along its motion when it nears an edge
    const ChVector3d p = m_follow->GetPos();
    const planet::ChSiteRegion inner = m_terrain->GetWindow().Inset(m_margin);
    if (!inner.Contains(p.x(), p.y())) {
        ChVector3d v = m_follow->GetPosDt();
        v.z() = 0;
        const double speed = v.Length();
        const ChVector3d ahead = speed > 1e-6 ? v / speed : ChVector3d(0, 0, 0);
        // lead by a quarter of the window, but no further than keeps the body well inside the new window's inner region
        const double reach_x = 0.8 * (0.5 * m_length - m_margin), reach_y = 0.8 * (0.5 * m_width - m_margin);
        double lead = 0.25 * std::min(m_length, m_width);
        if (std::abs(ahead.x()) * lead > reach_x)
            lead = reach_x / std::abs(ahead.x());
        if (std::abs(ahead.y()) * lead > reach_y)
            lead = reach_y / std::abs(ahead.y());
        Seed(ChVector2d(p.x() + ahead.x() * lead, p.y() + ahead.y() * lead));
        ++m_moves;
    }
    m_terrain->DoStepDynamics(step);
    for (const auto& wheel : m_wheels)
        Stamp(wheel);
}

std::shared_ptr<ChTriangleMeshConnected> PlanetCRMWindow::UpdateLooseSoil() {
    // Clods: radius 0.45 to 0.65 spacings, blend 0.15, meshed at half a spacing
    const double s = m_spacing;
    if (!m_loose)
        m_loose = std::make_unique<planet::ChSparseSdfGrid>(0.5 * s, static_cast<float>(2 * s));
    std::vector<ChVector3d> positions = m_terrain->GetFluidSystemSPH()->GetParticlePositions();
    positions.resize(std::min(positions.size(), m_terrain->GetNumSPHParticles()));
    const double h = m_ruts->GetSpacing();
    m_loose->Begin();
    m_num_loose = 0;
    for (size_t n = 0; n < positions.size(); ++n) {
        const ChVector3d& p = positions[n];
        const double u = std::clamp((p.x() - m_columns_origin.x()) / s, 0.0, m_columns_nx - 1.0001);
        const double v = std::clamp((p.y() - m_columns_origin.y()) / s, 0.0, m_columns_ny - 1.0001);
        const int a = static_cast<int>(u), b = static_cast<int>(v);
        const double fu = u - a, fv = v - b;
        auto at = [&](int i, int j) { return double(m_columns[size_t(i) + size_t(m_columns_nx) * j]); };
        double ground = (1 - fu) * (1 - fv) * at(a, b) + fu * (1 - fv) * at(a + 1, b) + (1 - fu) * fv * at(a, b + 1) + fu * fv * at(a + 1, b + 1);
        ground += m_ruts->GetDelta(ChVector2i(static_cast<int>(std::lround(p.x() / h)), static_cast<int>(std::lround(p.y() / h))));
        if (p.z() < ground + 0.1 * s)
            continue;
        // a size of its own for each particle, the same from frame to frame
        uint32_t hash = static_cast<uint32_t>(n) * 2654435761u;
        hash ^= hash >> 16;
        const double size = 0.45 + 0.2 * (hash & 0xffff) / 65535.0;
        m_loose->SplatSphere(p, size * s, 0.15 * s);
        ++m_num_loose;
    }
    m_loose->Remesh();
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    m_loose->AppendMeshes(*mesh);
    return mesh;
}

size_t PlanetCRMWindow::Publish() {
    if (m_wheels.empty())
        return m_terrain->PublishToDeformation(*m_ruts);
    if (m_berms)
        RaiseBerms();
    // Rut floors carry the wheel's heights, so the drawn terrain presses its finer relief flat there
    std::vector<planet::ChDeformationFilter::Node> nodes;
    nodes.reserve(m_pending.size());
    for (const auto& [key, p] : m_pending)
        nodes.push_back({ChVector2i(static_cast<int>(key >> 32), static_cast<int>(uint32_t(key))), p.delta, p.height});
    m_pending.clear();
    return m_ruts->SetNodes(nodes, 0.0);
}

void PlanetCRMWindow::RaiseBerms() {
    // Soil piled above the ground near the wheels raises it there, never over a rut and never back down: berms along
    // the ruts and soil pushed ahead of the wheels. Elsewhere the soil's jitter would show as noise.
    std::vector<ChVector3d> positions = m_terrain->GetFluidSystemSPH()->GetParticlePositions();
    positions.resize(std::min(positions.size(), m_terrain->GetNumSPHParticles()));
    const Surface surface = SoilSurface(positions);
    const double h = m_ruts->GetSpacing();
    const double min_height = 0.1 * m_spacing;  // piles lower than this are the soil's jitter
    for (const auto& wheel : m_wheels) {
        const ChVector3d c = wheel.body->GetFrameRefToAbs().GetPos();
        const double reach = 2 * wheel.radius + wheel.width;
        const int i0 = static_cast<int>(std::ceil((c.x() - reach) / h)), i1 = static_cast<int>(std::floor((c.x() + reach) / h));
        const int j0 = static_cast<int>(std::ceil((c.y() - reach) / h)), j1 = static_cast<int>(std::floor((c.y() + reach) / h));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const int64_t key = NodeKey(i, j);
                if (m_pending.count(key))
                    continue;  // stamped by a wheel this step
                const ChVector2i node(i, j);
                const double current = m_ruts->GetDelta(node);
                if (current < 0)
                    continue;  // a rut
                double top;
                if (!surface.Sample(i * h, j * h, m_spacing, top))
                    continue;
                const double pile = top - GetGround(node);
                if (pile > min_height && pile > current)
                    m_pending[key] = {pile, std::numeric_limits<double>::quiet_NaN()};
            }
    }
}

std::vector<PlanetCRMWindow::Ejecta> PlanetCRMWindow::MeasureEjecta(double time, double min_speed) {
    std::vector<Ejecta> ejecta(m_wheels.size());
    const auto sph = m_terrain->GetFluidSystemSPH();
    std::vector<ChVector3d> positions = sph->GetParticlePositions();
    std::vector<ChVector3d> velocities = sph->GetParticleVelocities();
    const size_t n = std::min(positions.size(), m_terrain->GetNumSPHParticles());
    velocities.resize(n);
    const bool have_prev = m_prev_velocities.size() == n && time > m_prev_time;
    const double dt = time - m_prev_time;
    const ChVector3d g = m_sys.GetGravitationalAcceleration();
    const double mass = sph->GetParticleMass();
    std::vector<ChVector3d> v2(m_wheels.size(), VNULL);
    for (size_t k = 0; have_prev && k < n; ++k) {
        const ChVector3d& p = positions[k];
        const ChVector3d& v = velocities[k];
        if (v.Length() < min_speed || m_emitted.count(k))
            continue;
        if (((v - m_prev_velocities[k]) / dt - g).Length() > 0.25 * g.Length())
            continue;
        if (p.z() - GetHeight(p.x(), p.y()) < 0.5 * m_spacing)
            continue;
        // the nearest wheel within reach
        int best = -1;
        double best_d = 1e300;
        for (size_t w = 0; w < m_wheels.size(); ++w) {
            const ChVector3d c = m_wheels[w].body->GetFrameRefToAbs().GetPos();
            const double d = std::hypot(p.x() - c.x(), p.y() - c.y());
            if (d < best_d && d < 3 * m_wheels[w].radius)
                best = static_cast<int>(w), best_d = d;
        }
        m_emitted.insert(k);
        if (best < 0)
            continue;
        Ejecta& e = ejecta[best];
        e.mass += mass;
        e.pos += p * mass;
        e.vel += v * mass;
        v2[best] += ChVector3d(v.x() * v.x(), v.y() * v.y(), v.z() * v.z()) * mass;
    }
    for (size_t w = 0; w < ejecta.size(); ++w) {
        Ejecta& e = ejecta[w];
        if (e.mass <= 0)
            continue;
        e.pos /= e.mass;
        e.vel /= e.mass;
        const ChVector3d var = v2[w] / e.mass - ChVector3d(e.vel.x() * e.vel.x(), e.vel.y() * e.vel.y(), e.vel.z() * e.vel.z());
        e.vel_spread = std::sqrt(std::max(0.0, (var.x() + var.y() + var.z()) / 3));
    }
    m_prev_velocities = std::move(velocities);
    m_prev_time = time;
    return ejecta;
}

size_t PlanetCRMWindow::EmitDust(ChDustField& dust, double time, double min_speed, int splits) {
    // Particles in free flight: clear of the drawn ground, moving, and falling freely since the last call (their
    // velocity changed as gravity alone would change it; soil pushed aside by a wheel, still in contact, does not).
    // Each is handed over once, its mass shared among the dust's grain sizes and among a few super-particles spread
    // over its volume, so the dust follows the soil.
    const auto sph = m_terrain->GetFluidSystemSPH();
    std::vector<ChVector3d> positions = sph->GetParticlePositions();
    std::vector<ChVector3d> velocities = sph->GetParticleVelocities();
    const size_t n = std::min(positions.size(), m_terrain->GetNumSPHParticles());
    velocities.resize(n);
    const bool have_prev = m_prev_velocities.size() == n && time > m_prev_time;
    const double dt = time - m_prev_time;
    const ChVector3d g = m_sys.GetGravitationalAcceleration();
    const double s = m_spacing;
    const double mass = sph->GetParticleMass();
    const auto& bins = dust.GetParams().bins;
    std::uniform_real_distribution<double> jitter(-0.5, 0.5);
    size_t emitted = 0;
    for (size_t k = 0; have_prev && k < n; ++k) {
        const ChVector3d& p = positions[k];
        const ChVector3d& v = velocities[k];
        if (v.Length() < min_speed || m_emitted.count(k))
            continue;
        if (((v - m_prev_velocities[k]) / dt - g).Length() > 0.25 * g.Length())
            continue;
        if (p.z() - GetHeight(p.x(), p.y()) < 0.5 * s)
            continue;
        m_emitted.insert(k);
        ++emitted;
        // A thrown parcel is not a clod: in vacuum its grains part at once, at the spread of speeds the shearing soil
        // gave them. Its mass goes out over the stretch of its path since the previous call, so parcels caught one
        // call apart join into a stream, as grains of 60% to 120% of its speed, up to about 20 degrees off its path.
        const double speed = v.Length();
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        for (int part = 0; part < splits; ++part) {
            const double back = unit(m_rng) * dt;
            const ChVector3d start = p - v * back + 0.5 * g * back * back;
            const ChVector3d offset(jitter(m_rng) * s, jitter(m_rng) * s, jitter(m_rng) * s);
            ChVector3d dir;
            do {
                dir = ChVector3d(jitter(m_rng), jitter(m_rng), jitter(m_rng)) * 2.0;
            } while (dir.Length2() > 1.0);
            const ChVector3d vel = v * (0.6 + 0.6 * unit(m_rng)) + dir * (0.35 * speed);
            for (size_t b = 0; b < bins.size(); ++b)
                dust.Emit(start + offset, vel, mass * bins[b].mass_fraction / splits, static_cast<int>(b), time - back);
        }
    }
    m_prev_velocities = std::move(velocities);
    m_prev_time = time;
    return emitted;
}

}  // namespace vehicle
}  // namespace chrono
