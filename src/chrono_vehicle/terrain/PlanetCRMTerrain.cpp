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

#include "chrono_vehicle/terrain/PlanetCRMTerrain.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

namespace chrono {
namespace vehicle {

namespace {
// Radius of a particle's sphere, in particle spacings: the soil's top is the highest cap of a sphere over a column.
// With this radius, particles seeded on a lattice give back the ground they came from to about 0.05%.
constexpr double kRadius = 0.64;
}  // namespace

PlanetCRMTerrain::PlanetCRMTerrain(ChSystem& sys, double spacing) : CRMTerrain(sys, spacing) {
    m_sysFSI->RegisterMBDCallback(chrono_types::make_shared<ScaledAdvance>(*this));
}

void PlanetCRMTerrain::ScaledAdvance::Advance(double step, double threshold) {
    // The soil's wrench on each body is held in the body's soil accumulator: force at the center of mass (absolute),
    // torque in the body frame. Scaling both scales the wrench.
    const double scale = m_terrain.m_force_scale;
    for (const auto& fsi_body : m_terrain.m_sysFSI->GetBodies()) {
        auto& body = fsi_body->body;
        const unsigned int idx = fsi_body->fsi_accumulator;
        const ChVector3d force = body->GetAccumulatedForce(idx);
        const ChVector3d torque = body->GetAccumulatedTorque(idx);
        m_terrain.m_wrenches[body.get()] = {force, body->GetRotMat() * torque};
        if (scale != 1) {
            body->EmptyAccumulator(idx);
            body->AccumulateForce(idx, force * scale, body->GetFrameCOMToAbs().GetPos(), false);
            body->AccumulateTorque(idx, torque * scale, true);
        }
    }
    // then as ChFsiSystem advances the multibody system without a callback
    ChSystem& sys = m_terrain.m_sysFSI->GetMultibodySystem();
    const double h_mbd = m_terrain.m_sysFSI->GetStepSizeMBD();
    double t = 0;
    while (t < step) {
        const double h = std::min(h_mbd, step - t);
        if (h <= threshold)
            break;
        sys.DoStepDynamics(h);
        t += h;
    }
}

bool PlanetCRMTerrain::GetSoilWrench(const ChBody& body, ChVector3d& force, ChVector3d& torque) const {
    auto itr = m_wrenches.find(&body);
    if (itr == m_wrenches.end()) {
        force = VNULL;
        torque = VNULL;
        return false;
    }
    force = itr->second.first;
    torque = itr->second.second;
    return true;
}

PlanetCRMTerrain::~PlanetCRMTerrain() {
    // Initialize added the soil model's ground body to the system; a window replaced by another must not leave it
    if (m_sysMBS && m_ground && m_ground->GetSystem() == m_sysMBS)
        m_sysMBS->RemoveBody(m_ground);
    // nor its last soil forces on the bodies it touched: their accumulators outlive it
    if (m_sysFSI && m_initialized) {
        for (const auto& fsi_body : m_sysFSI->GetBodies())
            fsi_body->body->EmptyAccumulator(fsi_body->fsi_accumulator);
    }
}

void PlanetCRMTerrain::ConstructFromHeight(const std::function<double(double x, double y)>& height,
                                           const planet::ChSiteRegion& window,
                                           double depth,
                                           int side_flags) {
    if (!height)
        throw std::invalid_argument("PlanetCRMTerrain::ConstructFromHeight: no height function");
    // Heights on the lattice's columns, bilinear between them, so particles fill each column up to its ground
    const double s = m_spacing;
    const int nx = static_cast<int>(std::round((window.max_x - window.min_x) / s)) + 1;
    const int ny = static_cast<int>(std::round((window.max_y - window.min_y) / s)) + 1;
    std::vector<double> column(size_t(nx) * ny);
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            column[size_t(i) + size_t(nx) * j] = height(window.min_x + i * s, window.min_y + j * s);
    auto top = [&](double x, double y) {
        const int i = std::clamp(static_cast<int>(std::lround((x - window.min_x) / s)), 0, nx - 1);
        const int j = std::clamp(static_cast<int>(std::lround((y - window.min_y) / s)), 0, ny - 1);
        return column[size_t(i) + size_t(nx) * j];
    };
    // Each column is shifted up or down by less than half a spacing so its top particle lies half a spacing below the
    // ground: the soil's surface follows the ground instead of stepping by whole spacings
    m_shift.assign(column.size(), 0.0);
    for (size_t c = 0; c < column.size(); ++c) {
        const double z = column[c] - 0.5 * s;
        m_shift[c] = z - std::round(z / s) * s;
    }
    // Carried soil over the window, above the floor Build will lay; under it the lattice fills each column up to a
    // spacing below its lowest carried particle
    double lowest = 1e300;
    for (double c : column)
        lowest = std::min(lowest, c);
    const double floor = std::floor((lowest - depth) / s) * s;
    const planet::ChSiteRegion region(std::max(window.min_x, m_carried_region.min_x), std::max(window.min_y, m_carried_region.min_y),
                                      std::min(window.max_x, m_carried_region.max_x), std::min(window.max_y, m_carried_region.max_y));
    std::vector<ParticleState> carried;
    std::vector<double> carried_bottom(column.size(), 1e300);
    std::vector<char> carried_column(column.size(), 0);
    if (!m_carried.empty() && !region.IsEmpty()) {
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i)
                if (region.Contains(window.min_x + i * s, window.min_y + j * s))
                    carried_column[size_t(i) + size_t(nx) * j] = 1;
        for (const auto& q : m_carried) {
            if (!region.Contains(q.pos.x(), q.pos.y()) || q.pos.z() < floor + 0.5 * s)
                continue;
            const int i = std::clamp(static_cast<int>(std::lround((q.pos.x() - window.min_x) / s)), 0, nx - 1);
            const int j = std::clamp(static_cast<int>(std::lround((q.pos.y() - window.min_y) / s)), 0, ny - 1);
            double& bottom = carried_bottom[size_t(i) + size_t(nx) * j];
            bottom = std::min(bottom, q.pos.z());
            carried.push_back(q);
        }
    }
    m_carried = std::move(carried);
    auto column_of = [&](double x, double y) {
        const int i = std::clamp(static_cast<int>(std::lround((x - window.min_x) / s)), 0, nx - 1);
        const int j = std::clamp(static_cast<int>(std::lround((y - window.min_y) / s)), 0, ny - 1);
        return size_t(i) + size_t(nx) * j;
    };

    if (m_overburden)
        m_column_ground = column;
    if (m_overburden || !m_carried.empty())
        RegisterParticlePropertiesCallback(chrono_types::make_shared<SeedCallback>(*this));
    Build(window, depth, side_flags, top, [&](const ChVector3d& p) {
        const size_t c = column_of(p.x(), p.y());
        if (carried_column[c])
            return p.z() < std::min(carried_bottom[c], top(p.x(), p.y())) - 0.75 * s;
        return p.z() < top(p.x(), p.y()) - 0.25 * s;
    });

    // The carried particles, marked on the lattice for Grid2Point to place, and indexed for SeedCallback to find
    m_carried_index.clear();
    for (size_t n = 0; n < m_carried.size(); ++n) {
        m_sph.insert(ChVector3i(kCarried, static_cast<int>(n), 0));
        m_carried_index[CarriedKey(m_carried[n].pos)] = n;
    }
}

std::vector<PlanetCRMTerrain::ParticleState> PlanetCRMTerrain::GetParticleStates() const {
    const size_t n = GetNumSPHParticles();
    std::vector<ChVector3d> pos = m_sysSPH->GetParticlePositions();
    std::vector<ChVector3d> vel = m_sysSPH->GetParticleVelocities();
    std::vector<ChVector3d> props = m_sysSPH->GetParticleFluidProperties();
    std::vector<ChVector3d> diag, offdiag;
    m_sysSPH->GetParticleStresses(diag, offdiag);
    std::vector<ParticleState> states;
    states.reserve(n);
    for (size_t i = 0; i < n && i < pos.size(); ++i)
        states.push_back({pos[i], vel[i], props[i].x(), props[i].y(), props[i].z(), diag[i], offdiag[i]});
    return states;
}

void PlanetCRMTerrain::SetCarriedParticles(std::vector<ParticleState> particles, const planet::ChSiteRegion& region) {
    m_carried = std::move(particles);
    m_carried_region = region;
}

int64_t PlanetCRMTerrain::CarriedKey(const ChVector3d& pos) {
    // Positions quantized to 0.1 mm: a carried particle is looked up by where Grid2Point placed it
    const auto q = [](double v) { return int64_t(std::llround(v * 1e4)) & 0x1fffff; };
    return (q(pos.x()) << 42) | (q(pos.y()) << 21) | q(pos.z());
}

void PlanetCRMTerrain::SeedCallback::set(const fsi::sph::ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) {
    ParticlePropertiesCallback::set(sysSPH, pos);
    const PlanetCRMTerrain& t = m_terrain;
    auto it = t.m_carried_index.find(CarriedKey(pos));
    if (it != t.m_carried_index.end()) {
        const ParticleState& q = t.m_carried[it->second];
        p0 = q.pressure;
        rho0 = q.rho;
        mu0 = q.mu;
        v0 = q.vel;
        tau_diag = q.tau_diag;
        tau_offdiag = q.tau_offdiag;
        return;
    }
    if (!t.m_overburden)
        return;
    // The weight of the soil above, up to the ground over the particle's column
    const int i = std::clamp(static_cast<int>(std::lround((pos.x() - t.m_window.min_x) / t.m_spacing)), 0, t.m_nx - 1);
    const int j = std::clamp(static_cast<int>(std::lround((pos.y() - t.m_window.min_y) / t.m_spacing)), 0, t.m_ny - 1);
    const double depth = std::max(0.0, t.m_column_ground[size_t(i) + size_t(t.m_nx) * j] - pos.z());
    const double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    const double c = sysSPH.GetSoundSpeed();
    p0 = sysSPH.GetDensity() * gz * depth;
    rho0 = c > 0 ? sysSPH.GetDensity() + p0 / (c * c) : sysSPH.GetDensity();
    tau_diag = ChVector3d(-p0);
}

ChVector3d PlanetCRMTerrain::Grid2Point(const ChVector3i& p) {
    if (p.x() == kCarried)
        return m_carried[size_t(p.y())].pos - m_offset_sph;
    // As for any Cartesian soil lattice (ChFsiProblemCartesian::Grid2Point, which is private)
    ChVector3d point(m_spacing * p.x(), m_spacing * p.y(), m_spacing * p.z());
    // Soil lattice points (in a column, above the floor) take their column's shift; boundary markers do not
    if (!m_shift.empty() && p.x() >= 0 && p.x() < m_nx && p.y() >= 0 && p.y() < m_ny && p.z() >= m_k_floor)
        point.z() += m_shift[size_t(p.x()) + size_t(m_nx) * p.y()];
    return point;
}

void PlanetCRMTerrain::Build(const planet::ChSiteRegion& window,
                             double depth,
                             int side_flags,
                             const std::function<double(double, double)>& top_at,
                             const std::function<bool(const ChVector3d&)>& soil_at) {
    using namespace fsi::sph;
    if (window.IsEmpty() || !(depth > 0))
        throw std::invalid_argument("PlanetCRMTerrain: empty window or depth");
    m_window = window;
    m_reference.clear();
    const double s = m_spacing;

    // The lattice: columns from the window's corner, levels at whole multiples of the spacing in site z
    const int nx = static_cast<int>(std::round((window.max_x - window.min_x) / s)) + 1;
    const int ny = static_cast<int>(std::round((window.max_y - window.min_y) / s)) + 1;
    m_nx = nx;
    m_ny = ny;
    double lowest = 1e300, highest = -1e300;
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            const double top = top_at(window.min_x + i * s, window.min_y + j * s);
            lowest = std::min(lowest, top);
            highest = std::max(highest, top);
        }
    const int k_floor = static_cast<int>(std::floor((lowest - depth) / s));
    const int k_top = static_cast<int>(std::ceil(highest / s)) + 1;
    m_floor = k_floor * s;
    m_k_floor = k_floor;
    const int layers = m_sysSPH->GetNumBCELayers();

    // A particle wherever there is soil, above the floor; boundary markers under it
    m_sph.clear();
    m_bce.clear();
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            const double x = window.min_x + i * s, y = window.min_y + j * s;
            const double shift = m_shift.empty() ? 0.0 : m_shift[size_t(i) + size_t(nx) * j];
            for (int k = k_floor; k <= k_top; ++k)
                if (soil_at(ChVector3d(x, y, k * s + shift)))
                    m_sph.insert(ChVector3i(i, j, k));
            if (side_flags & BoxSide::Z_NEG)
                for (int l = 1; l <= layers; ++l)
                    m_bce.insert(ChVector3i(i, j, k_floor - l));
        }

    // Walls around the window, from under the floor to above the highest ground
    const int k0 = k_floor - layers, k1 = k_top + layers;
    auto wall = [&](int i0, int i1, int j0, int j1) {
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                for (int k = k0; k <= k1; ++k)
                    m_bce.insert(ChVector3i(i, j, k));
    };
    if (side_flags & BoxSide::X_NEG)
        wall(-layers, -1, -layers, ny - 1 + layers);
    if (side_flags & BoxSide::X_POS)
        wall(nx, nx - 1 + layers, -layers, ny - 1 + layers);
    if (side_flags & BoxSide::Y_NEG)
        wall(0, nx - 1, -layers, -1);
    if (side_flags & BoxSide::Y_POS)
        wall(0, nx - 1, ny, ny - 1 + layers);

    m_offset_sph = ChVector3d(window.min_x, window.min_y, 0);
    m_offset_bce = m_offset_sph;

    // The computational domain: by default the soil model fits it to the markers it starts with, and would drop
    // soil a tool lifts above them
    const double around = m_margin + layers * s;
    SetComputationalDomain(ChAABB(ChVector3d(window.min_x - around, window.min_y - around, m_floor - (layers + 1) * s),
                                  ChVector3d(window.max_x + around, window.max_y + around, highest + m_headroom)),
                           BC_NONE);
    if (m_verbose)
        std::cout << "PlanetCRMTerrain: " << m_sph.size() << " SPH particles and " << m_bce.size() << " BCE markers over the window, floor at "
                  << m_floor << " m" << std::endl;
}

size_t PlanetCRMTerrain::PublishToDeformation(planet::ChDeformationFilter& filter, double tolerance) {
    // The top of the soil over each lattice column: the highest cap of a particle's sphere above it
    std::vector<ChVector3d> positions = m_sysSPH->GetParticlePositions();
    positions.resize(std::min(positions.size(), GetNumSPHParticles()));
    const double s = m_spacing;
    const double r = kRadius * s;
    std::vector<float> top(size_t(m_nx) * m_ny, -1e30f);
    for (const auto& p : positions) {
        const int ci = static_cast<int>(std::lround((p.x() - m_window.min_x) / s));
        const int cj = static_cast<int>(std::lround((p.y() - m_window.min_y) / s));
        for (int j = std::max(0, cj - 1); j <= std::min(m_ny - 1, cj + 1); ++j)
            for (int i = std::max(0, ci - 1); i <= std::min(m_nx - 1, ci + 1); ++i) {
                const double dx = p.x() - (m_window.min_x + i * s), dy = p.y() - (m_window.min_y + j * s);
                const double d2 = dx * dx + dy * dy;
                if (d2 >= r * r)
                    continue;
                float& t = top[size_t(i) + size_t(m_nx) * j];
                t = std::max(t, static_cast<float>(p.z() + std::sqrt(r * r - d2)));
            }
    }
    for (auto& t : top)
        if (t < -1e29f)
            t = static_cast<float>(m_floor);

    // The filter's nodes over the window, and the change to their height the first publish of this window starts from:
    // the soil as it was seeded, whatever ruts the filter already held there
    const double h = filter.GetSpacing();
    const int ni0 = static_cast<int>(std::ceil(m_window.min_x / h)), ni1 = static_cast<int>(std::floor(m_window.max_x / h));
    const int nj0 = static_cast<int>(std::ceil(m_window.min_y / h)), nj1 = static_cast<int>(std::floor(m_window.max_y / h));
    if (m_reference.empty()) {
        m_reference = top;
        m_base.clear();
        for (int j = nj0; j <= nj1; ++j)
            for (int i = ni0; i <= ni1; ++i)
                m_base.push_back(static_cast<float>(filter.GetDelta(ChVector2i(i, j))));
        return 0;
    }

    // Each node's height changes by how much the soil's top moved over it since the seeding, bilinear between columns
    std::vector<std::pair<ChVector2i, double>> nodes;
    nodes.reserve(m_base.size());
    size_t n = 0;
    for (int j = nj0; j <= nj1; ++j)
        for (int i = ni0; i <= ni1; ++i, ++n) {
            const double u = std::clamp((i * h - m_window.min_x) / s, 0.0, m_nx - 1.0001);
            const double v = std::clamp((j * h - m_window.min_y) / s, 0.0, m_ny - 1.0001);
            const int a = static_cast<int>(u), b = static_cast<int>(v);
            const double fu = u - a, fv = v - b;
            auto change = [&](int ca, int cb) {
                const size_t c = size_t(ca) + size_t(m_nx) * cb;
                return double(top[c]) - m_reference[c];
            };
            const double dz = (1 - fu) * (1 - fv) * change(a, b) + fu * (1 - fv) * change(a + 1, b) + (1 - fu) * fv * change(a, b + 1) + fu * fv * change(a + 1, b + 1);
            nodes.push_back({ChVector2i(i, j), m_base[n] + dz});
        }
    return filter.SetDeltas(nodes, tolerance);
}

}  // namespace vehicle
}  // namespace chrono
