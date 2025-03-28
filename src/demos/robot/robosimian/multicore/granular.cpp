// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include <algorithm>
#include <numeric>
#include <vector>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "granular.h"

using namespace chrono;

static constexpr int tag_particles = 100;

GroundGranular::GroundGranular(ChSystemMulticore* sys) : m_sys(sys), m_num_particles(0), m_radius(0), m_num_layers(0) {}

GroundGranularA::GroundGranularA(ChSystemMulticore* sys) : GroundGranular(sys) {
    m_terrain = chrono_types::make_shared<vehicle::GranularTerrain>(sys);
}

GroundGranularB::GroundGranularB(ChSystemMulticore* sys) : GroundGranular(sys) {
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetFixed(true);
    m_ground->EnableCollision(true);
    sys->AddBody(m_ground);
}

void GroundGranular::SetParticleProperties(double radius, double density, double friction, double cohesion) {
    m_radius = radius;
    m_density = density;
    m_friction = friction;
    m_cohesion = cohesion;
}

void GroundGranular::SetPatchProperties(double length, double width, unsigned int num_layers) {
    m_length = length;
    m_width = width;
    m_num_layers = num_layers;
}

void GroundGranular::Initialize(double x_min, double z_max, double step_size) {
    // Height of bottom boundary
    m_radius1 = 1.001 * m_radius;
    double bottom = z_max - m_num_layers * (2 * m_radius1);

    // Center of bottom boundary
    m_center = ChVector3d(x_min + m_length / 2, 0, bottom);

    // Create contact materials
    double coh_force = (CH_PI * m_radius * m_radius) * m_cohesion;

    switch (m_sys->GetContactMethod()) {
        case ChContactMethod::SMC: {
            auto mat_c = chrono_types::make_shared<ChContactMaterialSMC>();
            mat_c->SetFriction(0.0f);
            mat_c->SetRestitution(0.0f);
            mat_c->SetYoungModulus(8e5f);
            mat_c->SetPoissonRatio(0.3f);
            mat_c->SetAdhesion(0.0f);
            mat_c->SetKn(1.0e6f);
            mat_c->SetGn(6.0e1f);
            mat_c->SetKt(4.0e5f);
            mat_c->SetGt(4.0e1f);
            m_material_c = mat_c;

            auto mat_g = chrono_types::make_shared<ChContactMaterialSMC>();
            mat_g->SetFriction(static_cast<float>(m_friction));
            mat_g->SetRestitution(0.0f);
            mat_g->SetYoungModulus(8e5f);
            mat_g->SetPoissonRatio(0.3f);
            mat_g->SetAdhesion(static_cast<float>(coh_force));
            mat_g->SetKn(1.0e6f);
            mat_g->SetGn(6.0e1f);
            mat_g->SetKt(4.0e5f);
            mat_g->SetGt(4.0e1f);
            m_material_g = mat_g;

            break;
        }
        case ChContactMethod::NSC: {
            auto mat_c = chrono_types::make_shared<ChContactMaterialNSC>();
            mat_c->SetFriction(0.0f);
            mat_c->SetRestitution(0.0f);
            mat_c->SetCohesion(0.0f);
            m_material_c = mat_c;

            auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
            mat_g->SetFriction(static_cast<float>(m_friction));
            mat_g->SetRestitution(0.0f);
            mat_g->SetCohesion(static_cast<float>(coh_force * step_size));
            m_material_g = mat_g;

            break;
        }
    }

    // Adjust number of bins for collision detection
    int factor = 2;
    int binsX = (int)std::ceil((0.5 * m_length) / m_radius) / factor;
    int binsY = (int)std::ceil((0.5 * m_width) / m_radius) / factor;
    int binsZ = 1;
    m_sys->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

    // Adjust collision envelope (NSC only!)
    if (m_sys->GetContactMethod() == ChContactMethod::NSC)
        m_sys->GetSettings()->collision.collision_envelope = 0.02 * m_radius;
}

void GroundGranularA::Initialize(double x_min, double z_max, double step_size) {
    // Invoke base class method
    GroundGranular::Initialize(x_min, z_max, step_size);

    // Set contact material
    m_terrain->SetContactMaterial(m_material_g);
    if (m_sys->GetContactMethod() == ChContactMethod::NSC)
        m_terrain->SetCollisionEnvelope(0.02 * m_radius);

    m_terrain->EnableVisualization(true);
    m_terrain->EnableVerbose(true);
    m_terrain->Initialize(m_center, m_length, m_width, m_num_layers, m_radius, m_density);

    m_num_particles = m_terrain->GetNumParticles();
}

void GroundGranularB::Initialize(double x_min, double z_max, double step_size) {
    // Invoke base class method
    GroundGranular::Initialize(x_min, z_max, step_size);

    // Set position of ground body
    m_ground->SetPos(m_center);

    // Create container walls
    utils::AddBoxContainer(m_ground, m_material_c,                   //
                           ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT),  //
                           ChVector3d(m_length, m_width, 1), 0.1,    //
                           ChVector3i(2, 2, -1));

    // Create particles (all spheres)
    utils::ChPDSampler<double> sampler(2 * m_radius1);
    utils::ChGenerator gen(m_sys);
    std::shared_ptr<utils::ChMixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->SetDefaultMaterial(m_material_g);
    m1->SetDefaultDensity(m_density);
    m1->SetDefaultSize(m_radius);

    // Set starting value for body identifiers
    gen.SetStartTag(tag_particles);

    // Create particles in layers separated by an inflated particle diameter, starting at the bottom boundary
    ChVector3d hdims(m_length / 2 - m_radius1, m_width / 2 - m_radius1, 0);
    ChVector3d center = m_center;
    center.z() += 3 * m_radius1;

    for (unsigned int il = 0; il < m_num_layers; il++) {
        std::cout << "Create layer at " << center.z() << std::endl;
        gen.CreateObjectsBox(sampler, center, hdims);
        center.z() += 2 * m_radius1;
    }

    m_num_particles = gen.GetTotalNumBodies();
}

std::pair<double, double> GroundGranular::GetTopHeight(int num_samples) const {
    // Number of bins in x and y directions
    int nx = (int)std::ceil(std::sqrt(num_samples * m_length / m_width));
    int ny = (int)std::ceil(std::sqrt(num_samples * m_width / m_length));

    // Bin dimensions in x and y directions
    double dx = m_length / nx;
    double dy = m_width / ny;

    // Patch minimum x and y values
    double xm = m_center.x() - m_length / 2;
    double ym = m_center.y() - m_width / 2;

    // Initialize array of sampled heights
    num_samples = nx * ny;
    std::vector<double> heights(num_samples, -DBL_MAX);

    // Loop over all bodies in system
    for (auto body : m_sys->GetBodies()) {
        if (body->GetTag() < tag_particles)
            continue;
        auto pos = body->GetPos();
        int ix = (int)std::floor((pos.x() - xm) / dx);
        int iy = (int)std::floor((pos.y() - ym) / dy);
        int idx = ix * ny + iy;
        if (pos.z() > heights[idx])
            heights[idx] = pos.z();
    }

    ////for (auto h : heights)
    ////    std::cout << h << std::endl;
    ////std::cout << "    num samples: " << num_samples << "  " << heights.size() << std::endl;
    ////std::cout << "    min value:   " << *std::min_element(heights.begin(), heights.end()) << std::endl;
    ////std::cout << "    max value:   " << *std::max_element(heights.begin(), heights.end()) << std::endl;

    double avg_height = std::accumulate(heights.begin(), heights.end(), 0.0) / heights.size();
    double max_height = *std::max_element(heights.begin(), heights.end());

    return std::make_pair(max_height + m_radius, avg_height + m_radius);
}
