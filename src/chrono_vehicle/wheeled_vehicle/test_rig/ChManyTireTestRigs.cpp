// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// Implementation of manager class for multiple tire test rigs.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChManyTireTestRigs.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include <stdexcept>

namespace chrono {
namespace vehicle {

// =============================================================================
// RigConfiguration Implementation
// =============================================================================

void ChManyTireTestRigs::RigConfiguration::ApplyTo(ChTireTestRig& rig) const {
    // Apply basic parameters
    rig.SetNormalLoad(normal_load);
    rig.SetCamberAngle(camber_angle);
    rig.SetTimeDelay(time_delay);
    rig.SetTireStepsize(tire_step);
    rig.SetTireCollisionType(collision_type);
    rig.SetTireVisualizationType(tire_visualization);

    // Apply motion functions if specified
    if (long_speed_function) {
        rig.SetLongSpeedFunction(long_speed_function);
    }
    if (ang_speed_function) {
        rig.SetAngSpeedFunction(ang_speed_function);
    }
    if (slip_angle_function) {
        rig.SetSlipAngleFunction(slip_angle_function);
    }

    // Apply terrain configuration if specified
    rig.SetTerrainRigid(terrain_params_rigid);
}

ChManyTireTestRigs::ChManyTireTestRigs(ChSystem* system) : m_system(system), m_last_rig_offset(ChVector3d(0, 0, 0)) {
    if (!system) {
        throw std::invalid_argument("ChManyTireTestRigs: ChSystem pointer cannot be null");
    }
}

// -----------------------------------------------------------------------------
// Rig Management
// -----------------------------------------------------------------------------

ChTireTestRig& ChManyTireTestRigs::AddRig(std::string wheel_json, std::string tire_json, bool use_airless_tire) {
    std::shared_ptr<ChTire> tire;
    auto wheel = ReadWheelJSON(GetVehicleDataFile(wheel_json));

    if (!use_airless_tire) {
        tire = ReadTireJSON(GetVehicleDataFile(tire_json));
    } else {
        tire = chrono_types::make_shared<ANCFAirlessTire3443B>("ANCFairless tire");
        // These are default sizes for the polaris tire
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.225);           // Default is 0.225
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.225);              // Default is 0.225
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.4);                 // Default is 0.4
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);                // Default is 0.05
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(0.015);  // Default is 0.015
        // tire->SetYoungsModulus(y_mod);  // Default is 76e9
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e6);
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(5e9);
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);       // Default is 0.2
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);              // Default is 3
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);        // Default is 3
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);  // Default is 3
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetNumberSpokes(40);
        // Use straight spokes
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHubRelativeRotation(0);
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureXPoint(0);
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureZPoint(0);
    }

    bool fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;

    // Set tire contact surface (relevant for FEA tires only)
    if (fea_tire) {
        int collision_family = 7;
        auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
        double surface_dim = 0;
        tire->SetContactSurfaceType(surface_type, surface_dim, collision_family);
    }
    auto rig = std::make_unique<ChTireTestRig>(wheel, tire, m_system, m_last_rig_offset);
    rig->SetGravitationalAcceleration(m_grav);
    m_rigs.push_back(std::move(rig));
    // Offset each rig by 1 m in the y axis
    m_last_rig_offset += ChVector3d(0, 1, 0);
    return *m_rigs.back();
}

void ChManyTireTestRigs::AddManyRigs(std::string wheel_json,
                                     std::string tire_json,
                                     bool use_airless_tire,
                                     size_t num_rigs) {
    for (size_t i = 0; i < num_rigs; ++i) {
        AddRig(wheel_json, tire_json, use_airless_tire);
    }
}

void ChManyTireTestRigs::AddManyRigs(std::string wheel_json,
                                     std::string tire_json,
                                     bool use_airless_tire,
                                     size_t num_rigs,
                                     const RigConfiguration& config) {
    for (size_t i = 0; i < num_rigs; ++i) {
        auto& rig = AddRig(wheel_json, tire_json, use_airless_tire);
        config.ApplyTo(rig);
    }
}

ChTireTestRig& ChManyTireTestRigs::GetRig(size_t index) {
    if (index >= m_rigs.size()) {
        throw std::out_of_range("ChManyTireTestRigs::GetRig: index out of range");
    }
    return *m_rigs[index];
}

const ChTireTestRig& ChManyTireTestRigs::GetRig(size_t index) const {
    if (index >= m_rigs.size()) {
        throw std::out_of_range("ChManyTireTestRigs::GetRig: index out of range");
    }
    return *m_rigs[index];
}

// -----------------------------------------------------------------------------
// Batch Configuration Methods
// -----------------------------------------------------------------------------

void ChManyTireTestRigs::ApplyConfigurationToAll(const RigConfiguration& config) {
    for (auto& rig : m_rigs) {
        config.ApplyTo(*rig);
    }
}

void ChManyTireTestRigs::ApplyConfigurationToRange(const RigConfiguration& config,
                                                   size_t start_index,
                                                   size_t end_index) {
    if (start_index >= m_rigs.size() || end_index > m_rigs.size() || start_index >= end_index) {
        throw std::out_of_range("ChManyTireTestRigs::ApplyConfigurationToRange: invalid range");
    }
    for (size_t i = start_index; i < end_index; ++i) {
        config.ApplyTo(*m_rigs[i]);
    }
}

// -----------------------------------------------------------------------------
// Common Batch Setters
// -----------------------------------------------------------------------------

void ChManyTireTestRigs::SetGravitationalAcceleration(const ChVector3d& grav) {
    m_grav = grav;
    for (auto& rig : m_rigs) {
        rig->SetGravitationalAcceleration(grav);
    }
}

void ChManyTireTestRigs::SetGravitationalAcceleration(double grav) {
    SetGravitationalAcceleration(ChVector3d(0, 0, -grav));
}

void ChManyTireTestRigs::SetNormalLoadAll(double load) {
    for (auto& rig : m_rigs) {
        rig->SetNormalLoad(load);
    }
}

void ChManyTireTestRigs::SetCamberAngleAll(double camber) {
    for (auto& rig : m_rigs) {
        rig->SetCamberAngle(camber);
    }
}

void ChManyTireTestRigs::SetLongSpeedFunctionAll(std::shared_ptr<ChFunction> funct) {
    for (auto& rig : m_rigs) {
        rig->SetLongSpeedFunction(funct);
    }
}

void ChManyTireTestRigs::SetAngSpeedFunctionAll(std::shared_ptr<ChFunction> funct) {
    for (auto& rig : m_rigs) {
        rig->SetAngSpeedFunction(funct);
    }
}

void ChManyTireTestRigs::SetSlipAngleFunctionAll(std::shared_ptr<ChFunction> funct) {
    for (auto& rig : m_rigs) {
        rig->SetSlipAngleFunction(funct);
    }
}

void ChManyTireTestRigs::SetTireCollisionTypeAll(ChTire::CollisionType coll_type) {
    for (auto& rig : m_rigs) {
        rig->SetTireCollisionType(coll_type);
    }
}

void ChManyTireTestRigs::SetTireStepsizeAll(double step) {
    for (auto& rig : m_rigs) {
        rig->SetTireStepsize(step);
    }
}

void ChManyTireTestRigs::SetTireVisualizationTypeAll(VisualizationType vis) {
    for (auto& rig : m_rigs) {
        rig->SetTireVisualizationType(vis);
    }
}

void ChManyTireTestRigs::SetTerrainRigidAll(const ChTireTestRig::TerrainParamsRigid& params) {
    for (auto& rig : m_rigs) {
        rig->SetTerrainRigid(params);
    }
}

void ChManyTireTestRigs::SetTimeDelayAll(double delay) {
    for (auto& rig : m_rigs) {
        rig->SetTimeDelay(delay);
    }
}

// -----------------------------------------------------------------------------
// Smart Getters
// -----------------------------------------------------------------------------

std::shared_ptr<ChTerrain> ChManyTireTestRigs::GetTerrain(size_t index) const {
    if (index >= m_rigs.size()) {
        throw std::out_of_range("ChManyTireTestRigs::GetTerrain: index out of range");
    }
    return m_rigs[index]->GetTerrain();
}

ChVector3d ChManyTireTestRigs::GetCarrierPos(size_t index) const {
    if (index >= m_rigs.size()) {
        throw std::out_of_range("ChManyTireTestRigs::GetCarrierPos: index out of range");
    }
    return m_rigs[index]->GetPos();
}

double ChManyTireTestRigs::GetMass(size_t index) const {
    if (index >= m_rigs.size()) {
        throw std::out_of_range("ChManyTireTestRigs::GetMass: index out of range");
    }
    return m_rigs[index]->GetMass();
}

// -----------------------------------------------------------------------------
// Simulation Control
// -----------------------------------------------------------------------------

void ChManyTireTestRigs::InitializeAll(ChTireTestRig::Mode mode) {
    for (auto& rig : m_rigs) {
        rig->Initialize(mode);
    }
}

void ChManyTireTestRigs::AdvanceAll(double step) {
    double time = m_system->GetChTime();
    // Can't just call rig->Advance(step) for all rigs because we have a single system
    for (auto& rig : m_rigs) {
        rig->GetTerrain()->Synchronize(time);
        rig->GetTire()->Synchronize(time, *rig->GetTerrain().get());
        rig->GetSpindle()->EmptyTireAccumulator();
        rig->GetWheel()->Synchronize();

        // Advance state
        rig->GetTerrain()->Advance(step);
        rig->GetTire()->Advance(step);
    }

    m_system->DoStepDynamics(step);  // Advance the system
}

}  // end namespace vehicle
}  // end namespace chrono
