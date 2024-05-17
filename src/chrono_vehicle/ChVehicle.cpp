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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a vehicle model.
//
// =============================================================================

#include <algorithm>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/ChVehicle.h"

#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_vehicle/output/ChVehicleOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono_vehicle/output/ChVehicleOutputHDF5.h"
#endif

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

// Constructor for a ChVehicle using a default Chrono system.
// Specify default step size and solver parameters.
ChVehicle::ChVehicle(const std::string& name, ChContactMethod contact_method)
    : m_name(name),
      m_ownsSystem(true),
      m_output(false),
      m_output_step(0),
      m_output_db(nullptr),
      m_next_output_time(0),
      m_output_frame(0),
      m_mass(0),
      m_inertia(0),
      m_realtime_force(false),
      m_RTF(0),
      m_initialized(false) {
    m_system = (contact_method == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                        : static_cast<ChSystem*>(new ChSystemSMC);

    m_system->SetGravitationalAcceleration(-9.81 * ChWorldFrame::Vertical());

    // Set default solver for vehicle simulations
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    m_system->GetSolver()->AsIterative()->SetMaxIterations(150);
}

// Constructor for a ChVehicle using the specified Chrono ChSystem.
ChVehicle::ChVehicle(const std::string& name, ChSystem* system)
    : m_name(name),
      m_system(system),
      m_ownsSystem(false),
      m_output(false),
      m_output_step(0),
      m_output_db(nullptr),
      m_next_output_time(0),
      m_output_frame(0),
      m_mass(0),
      m_inertia(0),
      m_realtime_force(false),
      m_RTF(0),
      m_initialized(false) {}

ChVehicle::~ChVehicle() {
    delete m_output_db;
    if (m_ownsSystem)
        delete m_system;
}

// -----------------------------------------------------------------------------

void ChVehicle::SetCollisionSystemType(ChCollisionSystem::Type collsys_type) {
    if (m_ownsSystem)
        m_system->SetCollisionSystemType(collsys_type);
}

void ChVehicle::EnableRealtime(bool val) {
    m_realtime_force = val;
    m_realtime_timer.stop();
    if (m_realtime_force) {
        // ensure the embedded timer is restarted when this function is called
        m_realtime_timer.reset();
        m_realtime_timer.start();
    }
}

// -----------------------------------------------------------------------------
// Enable output for this vehicle system.
// -----------------------------------------------------------------------------

void ChVehicle::SetOutput(ChVehicleOutput::Type type,
                          const std::string& out_dir,
                          const std::string& out_name,
                          double output_step) {
    m_output = true;
    m_output_step = output_step;

    switch (type) {
        case ChVehicleOutput::ASCII:
            m_output_db = new ChVehicleOutputASCII(out_dir + "/" + out_name + ".txt");
            break;
        case ChVehicleOutput::JSON:
            //// TODO
            break;
        case ChVehicleOutput::HDF5:
#ifdef CHRONO_HAS_HDF5
            m_output_db = new ChVehicleOutputHDF5(out_dir + "/" + out_name + ".h5");
#endif
            break;
    }
}

void ChVehicle::SetOutput(ChVehicleOutput::Type type, std::ostream& out_stream, double output_step) {
    m_output = true;
    m_output_step = output_step;

    switch (type) {
        case ChVehicleOutput::ASCII:
            m_output_db = new ChVehicleOutputASCII(out_stream);
            break;
        case ChVehicleOutput::JSON:
            //// TODO
            break;
        case ChVehicleOutput::HDF5:
#ifdef CHRONO_HAS_HDF5
            //// TODO
#endif
            break;
    }
}

// -----------------------------------------------------------------------------

void ChVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Calculate total vehicle mass and inertia properties at initial configuration
    InitializeInertiaProperties();
    UpdateInertiaProperties();
    m_initialized = true;
}

void ChVehicle::InitializePowertrain(std::shared_ptr<ChPowertrainAssembly> powertrain) {
    m_powertrain_assembly = powertrain;
    m_powertrain_assembly->Initialize(m_chassis);
}

// -----------------------------------------------------------------------------
// Advance the state of the system.
// -----------------------------------------------------------------------------

void ChVehicle::Advance(double step) {
    // Ensure the vehicle mass includes the mass of subsystems that may have been initialized after the vehicle
    if (!m_initialized) {
        InitializeInertiaProperties();
        m_initialized = true;
    }

    if (m_output && m_system->GetChTime() >= m_next_output_time) {
        Output(m_output_frame, *m_output_db);
        m_next_output_time += m_output_step;
        m_output_frame++;
    }

    if (m_ownsSystem) {
        m_system->DoStepDynamics(step);
    }

    // Update inertia properties
    UpdateInertiaProperties();

    m_sim_timer.stop();
    m_RTF = m_sim_timer() / step;
    if (m_realtime_force) {
        m_realtime_timer.Spin(step);
    }
    m_sim_timer.reset();
    m_sim_timer.start();
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChEngine> ChVehicle::GetEngine() const {
    return m_powertrain_assembly ? m_powertrain_assembly->GetEngine() : nullptr;
}

std::shared_ptr<ChTransmission> ChVehicle::GetTransmission() const {
    return m_powertrain_assembly ? m_powertrain_assembly->GetTransmission() : nullptr;
}

// -----------------------------------------------------------------------------

void ChVehicle::SetChassisVisualizationType(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void ChVehicle::SetChassisRearVisualizationType(VisualizationType vis) {
    for (auto& c : m_chassis_rear)
        c->SetVisualizationType(vis);
}

void ChVehicle::SetChassisCollide(bool state) {
    m_chassis->EnableCollision(state);
    for (auto& c : m_chassis_rear)
        c->EnableCollision(state);
}

void ChVehicle::SetChassisOutput(bool state) {
    m_chassis->SetOutput(state);
    for (auto& c : m_chassis_rear)
        c->SetOutput(state);
}

}  // end namespace vehicle
}  // end namespace chrono
