// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Jayne Henry
// =============================================================================
//
// Wrapper classes for modeling an entire ARTcar vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef ARTCAR_H
#define ARTCAR_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/artcar/ARTcar_Vehicle.h"
#include "chrono_models/vehicle/artcar/ARTcar_RigidTire.h"
#include "chrono_models/vehicle/artcar/ARTcar_TMeasyTire.h"
#include "chrono_models/vehicle/artcar/ARTcar_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/artcar/ARTcar_EngineSimpleMap.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// Definition of the ARTcar assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// a RC car, the powertrain model, and the 4 tires. It provides wrappers to access the different
/// systems and subsystems, functions for specifying the tire types, as well as functions for
/// controlling the visualization mode of each component.
class CH_MODELS_API ARTcar {
  public:
    ARTcar();
    ARTcar(ChSystem* system);

    ~ARTcar();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }
    void SetCollisionSystemType(ChCollisionSystem::Type collsys_type) { m_collsysType = collsys_type; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

    void SetTireType(TireModelType val) { m_tireType = val; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }

    void Initialize();

    void LockAxleDifferential(int axle, bool lock) { m_vehicle->LockAxleDifferential(axle, lock); }

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
    void SetSteeringVisualizationType(VisualizationType vis) { m_vehicle->SetSteeringVisualizationType(vis); }
    void SetWheelVisualizationType(VisualizationType vis) { m_vehicle->SetWheelVisualizationType(vis); }
    void SetTireVisualizationType(VisualizationType vis) { m_vehicle->SetTireVisualizationType(vis); }

    /// Set parameters for tuning engine map -> Currently needs to be called before initialize
    void SetMaxMotorVoltageRatio(double voltage_ratio) { m_voltage_ratio = voltage_ratio; }

    /// Set stall torque -> Currently needs to be called before initialize
    void SetStallTorque(double stall_torque) { m_stall_torque = stall_torque; }

    /// Set tire rolling friction coefficient -> Currently needs to be called before initialize
    void SetTireRollingResistance(double rolling_resistance) { m_rolling_friction_coeff = rolling_resistance; }

    void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain);
    void Advance(double step);

    void LogHardpointLocations() { m_vehicle->LogHardpointLocations(); }
    void DebugLog(int what) { m_vehicle->DebugLog(what); }

  protected:
    ChContactMethod m_contactMethod;
    ChCollisionSystem::Type m_collsysType;
    CollisionType m_chassisCollisionType;
    bool m_fixed;

    TireModelType m_tireType;

    double m_tire_step_size;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    ARTcar_Vehicle* m_vehicle;

    double m_tire_mass;

    double m_stall_torque;
    double m_voltage_ratio;
    double m_rolling_friction_coeff;
};

/// @} vehicle_models_artcar

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono

#endif
