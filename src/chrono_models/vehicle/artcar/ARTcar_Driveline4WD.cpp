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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// ARTcar 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/artcar/ARTcar_Driveline4WD.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double ARTcar_Driveline4WD::m_central_differentialbox_inertia = 0.00000797;  // estimate
const double ARTcar_Driveline4WD::m_front_differentialbox_inertia = 0.00000797;    // estimate
const double ARTcar_Driveline4WD::m_rear_differentialbox_inertia = 0.00000797;     // estimate
const double ARTcar_Driveline4WD::m_driveshaft_inertia = 0.000001;                 // estimate
const double ARTcar_Driveline4WD::m_frontshaft_inertia = 0.000001;                 // estimate
const double ARTcar_Driveline4WD::m_rearshaft_inertia = 0.000001;                  // estimate

const double ARTcar_Driveline4WD::m_front_conicalgear_ratio = .5;  // TODO
const double ARTcar_Driveline4WD::m_rear_conicalgear_ratio = .5;   // TODO

const double ARTcar_Driveline4WD::m_axle_differential_locking_limit = 7.5;     // limited-slip differential
const double ARTcar_Driveline4WD::m_central_differential_locking_limit = 7.5;  // limited-slip central diff

// -----------------------------------------------------------------------------
// Constructor of the ARTcar_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
ARTcar_Driveline4WD::ARTcar_Driveline4WD(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}

// -----------------------------------------------------------------------------
// Override Initialize to enable limited-slip differentials by default
// -----------------------------------------------------------------------------
void ARTcar_Driveline4WD::Initialize(std::shared_ptr<ChChassis> chassis,
                                     const ChAxleList& axles,
                                     const std::vector<int>& driven_axles) {
    // First, call the base class Initialize to set up all the shafts and clutches
    ChShaftsDriveline4WD::Initialize(chassis, axles, driven_axles);

    // Enable limited-slip differentials by setting modulation to 1 (full engagement)
    // The 100 N·m torque limits will prevent complete locking while providing
    // resistance to excessive wheel spin
    LockAxleDifferential(-1, true);    // Enable for all axles
    LockCentralDifferential(0, true);  // Enable central differential
}

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono
