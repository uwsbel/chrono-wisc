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
// ARTcar 4WD simple driveline model based on ChSimpleDriveline.
//
// =============================================================================

#include <cmath>

#include "chrono_models/vehicle/artcar/ARTcar_Driveline4WD_simple.h"

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double ARTcar_Driveline4WD_simple::m_front_torque_frac = 0.5;
const double ARTcar_Driveline4WD_simple::m_front_diff_bias = 2.0;
const double ARTcar_Driveline4WD_simple::m_rear_diff_bias = 2.0;
const double ARTcar_Driveline4WD_simple::m_front_conicalgear_ratio = 0.5;
const double ARTcar_Driveline4WD_simple::m_rear_conicalgear_ratio = 0.5;

// -----------------------------------------------------------------------------

ARTcar_Driveline4WD_simple::ARTcar_Driveline4WD_simple(const std::string& name) : ChSimpleDriveline(name) {}

}  // end namespace artcar
}  // end namespace vehicle
}  // end namespace chrono
