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

#ifndef ARTCAR_DRIVELINE_4WD_SIMPLE_H
#define ARTCAR_DRIVELINE_4WD_SIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// Simple 4WD driveline model for the ARTcar vehicle (purely kinematic).
class CH_MODELS_API ARTcar_Driveline4WD_simple : public ChSimpleDriveline {
  public:
    ARTcar_Driveline4WD_simple(const std::string& name);

    ~ARTcar_Driveline4WD_simple() {}

    virtual double GetFrontTorqueFraction() const override { return m_front_torque_frac; }
    virtual double GetFrontDifferentialMaxBias() const override { return m_front_diff_bias; }
    virtual double GetRearDifferentialMaxBias() const override { return m_rear_diff_bias; }

    virtual double GetFrontConicalGearRatio() const override { return m_front_conicalgear_ratio; }
    virtual double GetRearConicalGearRatio() const override { return m_rear_conicalgear_ratio; }

  private:
    static const double m_front_torque_frac;
    static const double m_front_diff_bias;
    static const double m_rear_diff_bias;
    static const double m_front_conicalgear_ratio;
    static const double m_rear_conicalgear_ratio;
};

/// @} vehicle_models_artcar

}  // end namespace artcar
}  // end namespace vehicle
}  // end namespace chrono

#endif
