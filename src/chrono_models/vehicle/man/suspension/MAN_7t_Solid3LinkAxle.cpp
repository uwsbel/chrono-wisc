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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MAN 7t (rear) driven solid three link axle.
//
// =============================================================================

#include "chrono_models/vehicle/man/suspension/MAN_7t_Solid3LinkAxle.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_7t_Solid3LinkAxle::m_axleTubeMass = 709;
const double MAN_7t_Solid3LinkAxle::m_spindleMass = 14.705 * 4.1;
const double MAN_7t_Solid3LinkAxle::m_triangleMass = 50.0;
const double MAN_7t_Solid3LinkAxle::m_linkMass = 25.0;

const double MAN_7t_Solid3LinkAxle::m_axleTubeRadius = 0.0476;
const double MAN_7t_Solid3LinkAxle::m_spindleRadius = 0.10;
const double MAN_7t_Solid3LinkAxle::m_spindleWidth = 0.06;

const ChVector3d MAN_7t_Solid3LinkAxle::m_axleTubeInertia(329.00, 16.46, 330.00);
const ChVector3d MAN_7t_Solid3LinkAxle::m_spindleInertia(0.04117 * 6.56, 0.07352 * 6.56, 0.04117 * 6.56);
const ChVector3d MAN_7t_Solid3LinkAxle::m_triangleInertia(0.2, 0.2, 0.2);
const ChVector3d MAN_7t_Solid3LinkAxle::m_linkInertia(0.05, 0.1, 0.1);

const double MAN_7t_Solid3LinkAxle::m_springDesignLength = 0.499924994;
const double MAN_7t_Solid3LinkAxle::m_springCoefficient1 = 178892.0;  // linear
const double MAN_7t_Solid3LinkAxle::m_springCoefficient2 = 621618.0;  // quadratic
const double MAN_7t_Solid3LinkAxle::m_springRestLength = 0.625;
const double MAN_7t_Solid3LinkAxle::m_springMinLength = 0.380;
const double MAN_7t_Solid3LinkAxle::m_springMaxLength = m_springDesignLength + 0.15;
const double MAN_7t_Solid3LinkAxle::m_damperCoefExpansion = 98727.9;
const double MAN_7t_Solid3LinkAxle::m_damperDegresExpansion = 4.77954;
const double MAN_7t_Solid3LinkAxle::m_damperCoefCompression = 52526.6;
const double MAN_7t_Solid3LinkAxle::m_damperDegresCompression = 3.0;
const double MAN_7t_Solid3LinkAxle::m_axleShaftInertia = 0.4 * 6.56;

const double MAN_7t_Solid3LinkAxle::m_twin_tire_dist = 0.0;

// ---------------------------------------------------------------------------------------
// MAN spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class MAN_7t_SpringForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    MAN_7t_SpringForceRear(double spring_constant1, double spring_coefficient2, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant1;
    double m_spring_constant2;
    double m_min_length;
    double m_max_length;
    double m_scale;

    ChFunctionInterp m_bump;
};

MAN_7t_SpringForceRear::MAN_7t_SpringForceRear(double spring_constant1,
                                               double spring_constant2,
                                               double min_length,
                                               double max_length)
    : m_spring_constant1(spring_constant1),
      m_spring_constant2(spring_constant2),
      m_min_length(min_length),
      m_max_length(max_length),
      m_scale(0.749) {
    // From ADAMS/Car
    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3, 200.0);
    m_bump.AddPoint(4.0e-3, 400.0);
    m_bump.AddPoint(6.0e-3, 600.0);
    m_bump.AddPoint(8.0e-3, 800.0);
    m_bump.AddPoint(10.0e-3, 1000.0);
    m_bump.AddPoint(20.0e-3, 2500.0);
    m_bump.AddPoint(30.0e-3, 4500.0);
    m_bump.AddPoint(40.0e-3, 7500.0);
    m_bump.AddPoint(50.0e-3, 12500.0);
}

double MAN_7t_SpringForceRear::evaluate(double time,
                                        double rest_length,
                                        double length,
                                        double vel,
                                        const ChLinkTSDA& link) {
    double force = 0;

    double defl_spring = rest_length - length;
    double defl_bump = 0.0;
    double defl_rebound = 0.0;

    if (length < m_min_length) {
        defl_bump = m_min_length - length;
    }

    if (length > m_max_length) {
        defl_rebound = length - m_max_length;
    }

    force = m_scale * (defl_spring * m_spring_constant1 + defl_spring * std::abs(defl_spring) * m_spring_constant2) +
            m_bump.GetVal(defl_bump) - m_bump.GetVal(defl_rebound);

    return force;
}

MAN_7t_Solid3LinkAxle::MAN_7t_Solid3LinkAxle(const std::string& name) : ChSolidThreeLinkAxle(name) {
    m_springForceCB = chrono_types::make_shared<MAN_7t_SpringForceRear>(m_springCoefficient1, m_springCoefficient2,
                                                                        m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<utils::DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
MAN_7t_Solid3LinkAxle::~MAN_7t_Solid3LinkAxle() {}

const ChVector3d MAN_7t_Solid3LinkAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector3d(0.000, 0.655, 0.090);
        case SPRING_C:
            return ChVector3d(0.000, 0.585, 0.585);
        case SHOCK_A:
            return ChVector3d(-0.246, 0.688, -0.125);
        case SHOCK_C:
            return ChVector3d(-0.250, 0.562, 0.570);
        case SPINDLE:
            return ChVector3d(0.0, 2.07 / 2.0, 0.0);
        case TRIANGLE_A:
            return ChVector3d(0.000, 0.000, 0.260);
        case TRIANGLE_C:
            return ChVector3d(-0.762, 0.420, 0.100);
        case LINK_A:
            return ChVector3d(0.115, 0.688, -0.090);
        case LINK_C:
            return ChVector3d(1.139, 0.400, 0.100);
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
