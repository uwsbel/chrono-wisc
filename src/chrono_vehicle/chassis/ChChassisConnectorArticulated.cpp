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
// Template for an articulation chassis connector.  This is an active connector,
// modeled with a rotational motor (a revolute joint along the chassis vertical
// axis whose DOF can be actuated based on current steering input).
//
// =============================================================================

#include "chrono_vehicle/chassis/ChChassisConnectorArticulated.h"

namespace chrono {
namespace vehicle {

ChChassisConnectorArticulated::ChChassisConnectorArticulated(const std::string& name) : ChChassisConnector(name) {}

ChChassisConnectorArticulated::~ChChassisConnectorArticulated() {
    if (!m_initialized)
        return;

    auto sys = m_motor->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_motor);
}

void ChChassisConnectorArticulated::Initialize(std::shared_ptr<ChChassis> front, std::shared_ptr<ChChassisRear> rear) {
    ChChassisConnector::Initialize(front, rear);

    // Express the connector reference frame in the absolute coordinate system
    ChFrame<> to_abs(rear->GetLocalPosFrontConnector());
    to_abs.ConcatenatePreTransformation(rear->GetBody()->GetFrameRefToAbs());

    // Create the connection
    m_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_motor->SetName(m_name + " motor");
    m_motor->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>());
    m_motor->Initialize(rear->GetBody(), front->GetBody(), to_abs);
    rear->GetBody()->GetSystem()->AddLink(m_motor);
}

void ChChassisConnectorArticulated::Synchronize(double time, const DriverInputs& driver_inputs) {
    auto fun = std::static_pointer_cast<ChFunctionConst>(m_motor->GetAngleFunction());
    fun->SetConstant(-GetMaxSteeringAngle() * driver_inputs.m_steering);
}

}  // end namespace vehicle
}  // end namespace chrono
