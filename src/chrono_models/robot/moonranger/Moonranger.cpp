// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Luning Bakke, Huzaifa Unjhawala
// =============================================================================

#include <cmath>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/functions/ChFunctionSetpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_models/robot/moonranger/Moonranger.h"

namespace chrono {
namespace moonranger {

// =============================================================================

const double Moonranger::m_max_steer_angle = CH_PI / 6;
// initilize rover wheels
const double wheel_x = 0.2222;
const double wheel_y = 0.29207;
const double wheel_z = -0.1805;
const double chassis_dim_x = 0.25;
const double chassis_dim_y = 0.175;
const double chassis_dim_z = 0.14;

// =============================================================================

// Default contact material for rover parts
std::shared_ptr<ChContactMaterial> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

// Add a rotational speed motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotorSpeed(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<MoonrangerChassis> chassis,
                                                        const ChVector3d& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational angle motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationAngle> AddMotorAngle(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<MoonrangerChassis> chassis,
                                                        const ChVector3d& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational torque motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationTorque> AddMotorTorque(std::shared_ptr<ChBody> body1,
                                                          std::shared_ptr<ChBody> body2,
                                                          std::shared_ptr<MoonrangerChassis> chassis,
                                                          const ChVector3d& rel_pos,
                                                          const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Base class for all Moonranger Part
MoonrangerPart::MoonrangerPart(const std::string& name,
                               const ChFrame<>& rel_pos,
                               std::shared_ptr<ChContactMaterial> mat,
                               bool collide)
    : m_name(name), m_pos(rel_pos), m_mat(mat), m_collide(collide), m_visualize(true) {}

void MoonrangerPart::Construct(ChSystem* system) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetName(m_name + "_body");
    m_body->SetMass(m_mass);
    m_body->SetInertiaXX(m_inertia);
    m_body->SetFrameCOMToRef(m_cog);

    // Add visualization shape
    if (m_visualize) {
        auto vis_mesh_file = GetChronoDataFile("robot/moonranger/obj/" + m_mesh_name + ".obj");
        auto trimesh_vis = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_vis->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);
        m_body->AddVisualShape(trimesh_shape);
    }

    // Add collision shape
    if (m_collide) {
        auto col_mesh_file = GetChronoDataFile("robot/moonranger/col/" + m_mesh_name + ".obj");
        auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(col_mesh_file, false, false);
        trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_col->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

        auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh_col, false, false, 0.005);
        m_body->AddCollisionShape(shape);
        m_body->EnableCollision(m_collide);
    }

    system->AddBody(m_body);
}

void MoonrangerPart::CalcMassProperties(double density) {
    auto mesh_filename = GetChronoDataFile("robot/moonranger/col/" + m_mesh_name + ".obj");
    auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

    double vol;
    ChVector3d cog_pos;
    ChMatrix33<> cog_rot;
    ChMatrix33<> inertia;
    trimesh_col->ComputeMassProperties(true, vol, cog_pos, inertia);
    ChInertiaUtils::PrincipalInertia(inertia, m_inertia, cog_rot);
    m_mass = density * vol;
    m_inertia *= density;
    m_cog = ChFrame<>(cog_pos, cog_rot);
}

void MoonrangerPart::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrameRefToAbs() * m_pos;
    m_body->SetFrameRefToAbs(X_GC);
}

// =============================================================================

MoonrangerChassis::MoonrangerChassis(const std::string& name, std::shared_ptr<ChContactMaterial> mat)
    : MoonrangerPart(name, ChFrame<>(VNULL, QUNIT), mat, false) {
    m_mesh_name = "moonranger_chassis";
    m_color = ChColor(1.0f, 1.0f, 1.0f);

    m_mass = 17.048;  // weight of the chassis
    // Compute inertial from a cube with dimensions 0.25 x 0.175 x 0.14
    double cube_length = 0.25;
    double cube_width = 0.175;
    double cube_height = 0.14;
    double cube_mass = m_mass;
    double cube_density = cube_mass / (cube_length * cube_width * cube_height);
    double cube_IXX = cube_mass * (cube_width * cube_width + cube_height * cube_height) / 12;
    double cube_IYY = cube_mass * (cube_length * cube_length + cube_height * cube_height) / 12;
    double cube_IZZ = cube_mass * (cube_length * cube_length + cube_width * cube_width) / 12;
    m_inertia = ChVector3d(cube_IXX, cube_IYY, cube_IZZ);

    m_visualize = false;
    m_collide = false;
}

void MoonrangerChassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    m_body->SetFrameRefToAbs(pos);
}

// =============================================================================

// Moonranger Wheel
MoonrangerWheel::MoonrangerWheel(const std::string& name,
                                 const ChFrame<>& rel_pos,
                                 std::shared_ptr<ChContactMaterial> mat,
                                 MoonrangerWheelType wheel_type)
    : MoonrangerPart(name, rel_pos, mat, true) {
    switch (wheel_type) {
        case MoonrangerWheelType::RealWheel:
            m_mesh_name = "moonranger_wheel";
            break;
        case MoonrangerWheelType::SimpleWheel:
            m_mesh_name = "moonranger_wheel";
            break;
        case MoonrangerWheelType::CylWheel:
            m_mesh_name = "moonranger_wheel";
            break;
    }

    m_color = ChColor(0.4f, 0.7f, 0.4f);
    m_mass = 0.238;  // weight of the wheel
    double wheel_width = 0.06;
    double wheel_radius = 0.085;
    double wheel_IYY = m_mass * wheel_radius * wheel_radius / 2.0;
    double wheel_IXX = m_mass / 12 * (3 * wheel_radius * wheel_radius + wheel_width * wheel_width);

    double internal_radius = 0.0835;
    double external_radius = 0.0875;  // Some addition to account for the grousers
    double wheel_IZZ = m_mass / 2 * (external_radius * external_radius + internal_radius * internal_radius);

    m_inertia = ChVector3d(wheel_IXX, wheel_IYY, wheel_IZZ);  // principal inertia

    ChMatrix33<> A;
    A.setZero();
    A(0, 0) = -0.243;
    A(2, 0) = 0.97;
    A(0, 1) = 0.97;
    A(2, 1) = 0.243;
    A(1, 2) = 1;

    m_cog = ChFrame<>(ChVector3d(0, 0.00426, 0), A.GetQuaternion());
}

// =============================================================================

// Rover model
Moonranger::Moonranger(ChSystem* system, MoonrangerWheelType wheel_type) : m_system(system), m_chassis_fixed(false) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == ChContactMethod::NSC) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    m_default_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);

    Create(wheel_type);
}

void Moonranger::Create(MoonrangerWheelType wheel_type) {
    // create rover chassis
    m_chassis = chrono_types::make_shared<MoonrangerChassis>("chassis", m_default_material);

    m_wheels[LF] = chrono_types::make_shared<MoonrangerWheel>(
        "wheel_LF", ChFrame<>(ChVector3d(+wheel_x, +wheel_y, wheel_z), QUNIT), m_wheel_material, wheel_type);
    m_wheels[RF] = chrono_types::make_shared<MoonrangerWheel>(
        "wheel_RF", ChFrame<>(ChVector3d(+wheel_x, -wheel_y, wheel_z), QUNIT), m_wheel_material, wheel_type);
    m_wheels[LB] = chrono_types::make_shared<MoonrangerWheel>(
        "wheel_LB", ChFrame<>(ChVector3d(-wheel_x, +wheel_y, wheel_z), QUNIT), m_wheel_material, wheel_type);
    m_wheels[RB] = chrono_types::make_shared<MoonrangerWheel>(
        "wheel_RB", ChFrame<>(ChVector3d(-wheel_x, -wheel_y, wheel_z), QUNIT), m_wheel_material, wheel_type);

    m_wheels[RF]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));
    m_wheels[RB]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));
}

void Moonranger::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    m_chassis->Initialize(m_system, pos);
    m_chassis->GetBody()->SetFixed(m_chassis_fixed);

    for (int i = 0; i < 4; i++) {
        m_wheels[i]->Initialize(m_chassis->GetBody());
    }

    ChVector3d wheel_rel_pos[] = {
        ChVector3d(+wheel_x, +wheel_y, wheel_z),  // LF
        ChVector3d(+wheel_x, -wheel_y, wheel_z),  // RF
        ChVector3d(-wheel_x, +wheel_y, wheel_z),  // LB
        ChVector3d(-wheel_x, -wheel_y, wheel_z)   // RB
    };

    ChQuaternion<> z2x = QuatFromAngleY(CH_PI_2);

    for (int i = 0; i < 4; i++) {
        ChQuaternion<> z2y;
        z2y.SetFromAngleX(CH_PI_2);

        m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunctionSetpoint>();
        m_drive_motors[i] =
            AddMotorSpeed(m_chassis->GetBody(), m_wheels[i]->GetBody(), m_chassis, wheel_rel_pos[i], z2y);
        m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
    }
}

void Moonranger::SetWheelContactMaterial(std::shared_ptr<ChContactMaterial> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void Moonranger::SetChassisFixed(bool fixed) {
    m_chassis_fixed = fixed;
}

void Moonranger::SetChassisVisualization(bool state) {
    m_chassis->SetVisualize(state);
}

void Moonranger::SetWheelVisualization(bool state) {
    for (auto& wheel : m_wheels)
        wheel->SetVisualize(state);
}

ChVector3d Moonranger::GetWheelContactForce(MoonrangerWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector3d Moonranger::GetWheelContactTorque(MoonrangerWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector3d Moonranger::GetWheelAppliedForce(MoonrangerWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector3d Moonranger::GetWheelAppliedTorque(MoonrangerWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double Moonranger::GetWheelTracTorque(MoonrangerWheelID id) const {
    return m_drive_motors[id]->GetMotorTorque();
}

double Moonranger::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass();
    for (int i = 0; i < 4; i++) {
        tot_mass += m_wheels[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double Moonranger::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

void Moonranger::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    for (int i = 0; i < 4; i++) {
        // Extract driver inputs
        double driving = m_driver->drive_speeds[i];
        m_drive_motor_funcs[i]->SetSetpoint(driving, time);
    }
}
// =============================================================================

MoonrangerSpeedDriver::MoonrangerSpeedDriver(double time_ramp, double speed, double no_spin_time)
    : m_ramp(time_ramp), m_speed(speed), m_no_spin_time(no_spin_time) {}

void MoonrangerSpeedDriver::Update(double time) {
    double speed = m_speed;
    if (time < m_no_spin_time)
        speed = 0;
    else if (time < m_ramp)
        speed = m_speed * (time / m_ramp);
    drive_speeds = {speed, speed, speed, speed};
}

}  // namespace moonranger
}  // namespace chrono