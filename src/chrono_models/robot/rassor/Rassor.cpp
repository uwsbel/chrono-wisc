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
// Authors: Json Zhou
// =============================================================================
//
// NASA RASSOR Mining Experimental Rover Model Class.
// This class contains model for an experimental rover Rassor
// Reference page: https://technology.nasa.gov/patent/KSC-TOPS-7
//
// =============================================================================
#include <cmath>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/functions/ChFunctionSetpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_models/robot/rassor/Rassor.h"

namespace chrono {
namespace rassor {

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
                                                        std::shared_ptr<RassorChassis> chassis,
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
                                                        std::shared_ptr<RassorChassis> chassis,
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
                                                          std::shared_ptr<RassorChassis> chassis,
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

// =============================================================================

// Base class for all Rassor Part
RassorPart::RassorPart(const std::string& name,
                       const ChFrame<>& rel_pos,
                       std::shared_ptr<ChContactMaterial> mat,
                       bool collide)
    : m_name(name), m_pos(rel_pos), m_mat(mat), m_collide(collide), m_visualize(true) {}

void RassorPart::Construct(ChSystem* system) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetName(m_name + "_body");
    m_body->SetMass(m_mass);
    m_body->SetInertiaXX(m_inertia);
    m_body->SetFrameCOMToRef(m_cog);

    // Add visualization shape
    if (m_visualize) {
        auto vis_mesh_file = GetChronoDataFile("robot/rassor/obj/" + m_mesh_name + ".obj");
        auto trimesh_vis = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_vis->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetColor(m_color);
        m_body->AddVisualShape(trimesh_shape);
    }

    // Add collision shape
    auto col_mesh_file = GetChronoDataFile("robot/rassor/obj/" + m_mesh_name + ".obj");
    auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(col_mesh_file, false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

    auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh_col, false, false, 0.005);
    m_body->AddCollisionShape(shape);
    m_body->EnableCollision(m_collide);

    system->AddBody(m_body);
}

void RassorPart::CalcMassProperties(double density) {
    auto mesh_filename = GetChronoDataFile("robot/rassor/obj/" + m_mesh_name + ".obj");
    auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, true, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

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

void RassorPart::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrameRefToAbs() * m_pos;
    m_body->SetFrameRefToAbs(X_GC);
}

// =============================================================================

// Rover Chassis
RassorChassis::RassorChassis(const std::string& name, std::shared_ptr<ChContactMaterial> mat)
    : RassorPart(name, ChFrame<>(VNULL, QUNIT), mat, false) {
    m_mesh_name = "rassor_chassis";
    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(1650);
}

void RassorChassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    m_body->SetFrameRefToAbs(pos);
}

// =============================================================================

// Rassor Wheel
RassorWheel::RassorWheel(const std::string& name,
                         const ChFrame<>& rel_pos,
                         std::shared_ptr<ChContactMaterial> mat,
                         RassorWheelType wheel_type)
    : RassorPart(name, rel_pos, mat, true) {
    switch (wheel_type) {
        case RassorWheelType::RealWheel:
            m_mesh_name = "rassor_wheel";
            m_wheel_type = RassorWheelType::RealWheel;
            break;
    }

    m_color = ChColor(0.4f, 0.7f, 0.4f);
    CalcMassProperties(1200);
}

// =============================================================================

// Rassor Drum
RassorDrum::RassorDrum(const std::string& name, const ChFrame<>& rel_pos, std::shared_ptr<ChContactMaterial> mat)
    : RassorPart(name, rel_pos, mat, true) {
    m_mesh_name = "rassor_razor";
    m_color = ChColor(0.1f, 0.6f, 0.8f);
    CalcMassProperties(1200);
}
// =============================================================================

// Rassor Razor
RassorArm::RassorArm(const std::string& name, const ChFrame<>& rel_pos, std::shared_ptr<ChContactMaterial> mat)
    : RassorPart(name, rel_pos, mat, false) {
    m_mesh_name = "rassor_arm";
    m_color = ChColor(0.6f, 0.6f, 0.5f);
    CalcMassProperties(1200);
}
// =============================================================================

// Rover model
Rassor::Rassor(ChSystem* system, RassorWheelType wheel_type) : m_system(system), m_chassis_fixed(false) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();

    // Create the contact materials
    m_default_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);

    Create(wheel_type);
}

void Rassor::Create(RassorWheelType wheel_type) {
    // create rover chassis
    m_chassis = chrono_types::make_shared<RassorChassis>("chassis", m_default_material);

    // initialize rover wheels
    double wx = 0.23961;
    double wy = 0.24394;
    double wz = -0.00136;

    m_wheels[RA_LF] = chrono_types::make_shared<RassorWheel>("Wheel_LF", ChFrame<>(ChVector3d(+wx, +wy, wz), QUNIT),
                                                             m_wheel_material, wheel_type);
    m_wheels[RA_RF] = chrono_types::make_shared<RassorWheel>("Wheel_RF", ChFrame<>(ChVector3d(+wx, -wy, wz), QUNIT),
                                                             m_wheel_material, wheel_type);
    m_wheels[RA_LB] = chrono_types::make_shared<RassorWheel>("Wheel_LB", ChFrame<>(ChVector3d(-wx, +wy, wz), QUNIT),
                                                             m_wheel_material, wheel_type);
    m_wheels[RA_RB] = chrono_types::make_shared<RassorWheel>("Wheel_RB", ChFrame<>(ChVector3d(-wx, -wy, wz), QUNIT),
                                                             m_wheel_material, wheel_type);

    m_wheels[RA_RF]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));
    m_wheels[RA_RB]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));

    // initialize rover razors
    double rx = 0.71883;
    double ry = 0.0;
    double rz = -0.00136;

    ChQuaternion<> z2z180;
    z2z180.SetFromAngleAxis(CH_PI, ChVector3d(0, 0, 1));
    

    m_drums[0] =
        chrono_types::make_shared<RassorDrum>("razor_F", ChFrame<>(ChVector3d(+rx, ry, rz), QUNIT), m_wheel_material);

    m_drums[1] =
        chrono_types::make_shared<RassorDrum>("razor_B", ChFrame<>(ChVector3d(-rx, ry, rz), QUNIT), m_wheel_material);

    m_drums[0]->m_mesh_name = "drum_cw";
    m_drums[1]->m_mesh_name = "drum_ccw";


    // initialize rover arms

    double ax = 0.25;
    double ay = 0.0;
    double az = 0.0;
    ChQuaternion<> y2y180;
    y2y180.SetFromAngleAxis(CH_PI, ChVector3d(0, 1, 0));
    m_arms[0] =
        chrono_types::make_shared<RassorArm>("arm_F", ChFrame<>(ChVector3d(+ax, ay, az), QUNIT), m_wheel_material);

    m_arms[1] =
        chrono_types::make_shared<RassorArm>("arm_B", ChFrame<>(ChVector3d(-ax, ay, az), y2y180), m_wheel_material);
}

void Rassor::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    m_chassis->Initialize(m_system, pos);
    m_chassis->GetBody()->SetFixed(m_chassis_fixed);

    for (int i = 0; i < 4; i++) {
        m_wheels[i]->Initialize(m_chassis->GetBody());
    }

    for (int i = 0; i < 2; i++) {
        m_drums[i]->Initialize(m_chassis->GetBody());
        m_arms[i]->Initialize(m_chassis->GetBody());
    }

    // initialize drive motors
    double wx = 0.23961;
    double wy = 0.24394;
    double wz = -0.00136;

    std::vector<ChVector3d> drive_motor_rel_pos = {ChVector3d(+wx, +wy, wz), ChVector3d(+wx, -wy, wz),
                                                   ChVector3d(-wx, +wy, wz), ChVector3d(-wx, -wy, wz)};

    ChQuaternion<> z2y;
    z2y.SetFromAngleAxis(CH_PI / 2, ChVector3d(1, 0, 0));
    for (int i = 0; i < 4; i++) {
        m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.5);
        m_drive_motors[i] =
            AddMotorSpeed(m_chassis->GetBody(), m_wheels[i]->GetBody(), m_chassis, drive_motor_rel_pos[i], z2y);
        m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
    }

    // initialize motor at the shoulder joint
    double ax = 0.25;
    double ay = 0.0;
    double az = 0.0;
    std::vector<ChVector3d> arm_motor_rel_pos = {ChVector3d(+ax, ay, az), ChVector3d(-ax, ay, az)};
    for (int i = 0; i < 2; i++) {
        if (i == 0)
            m_shoulder_joint_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(-0.25);
        if (i == 1)
            m_shoulder_joint_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.25);
        m_shoulder_joint_motors[i] =
            AddMotorAngle(m_chassis->GetBody(), m_arms[i]->GetBody(), m_chassis, arm_motor_rel_pos[i], z2y);
        m_shoulder_joint_motors[i]->SetMotorFunction(m_shoulder_joint_motor_funcs[i]);
    }

    // intialize the speed motor of the drum
    double rx = 0.71883;
    double ry = 0.0;
    double rz = -0.00136;
    std::vector<ChVector3d> arm_motor_rel_pos_2 = {ChVector3d(+rx, ry, rz), ChVector3d(-rx, ry, rz)};

    for (int i = 0; i < 2; i++) {
        m_drum_joint_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.25);
        m_drum_joint_motors[i] = AddMotorSpeed(m_arms[i]->GetBody(), m_drums[i]->GetBody(), m_chassis, arm_motor_rel_pos_2[i], z2y);
        m_drum_joint_motors[i]->SetMotorFunction(m_drum_joint_motor_funcs[i]);
    }
}

void Rassor::SetDriver(std::shared_ptr<RassorDriver> driver) {
    m_driver = driver;
    m_driver->rassor = this;
}

void Rassor::SetWheelContactMaterial(std::shared_ptr<ChContactMaterial> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void Rassor::SetChassisFixed(bool fixed) {
    m_chassis_fixed = fixed;
}

ChVector3d Rassor::GetWheelContactForce(RassorWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector3d Rassor::GetWheelContactTorque(RassorWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector3d Rassor::GetWheelAppliedForce(RassorWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector3d Rassor::GetWheelAppliedTorque(RassorWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double Rassor::GetWheelTracTorque(RassorWheelID id) const {
    if (m_driver->GetDriveMotorType() == RassorDriver::DriveMotorType::TORQUE)
        return 0;

    return m_drive_motors[id]->GetMotorTorque();
}

double Rassor::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass();
    for (int i = 0; i < 4; i++) {
        tot_mass += m_wheels[i]->GetBody()->GetMass();
    }

    tot_mass += m_arms[0]->GetBody()->GetMass();
    tot_mass += m_arms[1]->GetBody()->GetMass();

    tot_mass += m_drums[0]->GetBody()->GetMass();
    tot_mass += m_drums[1]->GetBody()->GetMass();
    return tot_mass;
}

double Rassor::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

double Rassor::GetDrumMass() const {
    return m_drums[0]->GetBody()->GetMass();
}

void Rassor::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    for (int i = 0; i < 4; i++) {
        // Extract driver inputs
        double driving = m_driver->drive_speeds[i];

        // Set motor functions
        m_drive_motor_funcs[i]->SetConstant(driving);
    }

    for (int i = 0; i < 2; i++) {
        double shoulder_rotation = m_driver->shoulder_pos[i];
        double drum_speed = m_driver->drum_speeds[i];
        m_shoulder_joint_motor_funcs[i]->SetConstant(shoulder_rotation);
        m_drum_joint_motor_funcs[i]->SetConstant(drum_speed);
    }
}

void Rassor::writeMeshFile(const std::string& out_dir, int frame_number,  bool save_obj) {
	
    // a list of body pointers
    std::vector<std::shared_ptr<RassorPart>> body_mesh_list;
    body_mesh_list.push_back(m_chassis);
    for (int i = 0; i < 4; i++) {
		body_mesh_list.push_back(m_wheels[i]);
	}

    for (int i = 0; i < 2; i++) {
        body_mesh_list.push_back(m_arms[i]);
        body_mesh_list.push_back(m_drums[i]);
    }

    for (int i = 0; i < body_mesh_list.size(); i++) {

        auto part = body_mesh_list[i];
        // get body position and orientation
        auto body = part->GetBody();
        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();

        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(part->GetMeshName(), false, false);

        // Transform mesh itself
        trimesh->Transform(m_chassis->GetMeshTransform().GetPos(),
                           m_chassis->GetMeshTransform().GetRotMat());  // translate/rotate/scale mesh

        // Transform mesh based on body position and pose
        trimesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        std::string filename = out_dir + "/" + part->GetName() + "_" + std::to_string(frame_number) + ".obj";
        ChTriangleMeshConnected::WriteWavefront(filename, {*trimesh});
    }
}


// =============================================================================

RassorDriver::RassorDriver()
    : drive_speeds({0, 0, 0, 0}), shoulder_pos({0, 0}), drum_speeds({0, 0}), rassor(nullptr) {}

RassorSpeedDriver::RassorSpeedDriver(double time_ramp) : m_ramp(time_ramp) {}

/// Set current drive motor speed input.
void RassorSpeedDriver::SetDriveMotorSpeed(RassorWheelID wheel_id, double drive_speed) {
    drive_speeds[wheel_id] = drive_speed;
}

/// Set current arm motor speed input.
void RassorSpeedDriver::SetShoulderMotorAngle(RassorDirID dir_id, double shoulder_rot) {
    shoulder_pos[dir_id] = shoulder_rot;
}

/// Set current razor motor speed input.
void RassorSpeedDriver::SetDrumMotorSpeed(RassorDirID dir_id, double razor_speed) {
    drum_speeds[dir_id] = razor_speed;
}

void RassorSpeedDriver::Update(double time) {}

}  // namespace rassor
}  // namespace chrono
