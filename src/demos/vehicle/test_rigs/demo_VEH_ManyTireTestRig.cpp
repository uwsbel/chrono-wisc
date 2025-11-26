// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Demonstration of managing many tire test rigs on narrow rigid terrain patches.
//
// =============================================================================

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChManyTireTestRigs.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "demos/SetChronoSolver.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
namespace {

struct DemoParams {
    std::string wheel_json = "Polaris/Polaris_Wheel.json";
    std::string tire_json = "Polaris/Polaris_TMeasyTire.json";
    bool deformable = false;
    bool is_deformable_airless = false;
    double normal_load = 2400.0;  // [N]
    double gravity = 9.81;        // [m/s^2]
    double forward_speed = 0.5;   // [m/s]
    double sim_time = 8.0;        // [s]
    bool render = true;
    double render_fps = 60.0;
    bool set_long_speed = true;
    bool set_ang_speed = false;
    bool set_slip_angle = false;
    double ramp_time = 1.0;        // [s]
    double slip_ratio = 0.0;       // [0..1)
    double slip_angle_deg = 5.0;   // [deg]
    double slip_angle_freq = 0.2;  // [Hz]
    
    void SetTireJson() {
        if (deformable && !is_deformable_airless) {
            tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";
        }
    }
};

bool ParseCommandLine(int argc, char** argv, DemoParams& params) {
    ChCLI cli(argv[0], "Many Tire Test Rig Demo");

    cli.AddOption<double>("Rig", "normal_load", "Normal load per rig (N)", std::to_string(params.normal_load));
    cli.AddOption<double>("Rig", "speed", "Prescribed longitudinal speed (m/s)", std::to_string(params.forward_speed));
    cli.AddOption<double>("Simulation", "gravity", "Gravity magnitude (m/s^2)", std::to_string(params.gravity));
    cli.AddOption<double>("Simulation", "time", "Simulation duration (s)", std::to_string(params.sim_time));
    cli.AddOption<bool>("Visualization", "render", "Enable run-time visualization", "true");
    cli.AddOption<bool>("Motion", "longitudinal", "Enable longitudinal speed profile", "true");
    cli.AddOption<bool>("Motion", "angular", "Enable angular speed profile", "false");
    cli.AddOption<bool>("Motion", "slip_angle", "Enable sinusoidal slip angle", "false");
    cli.AddOption<double>("Motion", "ramp", "Ramp time applied to motion inputs (s)", std::to_string(params.ramp_time));
    cli.AddOption<double>("Motion", "slip_ratio", "Longitudinal slip ratio for angular speed target",
                          std::to_string(params.slip_ratio));
    cli.AddOption<double>("Motion", "slip_angle_amp", "Slip angle amplitude (deg)",
                          std::to_string(params.slip_angle_deg));
    cli.AddOption<double>("Motion", "slip_angle_freq", "Slip angle frequency (Hz)",
                          std::to_string(params.slip_angle_freq));

    if (!cli.Parse(argc, argv))
        return false;

    params.normal_load = cli.GetAsType<double>("normal_load");
    params.forward_speed = cli.GetAsType<double>("speed");
    params.gravity = cli.GetAsType<double>("gravity");
    params.sim_time = cli.GetAsType<double>("time");
    params.render = cli.GetAsType<bool>("render");
    params.set_long_speed = cli.GetAsType<bool>("longitudinal");
    params.set_ang_speed = cli.GetAsType<bool>("angular");
    params.set_slip_angle = cli.GetAsType<bool>("slip_angle");
    params.ramp_time = cli.GetAsType<double>("ramp");
    params.slip_ratio = cli.GetAsType<double>("slip_ratio");
    params.slip_angle_deg = cli.GetAsType<double>("slip_angle_amp");
    params.slip_angle_freq = cli.GetAsType<double>("slip_angle_freq");

    return true;
}

std::shared_ptr<ChFunctionSequence> CreateRampHoldFunction(double target_value, double ramp_time, double hold_time) {
    auto seq = chrono_types::make_shared<ChFunctionSequence>();

    if (ramp_time > 0) {
        auto f_ramp = chrono_types::make_shared<ChFunctionRamp>(0.0, target_value / ramp_time);
        seq->InsertFunct(f_ramp, ramp_time, 1, false, false, false, 0);
    }

    if (hold_time > 0) {
        auto f_const = chrono_types::make_shared<ChFunctionConst>(target_value);
        seq->InsertFunct(f_const, hold_time, 1, false, false, false, ramp_time > 0 ? 1 : 0);
    }

    if (ramp_time <= 0 && hold_time <= 0) {
        seq->InsertFunct(chrono_types::make_shared<ChFunctionConst>(target_value), 0.0, 1, false, false, false, 0);
    }

    seq->Setup();
    return seq;
}

std::shared_ptr<ChSystem> SetupSystem(bool fea_tire) {
    std::shared_ptr<ChSystem> system;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;
    
    // Create system based on FEA tire detection
    if (fea_tire) {
        system = chrono_types::make_shared<ChSystemSMC>();
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::HHT;
    } else {
        system = chrono_types::make_shared<ChSystemNSC>();
        solver_type = ChSolver::Type::BARZILAIBORWEIN;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    }

    system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    // Same as in demo_VEH_TireTestRig_CRM.cpp
    // Number of OpenMP threads used in Chrono
    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
    // Number of threads used in collision detection
    int num_threads_collision = 1;
    // Number of threads used by Eigen
    int num_threads_eigen = 1;
    // Number of threads used by PardisoMKL
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());
    
    system->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
    SetChronoSolver(*system, solver_type, integrator_type, num_threads_pardiso);
    
    return system;
}

void ConfigureVisualSystem(std::shared_ptr<ChVisualSystem>& vis, std::shared_ptr<ChSystem> sys, bool enable_rendering) {
#ifdef CHRONO_VSG
    if (!enable_rendering) {
        vis = nullptr;
        return;
    }

    auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
    vis_vsg->AttachSystem(sys.get());
    vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
    vis_vsg->SetWindowTitle("Chrono :: Many Tire Test Rigs");
    vis_vsg->SetWindowSize(1280, 720);
    vis_vsg->AddCamera(ChVector3d(2.0, 6.0, 1.5));
    vis_vsg->SetLightIntensity(0.9f);
    vis_vsg->SetLightDirection(CH_PI_2, CH_PI / 6);
    vis_vsg->Initialize();

    vis = vis_vsg;
#else
    (void)sys;
    (void)enable_rendering;
    vis = nullptr;
    std::cout << "Chrono built without VSG support. Visualization disabled." << std::endl;
#endif
}

}  // namespace

// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    DemoParams params;
    if (!ParseCommandLine(argc, argv, params))
        return 1;

#ifdef CHRONO_DATA_DIR
    SetChronoDataPath(CHRONO_DATA_DIR);
#endif
    
    // Set tire JSON based on deformable flag
    params.SetTireJson();
    
    std::cout << "Deformable: " << params.deformable << std::endl;    
    auto system = SetupSystem(params.deformable);

    const double step_size = params.deformable ? 5e-4 : 2e-4;
    const double ramp_time = std::max(0.0, params.ramp_time);
    const double hold_time = std::max(0.0, params.sim_time - ramp_time);

    ChManyTireTestRigs rigs(system.get());
    rigs.SetGravitationalAcceleration(ChVector3d(0, 0, -params.gravity));

    ChManyTireTestRigs::RigConfiguration config;
    config.normal_load = params.normal_load;
    config.tire_step = step_size;
    config.time_delay = 0.1;
    config.camber_angle = 0.0;
    config.tire_visualization = VisualizationType::MESH;
    config.collision_type = ChTire::CollisionType::SINGLE_POINT;
    if (params.set_long_speed) {
        config.long_speed_function = CreateRampHoldFunction(params.forward_speed, ramp_time, hold_time);
    }

    config.terrain_params_rigid.friction = 0.8f;
    config.terrain_params_rigid.restitution = 0.0f;
    config.terrain_params_rigid.Young_modulus = 2e7f;
    config.terrain_params_rigid.length = 8.0;
    config.terrain_params_rigid.width = 0.8;  // requirement: width < 1 m

    const size_t num_rigs = 50;

    rigs.AddManyRigs(params.wheel_json, params.tire_json, params.is_deformable_airless, num_rigs, config);

    if (params.set_ang_speed) {
        auto& first_rig = rigs.GetRig(0);
        double radius = first_rig.GetWheel()->GetRadius();
        double slip_denom = std::max(1e-3, 1.0 - params.slip_ratio);
        double angular_target = params.forward_speed / (radius * slip_denom);
        auto ang_fun = CreateRampHoldFunction(angular_target, ramp_time, hold_time);
        rigs.SetAngSpeedFunctionAll(ang_fun);
    }

    if (params.set_slip_angle) {
        auto slip_fun =
            chrono_types::make_shared<ChFunctionSine>(params.slip_angle_deg * CH_DEG_TO_RAD, params.slip_angle_freq);
        rigs.SetSlipAngleFunctionAll(slip_fun);
    }

    rigs.InitializeAll(ChTireTestRig::Mode::TEST);

    // Setup system once before querying DOF counts.
    system->Setup();
    std::cout << "Initialized " << rigs.GetNumRigs() << " tire test rigs on rigid terrain.\n";
    std::cout << "Wheel json: " << params.wheel_json << "\n";
    std::cout << "Tire json: "
              << (params.is_deformable_airless ? std::string("ANCFAirlessTire3443B (built-in)") : params.tire_json) << "\n";
    std::cout << "Position-level DOFs: " << system->GetNumCoordsPosLevel() << "\n";
    std::cout << "Velocity-level DOFs: " << system->GetNumCoordsVelLevel() << "\n";

    std::shared_ptr<ChVisualSystem> vis;
    ConfigureVisualSystem(vis, system, params.render);
    int render_frame = 0;

    while (system->GetChTime() < params.sim_time) {
#ifdef CHRONO_VSG
        if (vis && system->GetChTime() >= render_frame / params.render_fps) {
            auto focus_index = num_rigs / 2;
            ChVector3d focus = rigs.GetCarrierPos(focus_index);
            vis->UpdateCamera(focus + ChVector3d(2.0, 3.0, 1.0), focus);

            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
#endif

        rigs.AdvanceAll(step_size);
    }

    std::cout << "Final time: " << system->GetChTime() << " s\n";
    return 0;
}
