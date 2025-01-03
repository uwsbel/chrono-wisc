// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ganesh Arivoli, Huzaifa Unjhawala
// =============================================================================
//
// Test rig with CLI for automated testing of tire models
//
// =============================================================================

#include <algorithm>

#include <fstream>
#include <filesystem>
#include <regex>
#include <sstream>
#include <iomanip>
#include <iostream>
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif
#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Tire model
enum class TireType { RIGID, ANCF_TOROIDAL, ANCF_AIRLESS };
TireType tire_type = TireType::ANCF_AIRLESS;

// Terrain type (RIGID or SCM)
enum class TerrainType { RIGID, SCM };

double render_fps = 100;
bool log_output = true;
bool blender_output = false;
////std::string wheel_json = "hmmwv/wheel/HMMWV_Wheel.json";
std::string wheel_json = "Polaris/Polaris_Wheel.json";
double time_delay = 0.1;

// ----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     int& refine_level,
                     double& y_mod,
                     double& step_c,
                     double& p_ratio,
                     std::string& scm_type,
                     double& normal_load,
                     std::string& terrain_type_str,
                     double& slope,
                     std::string& tire_type_str,
                     bool& set_long_speed,
                     bool& set_ang_speed,
                     bool& set_slip_angle,
                     ChSolver::Type& solver_type) {
    ChCLI cli(argv[0], "Tire Test Rig Configuration");

    cli.AddOption<int>("Mesh", "refine", "Mesh refinement level (1,2,3) - Only for ANCF_AIRLESS tire",
                       std::to_string(refine_level));
    // Add options for Young's modulus and step size
    cli.AddOption<double>("Material", "ym", "Young's modulus (Pa) - Required Parameter", std::to_string(y_mod));
    cli.AddOption<double>("Material", "pr", "Poisson's ratio - Required Parameter", std::to_string(p_ratio));
    cli.AddOption<double>("Simulation", "st", "Step size - Required Parameter", std::to_string(step_c));
    cli.AddOption<std::string>("Terrain", "type", "Terrain type (rigid/scm)", terrain_type_str);
    cli.AddOption<std::string>("SCM type", "scm", "SCM Terrain Type (soft, medium, hard) - Only if terrain is SCM",
                               scm_type);
    cli.AddOption<std::string>("Tire", "tire", "Tire type (rigid/ancf_toroidal/ancf_airless)", tire_type_str);
    cli.AddOption<double>("Simulation", "nl", "Normal Load (N)", std::to_string(normal_load));

    // Motion control enable/disable options
    cli.AddOption<std::string>("Motion", "long_speed", "Enable longitudinal speed control (default: 0, use 0/1)", "0");
    cli.AddOption<std::string>("Motion", "ang_speed", "Enable angular speed control (default: 1, use 0/1)", "1");
    cli.AddOption<std::string>("Motion", "slip", "Enable slip angle control (default: 0, use 0/1)", "0");
    cli.AddOption<double>("Terrain", "slope", "Terrain slope (degrees)", std::to_string(slope));

    cli.AddOption<std::string>(
        "Solver", "solver",
        "Solver type - Only applicable for ANCF Tires (pardiso_mkl/sparse_lu, default: pardiso_mkl)", "pardiso_mkl");

    if (argc == 1) {
        cout << "Required parameters missing. See required parameters and descriptions below:\n\n";
        cli.Help();
        return false;
    }

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    // Retrieve values from CLI
    y_mod = cli.GetAsType<double>("ym");
    step_c = cli.GetAsType<double>("st");
    p_ratio = cli.GetAsType<double>("pr");
    terrain_type_str = cli.GetAsType<std::string>("type");
    tire_type_str = cli.GetAsType<std::string>("tire");
    slope = cli.GetAsType<double>("slope");
    normal_load = cli.GetAsType<double>("nl");

    // Only get SCM type if terrain is SCM
    if (terrain_type_str == "scm") {
        scm_type = cli.GetAsType<std::string>("scm");
    }

    // Mesh refinement level only for ANCF_AIRLESS tire
    if (tire_type_str == "ancf_airless") {
        refine_level = cli.GetAsType<int>("refine");
    }

    // Get motion control flags
    std::string long_speed_str = cli.GetAsType<std::string>("long_speed");
    std::string ang_speed_str = cli.GetAsType<std::string>("ang_speed");
    std::string slip_str = cli.GetAsType<std::string>("slip");

    // Convert string to bool (accepting 0/1 or true/false)
    auto str_to_bool = [](const std::string& str) {
        return (str == "1" || str == "true" || str == "True") ? true : false;
    };

    set_long_speed = str_to_bool(long_speed_str);
    set_ang_speed = str_to_bool(ang_speed_str);
    set_slip_angle = str_to_bool(slip_str);

    std::string solver_str = cli.GetAsType<std::string>("solver");
    if (solver_str == "pardiso_mkl") {
        solver_type = ChSolver::Type::PARDISO_MKL;
    } else if (solver_str == "sparse_lu") {
        solver_type = ChSolver::Type::SPARSE_LU;
    } else {
        std::cout << "Unknown solver type '" << solver_str << "', using default PARDISO_MKL" << std::endl;
        solver_type = ChSolver::Type::PARDISO_MKL;
    }

    return true;
}
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double y_mod = 10e9;
    double step_c = 1e-4;
    double p_ratio = 0.2;
    std::string scm_type = "soft";
    double normal_load = 3000;
    std::string terrain_type_str = "rigid";
    std::string tire_type_str = "ancf_airless";
    int refine_level = 1;
    double slope = 0;
    // Motion control flags
    bool set_longitudinal_speed = false;
    bool set_angular_speed = true;
    bool set_slip_angle = false;
    ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;
    if (!GetProblemSpecs(argc, argv, refine_level, y_mod, step_c, p_ratio, scm_type, normal_load, terrain_type_str,
                         slope, tire_type_str, set_longitudinal_speed, set_angular_speed, set_slip_angle,
                         solver_type)) {
        return 1;
    }
    TerrainType terrain_type;
    TireType tire_type;
    // Set terrain type based on input
    if (terrain_type_str == "scm") {
        terrain_type = TerrainType::SCM;
    } else {
        terrain_type = TerrainType::RIGID;
    }

    // Set tire type based on input using switch
    if (tire_type_str == "rigid") {
        tire_type = TireType::RIGID;
    } else if (tire_type_str == "ancf_toroidal") {
        tire_type = TireType::ANCF_TOROIDAL;
    } else if (tire_type_str == "ancf_airless") {
        tire_type = TireType::ANCF_AIRLESS;
    } else {
        std::cout << "Unknown tire type '" << tire_type_str << "', using default ANCF Airless Tire" << std::endl;
        tire_type = TireType::ANCF_AIRLESS;
    }

    const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    std::string log_file = out_dir + "/tire_test.txt";
    std::ofstream logfile(log_file);
    if (!logfile.is_open()) {
        cerr << "Could not open log file " << log_file << endl;
        return 1;
    }

    // Write all problem specs to cout and logfile
    auto WriteSpecs = [&](std::ostream& out) {
        out << "Problem Specs:" << std::endl;
        out << "Tire Type: " << tire_type_str << std::endl;
        out << "Terrain Type: " << terrain_type_str << std::endl;
        if (terrain_type_str == "scm") {
            out << "SCM Type: " << scm_type << std::endl;
        }
        if (tire_type_str == "ancf_airless") {
            out << "Refine Level: " << refine_level << std::endl;
        }
        out << "Young's Modulus: " << y_mod << std::endl;
        out << "Poisson's Ratio: " << p_ratio << std::endl;
        out << "Step Size: " << step_c << std::endl;
        out << "Normal Load: " << normal_load << std::endl;
        out << "Set Longitudinal Speed: " << set_longitudinal_speed << std::endl;
        out << "Set Angular Speed: " << set_angular_speed << std::endl;
        out << "Set Slip Angle: " << set_slip_angle << std::endl;
        out << "Solver Type: " << ChSolver::GetTypeAsString(solver_type) << std::endl;
        out << std::endl;
    };

    // Write to console
    WriteSpecs(std::cout);

    // Write to logfile
    WriteSpecs(logfile);

    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = ReadWheelJSON(vehicle::GetDataFile(wheel_json));

    std::shared_ptr<ChTire> tire;
    if (tire_type == TireType::ANCF_TOROIDAL) {
        auto ancf_tire = chrono_types::make_shared<ANCFToroidalTire>("ANCFtoroidal tire");
        ancf_tire->SetRimRadius(0.225);
        ancf_tire->SetHeight(0.225);
        ancf_tire->SetThickness(0.015);
        ancf_tire->SetDivCircumference(40);
        ancf_tire->SetDivWidth(8);
        ancf_tire->SetPressure(320e3);
        ancf_tire->SetAlpha(0.15);
        tire = ancf_tire;
    } else if (tire_type == TireType::ANCF_AIRLESS) {
        auto ancf_tire = chrono_types::make_shared<ANCFAirlessTire>("ANCFairless tire");
        // These are default sizes for the polaris tire
        ancf_tire->SetRimRadius(0.225);                        // Default is 0.225
        ancf_tire->SetHeight(0.225);                           // Default is 0.225
        ancf_tire->SetWidth(0.24);                             // Default is 0.4
        ancf_tire->SetAlpha(0.05);                             // Default is 0.05
        ancf_tire->SetYoungsModulus(y_mod);                    // Default is 76e9
        ancf_tire->SetPoissonsRatio(p_ratio);                  // Default is 0.2
        ancf_tire->SetDivWidth(3 * refine_level);              // Default is 3
        ancf_tire->SetDivSpokeLength(3 * refine_level);        // Default is 3
        ancf_tire->SetDivOuterRingPerSpoke(3 * refine_level);  // Default is 3
        tire = ancf_tire;
    } else {
        std::string tire_file;
        tire_file = "hmmwv/tire/HMMWV_RigidTire_mod.json";  // This mod tire has the correct dimensions
        tire = ReadTireJSON(vehicle::GetDataFile(tire_file));
    }

    bool fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;

    // Set tire contact surface (relevant for FEA tires only)
    if (fea_tire) {
        int collision_family = 7;
        auto surface_type = ChTire::ContactSurfaceType::NODE_CLOUD;
        double surface_dim = 0.02;
        if (terrain_type == TerrainType::SCM) {
            surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
            surface_dim = 0;
        }
        tire->SetContactSurfaceType(surface_type, surface_dim, collision_family);
    }

    // ---------------------------------------------------------
    // Create system and set default solver and integrator types
    // ---------------------------------------------------------

    ChSystem* sys = nullptr;
    double step_size = 0;
    ChTimestepper::Type integrator_type;

    if (fea_tire) {
        sys = new ChSystemSMC;
        step_size = step_c;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    } else {
        sys = new ChSystemNSC;
        step_size = 2e-4;  // 2e-4
        solver_type = ChSolver::Type::BARZILAIBORWEIN;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    }

    // Set collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // Number of OpenMP threads used in Chrono (SCM ray-casting and FEA)
    int num_threads_chrono;

    // Number of threads used in collision detection
    int num_threads_collision;

    // Number of threads used by Eigen
    int num_threads_eigen;

    // Number of threads used by PardisoMKL
    int num_threads_pardiso;

    if (solver_type == ChSolver::Type::PARDISO_MKL) {
        num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
        num_threads_collision = 1;
        num_threads_eigen = 1;
        num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());
    } else {
        num_threads_chrono = std::min(14, ChOMP::GetNumProcs());
        num_threads_collision = 1;
        num_threads_eigen = 1;
        num_threads_pardiso = 0;
    }

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
    SetChronoSolver(*sys, solver_type, integrator_type, num_threads_pardiso);
    tire->SetStepsize(step_size);

    // -----------------------------
    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, sys);

    // Set runoff distance from tire to edge of terrain
    double run_off = 5 * tire->GetRadius();
    rig.SetRunOff(run_off);

    rig.SetGravitationalAcceleration(9.8);
    rig.SetNormalLoad(normal_load);

    rig.SetTireStepsize(step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    if (terrain_type == TerrainType::RIGID) {
        ChTireTestRig::TerrainParamsRigid params;
        params.friction = 0.8f;
        params.restitution = 0;
        params.Young_modulus = 2e7f;
        params.length = 20;
        params.width = 1;

        rig.SetTerrainRigid(params);
    } else if (terrain_type == TerrainType::SCM) {
        ChTireTestRig::TerrainParamsSCM params;

        if (scm_type == "soft") {  // Soft
            params.Bekker_Kphi = 1e7;
            params.Bekker_Kc = 0;
            params.Bekker_n = 1.1;
            params.Mohr_cohesion = 0;
            params.Mohr_friction = 20;
            params.Janosi_shear = 0.01;
            params.Elastic_Stiffness = 2e8;
            params.Damping = 3e4;
            std::cout << "Using Soft SCM Terrain" << std::endl;
        } else if (scm_type == "medium") {  // Medium
            params.Bekker_Kphi = 2e7;
            params.Bekker_Kc = 0;
            params.Bekker_n = 1.1;
            params.Mohr_cohesion = 0;
            params.Mohr_friction = 20;
            params.Janosi_shear = 0.01;
            params.Elastic_Stiffness = 2e8;
            params.Damping = 3e4;
            std::cout << "Using Medium SCM Terrain" << std::endl;
        } else if (scm_type == "hard") {  // Hard
            params.Bekker_Kphi = 4e7;
            params.Bekker_Kc = 0;
            params.Bekker_n = 1.1;
            params.Mohr_cohesion = 0;
            params.Mohr_friction = 20;
            params.Janosi_shear = 0.01;
            params.Elastic_Stiffness = 2e8;
            params.Damping = 3e4;
            std::cout << "Using Hard SCM Terrain" << std::endl;
        } else {
            // initial terrain parameters
            params.Bekker_Kphi = 2e6;
            params.Bekker_Kc = 0;
            params.Bekker_n = 1.1;
            params.Mohr_cohesion = 0;
            params.Mohr_friction = 30;
            params.Janosi_shear = 0.01;
            params.Elastic_Stiffness = 2e8;
            params.Damping = 3e4;
            std::cout << "Unknown SCM type '" << scm_type << "', using default terrain parameters" << std::endl;
        }
        params.length = 20;
        params.width = 1;
        params.grid_spacing = 0.025;
        rig.SetTerrainSCM(params);
    }

    // Scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 50 RPM
    //   slip angle: sinusoidal +- 5 deg with 5 s period
    if (set_longitudinal_speed) {
        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.2));
        std::cout << "Longitudinal speed enabled: 0.2 m/s" << std::endl;
    }

    if (set_angular_speed) {
        rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10 * CH_RPM_TO_RAD_S));
        std::cout << "Angular speed enabled: 10 RPM" << std::endl;
    }

    if (set_slip_angle) {
        rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionSine>(5 * CH_DEG_TO_RAD, 0.2));
        std::cout << "Slip angle enabled: 5 deg amplitude, 0.2 Hz" << std::endl;
    }

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(time_delay);

    // Set the slope
    rig.SetSlope(slope * CH_DEG_TO_RAD);
    ////rig.Initialize(ChTireTestRig::Mode::SUSPEND);
    ////rig.Initialize(ChTireTestRig::Mode::DROP);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        auto visFEA = chrono_types::make_shared<ChVisualShapeFEA>();
        visFEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        visFEA->SetShellResolution(3);
        visFEA->SetWireframe(false);
        visFEA->SetColorscaleMinMax(0.0, 5.0);
        visFEA->SetSmoothFaces(true);
        tire_def->AddVisualShapeFEA(visFEA);
    }

    // -----------------
    // Initialize output
    // -----------------

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 600);
            vis_irr->SetWindowTitle("Tire Test Rig");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_irr->AddLightDirectional();

            vis_irr->GetActiveCamera()->setFOV(irr::core::PI / 4.5f);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowSize(1200, 600);
            vis_vsg->SetWindowTitle("Tire Test Rig");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

#ifdef CHRONO_POSTPROCESS
    // ---------------------------
    // Create the Blender exporter
    // ---------------------------

    postprocess::ChBlender blender_exporter(sys);

    if (blender_output) {
        std::string blender_dir = out_dir + "/blender";
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            cerr << "Error creating directory " << blender_dir << endl;
            return 1;
        }

        blender_exporter.SetBlenderUp_is_ChronoZ();
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(3, 3, 1), ChVector3d(0, 0, 0), 50);
        blender_exporter.ExportScript();
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------

    // Timers and counters
    ChTimer timer;         // timer for measuring total run time
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    int render_frame = 0;  // render frame counter

    // Data collection
    ChFunctionInterp long_slip_fct;
    ChFunctionInterp slip_angle_fct;
    ChFunctionInterp camber_angle_fct;

    double time_offset = 0.5;
    double step_time;
    std::shared_ptr<ChBody> spindle_body = rig.GetSpindleBody();
    double wheel_init_x = spindle_body->GetPos().x();
    // Write crash info to both console and logfile
    auto WriteSimStats = [&](std::ostream& out) {
        out << "Simulated time: " << time << std::endl;
        out << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << std::endl;
        out << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << std::endl;
        out << "RTF: " << sys->GetRTF() << std::endl;
    };
    timer.start();
    while (vis->Run() && time < 5) {
        time = sys->GetChTime();
        if (std::isnan(spindle_body->GetPos().z()) || abs(spindle_body->GetPos().z()) > 1000) {
            ChVector3d pos = spindle_body->GetPos();
            timer.stop();
            step_time = timer();
            std::cout << "Simulation appears to have crashed" << std::endl;

            WriteSimStats(std::cout);
            WriteSimStats(logfile);

            return 1;
        } else if (wheel_init_x - spindle_body->GetPos().x() > run_off) {
            std::cout << std::endl << "Wheel has moved backwards beyond the runoff distance" << std::endl;
            timer.stop();
            step_time = timer();

            WriteSimStats(std::cout);
            WriteSimStats(logfile);

            return 1;
        }

        if (time >= render_frame / render_fps) {
            auto& loc = rig.GetPos();

            vis->BeginScene();
            vis->Render();
            vis->EndScene();

#ifdef CHRONO_POSTPROCESS
            if (blender_output)
                blender_exporter.ExportData();
#endif
        }

        rig.Advance(step_size);
        sim_time += sys->GetTimerStep();

        if (log_output) {
            // cout << time << endl;
            auto long_slip = tire->GetLongitudinalSlip();
            auto slip_angle = tire->GetSlipAngle();
            auto camber_angle = tire->GetCamberAngle();
            // cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << endl;

            auto tforce = rig.ReportTireForce();
            auto frc = tforce.force;
            auto pnt = tforce.point;
            auto trq = tforce.moment;
            // cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << endl;
            // cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << endl;
            // cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << endl;
            /*
                        //  Log the data to file
                        logfile << "Time: " << time << std::endl;
                        logfile << "   Longitudinal_Slip: " << long_slip << std::endl;
                        logfile << " Slip_Angle: " << slip_angle << std::endl;
                        logfile << " Camber_Angle: " << camber_angle << std::endl;
                        logfile << "   Force: " << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
                        logfile << "   Point: " << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
                        logfile << "   Moment: " << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
            */
        }

        cout << "\rRTF: " << sys->GetRTF();
    }

    timer.stop();
    step_time = timer();
    std::cout << std::endl << "Simulation completed" << std::endl;
    WriteSimStats(std::cout);
    WriteSimStats(logfile);

    return 0;
}