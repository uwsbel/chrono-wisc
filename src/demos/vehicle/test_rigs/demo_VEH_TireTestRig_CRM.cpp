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
// Authors: Radu Serban, Huzaifa Unjhawala
// =============================================================================
//
// Demonstration of the single-wheel tire test rig.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------
bool use_airless_tire = true;
// Tire specification file
std::string tire_json = "Polaris/Polaris_RigidMeshTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Wheel specification file
std::string wheel_json = "Polaris/Polaris_Wheel.json";

double render_fps = 100;
bool debug_output = false;
bool gnuplot_output = false;
bool blender_output = false;

bool set_longitudinal_speed = false;
bool set_angular_speed = true;
bool set_slip_angle = false;

double max_linear_speed = 1;
double ramp_time = 1;
double constant_time = 8;
double slip = 0.0;
double sim_time_max = ramp_time + constant_time;

bool set_str_spk = true;
bool snapshots = false;

double input_time_delay = 0.5;
bool render = false;

// -----------------------------------------------------------------------------

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& density,
                     double& cohesion,
                     double& friction,
                     double& gravity,
                     double& slope,
                     bool& use_deformable_tire) {
    ChCLI cli(argv[0], "Tire Test Rig on CRM Terrain Demo");

    cli.AddOption<double>("Terrain", "density", "Terrain bulk density (kg/m3)", std::to_string(density));
    cli.AddOption<double>("Terrain", "cohesion", "Terrain cohesion (Pa)", std::to_string(cohesion));
    cli.AddOption<double>("Terrain", "friction", "Terrain friction coefficient", std::to_string(friction));
    cli.AddOption<double>("Simulation", "gravity", "Gravitational acceleration magnitude (m/s2)",
                          std::to_string(gravity));
    cli.AddOption<double>("Simulation", "slope", "Slope angle in degrees", std::to_string(slope));
    cli.AddOption<bool>("Tire", "deformable", "Use deformable tire (true) or rigid tire (false)", 
                        "false");

    if (!cli.Parse(argc, argv))
        return false;

    density = cli.GetAsType<double>("density");
    cohesion = cli.GetAsType<double>("cohesion");
    friction = cli.GetAsType<double>("friction");
    gravity = cli.GetAsType<double>("gravity");
    slope = cli.GetAsType<double>("slope");
    use_deformable_tire = cli.GetAsType<bool>("deformable");
    return true;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Default values for CLI parameters
    double density = 1700;
    double cohesion = 1e2;
    double friction = 0.7;
    double gravity = 9.8;
    double slope = 0.0;  // degrees
    bool use_deformable_tire = false;  // Default to rigid tire

    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, density, cohesion, friction, gravity, slope, use_deformable_tire)) {
        return 1;
    }

    cout << "Density: " << density << endl;
    cout << "Cohesion: " << cohesion << endl;
    cout << "Friction: " << friction << endl;
    cout << "Gravity: " << gravity << endl;
    cout << "Slope: " << slope << endl;
    cout << "Tire type: " << (use_deformable_tire ? "deformable" : "rigid") << endl;
    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------
    std::shared_ptr<ChTire> tire;
    auto wheel = ReadWheelJSON(GetVehicleDataFile(wheel_json));
    
    // Set use_airless_tire based on CLI parameter
    use_airless_tire = use_deformable_tire;
    
    if (!use_airless_tire) {
        tire = ReadTireJSON(GetVehicleDataFile(tire_json));
    } else {
        tire = chrono_types::make_shared<ANCFAirlessTire3443B>("ANCFairless tire");
        // These are default sizes for the polaris tire
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.225);           // Default is 0.225
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.225);              // Default is 0.225
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.4);                 // Default is 0.4
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);                // Default is 0.05
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(0.015);  // Default is 0.015
        // tire->SetYoungsModulus(y_mod);  // Default is 76e9
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e6);
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(5e9);
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);       // Default is 0.2
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);              // Default is 3
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);        // Default is 3
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);  // Default is 3
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetNumberSpokes(40);
        // Options to set for straight spokes
        if (set_str_spk) {
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHubRelativeRotation(0);
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureXPoint(0);
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetSpokeCurvatureZPoint(0);
        }
    }

    bool fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;

    // Set tire contact surface (relevant for FEA tires only)
    if (fea_tire) {
        int collision_family = 7;
        auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
        double surface_dim = 0;
        tire->SetContactSurfaceType(surface_type, surface_dim, collision_family);
    }

    // ---------------------------------------------------------
    // Create system and set default solver and integrator types
    // ---------------------------------------------------------

    ChSystem* sys = nullptr;
    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    if (fea_tire) {
        sys = new ChSystemSMC;
        step_size = 1e-3;
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::HHT;

    } else {
        sys = new ChSystemNSC;
        step_size = 2e-4;
        solver_type = ChSolver::Type::BARZILAIBORWEIN;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    }

    // Set collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Number of OpenMP threads used in Chrono (SCM ray-casting and FEA)
    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());

    // Number of threads used in collision detection
    int num_threads_collision = 1;

    // Number of threads used by Eigen
    int num_threads_eigen = 1;

    // Number of threads used by PardisoMKL
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
    SetChronoSolver(*sys, solver_type, integrator_type, num_threads_pardiso);
    tire->SetStepsize(step_size);

    // -----------------------------
    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, sys);

    // Calculate gravity vector based on slope
    ChVector3d gravity_vec;
    // No slope - standard vertical gravity
    gravity_vec = ChVector3d(0, 0, -gravity);
    // Set the slope angle in degrees
    rig.SetSlope(slope);

    // Set gravity in the system
    rig.SetGravitationalAcceleration(gravity_vec);  // Set magnitude for rig internal calculations

    rig.SetNormalLoad(2500);

    ////rig.SetCamberAngle(+15 * CH_DEG_TO_RAD);

    rig.SetTireStepsize(step_size);
    // Disable visualization for rigid tires
    rig.SetTireVisualizationType(VisualizationType::NONE);

    ChTireTestRig::TerrainParamsCRM params;
    params.radius = 0.01;
    params.density = density;
    params.cohesion = cohesion;
    params.friction = friction;
    params.length = 10;
    params.width = 1;
    params.depth = 0.2;
    rig.SetTerrainCRM(params);

    // -----------------
    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10.0));
    ////rig.Initialize();

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(1.0));
    ////rig.Initialize();

    // Scenario: imobilized wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    ////rig.Initialize();

    // Scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 10 RPM
    //   slip angle: sinusoidal +- 5 deg with 5 s period
    
    if (set_longitudinal_speed) {
        ChFunctionSequence f_sequence;
        auto f_ramp = chrono_types::make_shared<ChFunctionRamp>(0, max_linear_speed / ramp_time);
        f_sequence.InsertFunct(f_ramp, ramp_time, 1, false, false, false, 0);
        auto f_const = chrono_types::make_shared<ChFunctionConst>(max_linear_speed);
        f_sequence.InsertFunct(f_const, constant_time, 1, false, false, false, 1);
        f_sequence.Setup();
        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionSequence>(f_sequence));
        std::cout << "Longitudinal speed enabled with max speed applied as ramp: " << max_linear_speed << " m/s"
                  << std::endl;
    }

    if (set_angular_speed) {
        double angular_vel = max_linear_speed / (wheel->GetRadius() * (1 - slip));
        double rate = angular_vel / ramp_time;
        ChFunctionSequence f_sequence;
        auto f_ramp = chrono_types::make_shared<ChFunctionRamp>(0, rate);
        f_sequence.InsertFunct(f_ramp, ramp_time, 1, false, false, false, 0);
        auto f_const = chrono_types::make_shared<ChFunctionConst>(angular_vel);
        f_sequence.InsertFunct(f_const, constant_time, 1, false, false, false, 1);
        f_sequence.Setup();
        rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionSequence>(f_sequence));
        std::cout << "Angular speed of " << angular_vel << " rad/s enabled: With slip " << slip
                  << " and max speed applied as ramp: " << max_linear_speed << " m/s" << std::endl;
    }

    if (set_slip_angle) {
        rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionSine>(5 * CH_DEG_TO_RAD, 0.2));
        std::cout << "Slip angle enabled: 5 deg amplitude, 0.2 Hz" << std::endl;
    }

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(input_time_delay);
    ////rig.Initialize(ChTireTestRig::Mode::SUSPEND);
    ////rig.Initialize(ChTireTestRig::Mode::DROP);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    // Print tire mass and inertia
    double tire_mass = tire->GetTireMass();
    ChVector3d tire_inertia = tire->GetTireInertia();
    
    std::cout << "Tire mass: " << tire_mass << " kg" << std::endl;
    std::cout << "Tire inertia (Ixx, Iyy, Izz): " << tire_inertia.x() << ", " 
              << tire_inertia.y() << ", " << tire_inertia.z() << " kg·m²" << std::endl;

    // Set gravity in CRM terrain system after initialization
    std::shared_ptr<CRMTerrain> crm_terrain = std::dynamic_pointer_cast<CRMTerrain>(rig.GetTerrain());
    if (crm_terrain) {
        auto sysFSI = crm_terrain->GetFsiSystemSPH();
        if (sysFSI) {
            // Set gravity on the fluid system
            auto& fluidSys = sysFSI->GetFluidSystemSPH();
            fluidSys.SetGravitationalAcceleration(gravity_vec);
        }
    }

    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        auto visFEA = chrono_types::make_shared<ChVisualShapeFEA>();
        visFEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        visFEA->SetShellResolution(3);
        visFEA->SetWireframe(false);
        visFEA->SetColormapRange(0.0, 5.0);
        visFEA->SetSmoothFaces(true);
        tire_def->AddVisualShapeFEA(visFEA);
    }

    // -----------------
    // Initialize output
    // -----------------

    // Create output directory
    std::string output_dir = "tire_test_crm";
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directory(output_dir);
    }

    // Generate unique filename with simulation parameters
    std::string filename = output_dir + "/tire_data_d" + std::to_string((int)density) + "_f" +
                           std::to_string((int)(friction * 10)) + "_c" + std::to_string((int)(cohesion / 100)) + "_g" +
                           std::to_string((int)(gravity * 10)) + "_s" + std::to_string((int)slope) + ".csv";

    // Create snapshots directory if snapshots are enabled
    std::string snapshots_dir;
    if (render && snapshots) {
        snapshots_dir = output_dir + "/snapshots_d" + std::to_string((int)density) + "_f" +
                        std::to_string((int)(friction * 10)) + "_c" + std::to_string((int)(cohesion / 100)) + "_g" +
                        std::to_string((int)(gravity * 10)) + "_s" + std::to_string((int)slope);
        if (!std::filesystem::exists(snapshots_dir)) {
            std::filesystem::create_directory(snapshots_dir);
        }
    }

    // Open output file for writing
    std::ofstream position_file;
    position_file.open(filename);

    // Write CSV header
    position_file << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,omega,torque" << std::endl;

    const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

#ifdef CHRONO_VSG
    std::shared_ptr<ChVisualSystem> vis = nullptr;
    if (render) {
        // FSI plugin
        auto sysFSI = std::dynamic_pointer_cast<CRMTerrain>(rig.GetTerrain())->GetFsiSystemSPH();
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->EnableFlexBodyMarkers(true);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(sys);
        visVSG->SetWindowTitle("Tire Test Rig on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(1.0, 2.5, 1.0), ChVector3d(0, 1, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

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
    double sim_time_max = 10;

    // Data collection
    ChFunctionInterp long_slip_fct;
    ChFunctionInterp slip_angle_fct;
    ChFunctionInterp camber_angle_fct;

    double time_offset = 0.5;
    timer.start();

    // Find the spindle body for getting position and velocity
    std::shared_ptr<ChBody> spindle_body = nullptr;
    for (auto body : sys->GetBodies()) {
        if (body->GetName() == "rig_spindle") {
            spindle_body = body;
            break;
        }
    }

    // Find the rotation motor for getting torque
    std::shared_ptr<ChLinkMotorRotationSpeed> rot_motor = nullptr;
    for (auto link : sys->GetLinks()) {
        if (auto motor = std::dynamic_pointer_cast<ChLinkMotorRotationSpeed>(link)) {
            rot_motor = motor;
            break;
        }
    }

    // Find the slip body for camera tracking
    std::shared_ptr<ChBody> slip_body = nullptr;
    for (auto body : sys->GetBodies()) {
        if (body->GetName() == "rig_slip") {
            slip_body = body;
            break;
        }
    }
    double x_max = params.length - 0.7;
    while (time < sim_time_max) {
        time = sys->GetChTime();

        if ((debug_output || gnuplot_output) && time >= time_offset) {
            long_slip_fct.AddPoint(time, tire->GetLongitudinalSlip());
            slip_angle_fct.AddPoint(time, tire->GetSlipAngle() * CH_RAD_TO_DEG);
            camber_angle_fct.AddPoint(time, tire->GetCamberAngle() * CH_RAD_TO_DEG);
        }

        // Get spindle position and velocity
        ChVector3d pos = spindle_body ? spindle_body->GetPos() : ChVector3d(0, 0, 0);
        ChVector3d vel = spindle_body ? spindle_body->GetLinVel() : ChVector3d(0, 0, 0);
        
        // Get motor torque and angular velocity (about motor rotation axis)
        double torque = rot_motor ? rot_motor->GetMotorTorque() : 0.0;
        double omega = rot_motor ? rot_motor->GetMotorAngleDt() : 0.0;

        // Leave if end of terrain reached
        if (pos.x() > x_max) {
            std::cout << "End of terrain reached" << std::endl;
            break;
        }
        // Write position, velocity, angular velocity, and torque data to CSV file
        if (time >= time_offset) {
            position_file << time << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << vel.x() << ","
                          << vel.y() << "," << vel.z() << "," << omega << "," << torque << std::endl;
        }

#ifdef CHRONO_VSG
        if (render && vis && time >= render_frame / render_fps) {
            // Follow the slip body (green box) for camera tracking
            ChVector3d loc = slip_body ? slip_body->GetPos() : rig.GetPos();
            vis->UpdateCamera(loc + ChVector3d(1.0, 2.5, 0.5), loc + ChVector3d(0, 0.25, -0.25));

            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;

#ifdef CHRONO_POSTPROCESS
            if (blender_output)
                blender_exporter.ExportData();
#endif

            // Save snapshots if enabled
            if (snapshots) {
                std::ostringstream snapshot_filename;
                snapshot_filename << snapshots_dir << "/" << std::setw(5) << std::setfill('0') << render_frame
                                  << ".jpg";
                vis->WriteImageToFile(snapshot_filename.str());
            }
        }
#endif

        rig.Advance(step_size);
        sim_time += sys->GetTimerStep();

        if (debug_output) {
            cout << time << endl;
            auto long_slip = tire->GetLongitudinalSlip();
            auto slip_angle = tire->GetSlipAngle();
            auto camber_angle = tire->GetCamberAngle();
            cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << endl;
            auto tforce = rig.ReportTireForce();
            auto frc = tforce.force;
            auto pnt = tforce.point;
            auto trq = tforce.moment;
            cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << endl;
            cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << endl;
            cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << endl;
        } else {
            // cout << "\rRTF: " << sys->GetRTF();
        }
    }
    timer.stop();

    // Close the position file
    position_file.close();
    std::cout << "\nPosition and velocity data saved to: " << filename << std::endl;

    double step_time = timer();
    cout << "\rSimulated time: " << time << endl;
    cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << endl;

#ifdef CHRONO_POSTPROCESS
    // ------------
    // Plot results
    // ------------

    if (gnuplot_output && sys->GetChTime() > time_offset) {
        postprocess::ChGnuPlot gplot_long_slip(out_dir + "/tmp1.gpl");
        gplot_long_slip.SetGrid();
        gplot_long_slip.SetLabelX("time (s)");
        gplot_long_slip.SetLabelY("Long. slip");
        gplot_long_slip.Plot(long_slip_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_slip_angle(out_dir + "/tmp2.gpl");
        gplot_slip_angle.SetGrid();
        gplot_slip_angle.SetLabelX("time (s)");
        gplot_slip_angle.SetLabelY("Slip angle");
        gplot_slip_angle.Plot(slip_angle_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_camber_angle(out_dir + "/tmp3.gpl");
        gplot_camber_angle.SetGrid();
        gplot_camber_angle.SetLabelX("time (s)");
        gplot_camber_angle.SetLabelY("Camber angle");
        gplot_camber_angle.Plot(camber_angle_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
#endif

    return 0;
}
