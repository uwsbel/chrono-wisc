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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

bool use_airless_tire = true;
// Tire specification file - When above is set to true, this is ignored
std::string tire_json = "Polaris/Polaris_RigidMeshTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Wheel specification file
std::string wheel_json = "Polaris/Polaris_Wheel.json";

double render_fps = 100;
bool debug_output = false;
bool gnuplot_output = true;
bool blender_output = false;

bool set_longitudinal_speed = true;
bool set_angular_speed = true;
bool set_slip_angle = false;

double max_linear_speed = 5;
double ramp_time = 5;
double constant_time = 4;
double slip = 0.1;
double sim_time_max = ramp_time + constant_time;

double input_time_delay = 0.5;
bool render = true;
bool set_str_spk = true;
bool snapshots = true;
// -----------------------------------------------------------------------------

int main() {
    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = ReadWheelJSON(vehicle::GetDataFile(wheel_json));
    std::shared_ptr<ChTire> tire;
    if (!use_airless_tire) {
        tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
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
        std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetNumberSpokes(80);
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
        auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;  // CRM only supports triangle mesh
        double surface_dim = 0;                                         // Irrelevant for CRM
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

    rig.SetGravitationalAcceleration(9.8);
    rig.SetNormalLoad(2500);

    ////rig.SetCamberAngle(+15 * CH_DEG_TO_RAD);

    rig.SetTireStepsize(step_size);
    rig.SetTireVisualizationType(VisualizationType::COLLISION);

    ChTireTestRig::TerrainParamsCRM params;
    params.radius = 0.01;
    params.density = 1700;
    params.cohesion = 5e3;
    params.length = 20;
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
        std::cout << "Angular speed enabled: With slip " << slip
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

    const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    std::string snap_dir = out_dir + "/SNAPSHOTS";
    if (render && snapshots) {
        if (!filesystem::create_directory(filesystem::path(snap_dir))) {
            std::cerr << "Error creating directory " << snap_dir << std::endl;
            return 1;
        }
    }

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        auto& sysFSI = std::dynamic_pointer_cast<CRMTerrain>(rig.GetTerrain())->GetSystemFSI();
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle("Tire Test Rig on CRM deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(1.0, 2.5, 1.0), ChVector3d(0, 1, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->SetLightIntensity(0.7);
        visFSI->SetLightDirection(CH_PI_2, CH_PI / 6);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->EnableFlexBodyMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->AttachSystem(sys);
        visFSI->Initialize();
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
    timer.start();
    while (time < sim_time_max) {
        time = sys->GetChTime();

        if ((debug_output || gnuplot_output) && time >= time_offset) {
            long_slip_fct.AddPoint(time, tire->GetLongitudinalSlip());
            slip_angle_fct.AddPoint(time, tire->GetSlipAngle() * CH_RAD_TO_DEG);
            camber_angle_fct.AddPoint(time, tire->GetCamberAngle() * CH_RAD_TO_DEG);
        }

        if (time >= render_frame / render_fps) {
            auto& loc = rig.GetPos();
            visFSI->UpdateCamera(loc + ChVector3d(1.0, 2.5, 0.5), loc + ChVector3d(0, 0.25, -0.25));

            if (!visFSI->Render())
                break;
            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << snap_dir << "/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }
            render_frame++;

#ifdef CHRONO_POSTPROCESS
            if (blender_output)
                blender_exporter.ExportData();
#endif
        }

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
            cout << "\rRTF: " << sys->GetRTF();
        }
    }
    timer.stop();

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
