// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Luning Bakke, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Final simulation time
double t_end = 10.0;
//double initial_spacing = 0.01;
double initial_spacing = 0.01;

// Position and dimensions of WEC device
ChVector3d wec_pos(-1.5, 0, 0.33);

// Container dimensions
ChVector3d csize(5.0, 1.2, 0.6);  // original size????

// Beach start location
double x_start = csize.x() / 2;
std::string mesh_obj_filename = GetChronoDataFile("models/flap_round.obj");

double pto_damping = 0;

// Fluid depth
// double depth = 1.3;

// this is for testing wec deivce
double depth = 0.44;

// Output frequency
bool output = true;
double output_fps = 5;
bool save_csv = true;

// write info frequency
double csv_fps = 100;

// Enable/disable run-time visualization
bool render = false;
float render_fps = 20;

// Enable saving snapshots
bool snapshots = false;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y >= 0; }
};
#endif

// -----------------------------------------------------------------------------

class WaveFunction : public ChFunction {
  public:
    WaveFunction() : delay(0), a2(0), omega(0) {}
    WaveFunction(double delay, double amplitude, double frequency)
        : delay(delay), a2(amplitude / 2), omega(CH_2PI * frequency) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        if (t <= delay)
            return 0;
        double tt = t - delay;
        return a2 * (1 - std::cos(omega * tt));
    }

  private:
    double delay;
    double a2;
    double omega;
};

class WaveFunctionDecay : public ChFunction {
  public:
    // stroke s0, period T, with an exponential decay
    WaveFunctionDecay() : a2(0.1), omega(1) {}
    WaveFunctionDecay(double s0, double frequency) : a2(s0 / 2), omega(CH_2PI * frequency) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        double T = CH_2PI / omega;
        return a2 * (1 - std::exp(-t / T)) * (1 - std::cos(2. * CH_PI / T * t));
    }

  private:
    double a2;     // stroke
    double omega;  // period
};

std::shared_ptr<ChLinkMotorRotationTorque> CreateFlap(ChFsiProblemSPH& fsi, double prescribed_torque, bool use_bce) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Common contact material and geometry
    ChContactMaterialData cmat;
    cmat.Y = 1e8f;
    cmat.mu = 0.2f;
    cmat.cr = 0.05f;

    // TODO: use multiple chbodies

    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(cmat);
    // geometry->coll_boxes.push_back(
    //     utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0.5 * wec_size.z()), QUNIT, wec_size, 0));

    double door_thickness = 0.14;
    double door_height = 0.46;
    double door_width = 0.853;


    ChVector3d box_size(door_thickness, door_width, door_height);
    // location of front box -y
    ChVector3d box_pos(0, 0, 0);

    if (use_bce) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_obj_filename, true, false);
        geometry->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, Q_ROTATE_Y_TO_Z, mesh_obj_filename, VNULL, 1.0, 0.01, 0));
    }

    else {
        geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(box_pos, QUNIT, box_size, 0));
            std::cout << "Add front box at location " << box_pos << ", size of : " << box_size
                      << " and initial spacing of: " << initial_spacing << std::endl;
    }




    auto flap = chrono_types::make_shared<ChBody>();
    flap->SetName("WEC flap");
    flap->SetPos(wec_pos);
    flap->SetRot(QUNIT);
    flap->SetFixed(false);

    double wec_mass = 24.5;
    flap->SetMass(wec_mass);
    flap->SetInertiaXX(ChVector3d(0.76, 0.76, 0.76));

    std::cout << "wec_inertia: " << std::endl << flap->GetInertiaXX() << std::endl;
    sysMBS.AddBody(flap);
    if (show_rigid)
        geometry->CreateVisualizationAssets(flap, VisualizationType::COLLISION);

    fsi.AddRigidBody(flap, geometry, true, true);

    // add ground
    auto ground = chrono_types::make_shared<ChBody>();

    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    ChVector3d joint_pos(-1.5, 0, 0.0);

    // add motor to the body
    auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    motor->Initialize(ground, flap, ChFrame<>(joint_pos, Q_ROTATE_Z_TO_Y));

    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, prescribed_torque), 1);   // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(prescribed_torque), t_end);  // 0.0

    motor->SetTorqueFunction(f_segment);
    sysMBS.AddLink(motor);

    return motor;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double step_size = 1e-4;  // used to be 5e-5!
    bool verbose = true;

    // take input of prescribed angle from 0.1 all the way to 0.6
    double prescribed_torque = atof(argv[1]);

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemWavetank fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    sph_params.d0_multiplier = 1.2;
    sph_params.max_velocity = 4;
    sph_params.shifting_method = ShiftingMethod::DIFFUSION;
    sph_params.shifting_diffusion_A = 1.0;
    sph_params.shifting_diffusion_AFST = 2.0;
    sph_params.shifting_diffusion_AFSM = 3.0;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.artificial_viscosity = 0.02;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;

    sph_params.num_bce_layers = 5;
    fsi.SetSPHParameters(sph_params);
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Create WEC device
    std::shared_ptr<ChLinkMotorRotationTorque> revolute = CreateFlap(fsi, -prescribed_torque, true);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(depth));

    // Create a wave tank
    double stroke = 0; 

    double frequency = 1.0;
    auto fun = chrono_types::make_shared<WaveFunctionDecay>(0, frequency);

    ////fsi.SetProfile(chrono_types::make_shared<WaveTankBezierBeach>(x_start), false);
    fsi.SetProfile(chrono_types::make_shared<WaveTankRampBeach>(x_start, 0.2), true);

    auto body = fsi.ConstructWaveTank(ChFsiProblemWavetank::WavemakerType::PISTON,  //
                                      ChVector3d(0, 0, 0), csize, depth,            //
                                      fun);                                         //

    // Initialize the FSI system
    fsi.Initialize();

    // Create oputput directories
    std::string out_dir =
        GetChronoOutputPath() + "Flap_Buoyance_torque_" + argv[1] + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    //out_dir = out_dir + fsi.GetSphIntegrationSchemeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (save_csv) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            cerr << "Error creating directory " << out_dir + "/particles" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            cerr << "Error creating directory " << out_dir + "/fsi" << endl;
            return 1;
        }
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        cerr << "Error creating directory " << out_dir + "/vtk" << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
        cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
        return 1;
    }

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/wave_fun.gpl");
    gplot.SetGrid();
    std::string speed_title = "Wave function";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(*fun, 0, 5, 0.02, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    ////fsi.SaveInitialMarkers(out_dir);

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(-1000, 12000, true);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::RED_BLUE);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("WEC Device");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(wec_pos + ChVector3d(0, -9 * csize.y(), 0), wec_pos);
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int csv_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    double motor_torque;
    double flap_angular_velo;  // pitch velo
    double pto_power;
    ChVector3 flap_pos;
    ChQuaterniond flap_quat;

    std::ofstream ofile;
    ofile.open(out_dir + "/info.csv");
    ofile << "time,torque,angle" << std::endl;

    while (time < t_end) {
        if (save_csv && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");
            out_frame++;
        }

        if (output && time >= csv_frame / csv_fps) {
            // get the reaction force
            motor_torque = revolute->GetMotorTorque(); 
            flap_pos = sysMBS.GetBodies()[1]->GetPos();
            flap_quat = sysMBS.GetBodies()[1]->GetRot();

            ofile << "time, " << time << "," << -motor_torque << ","
                      << -revolute->GetMotorAngle() << std::endl;

            csv_frame++;
        }

        // Render FSI system
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif

        // Call the FSI solver
        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    ofile.close();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    return 0;
}
