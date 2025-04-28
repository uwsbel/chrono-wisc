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
// Author: Wei Hu, Radu Serban
// =============================================================================

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>

#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/ChConfig.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

#include <cuda_runtime.h>

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
double smalldis = 1.0e-9;
// double bxDim = 4.0 + smalldis;
// double byDim = 0.6 + smalldis;
// double bzDim = 4.0 + smalldis;

double bxDim = 5.0 + smalldis;
double byDim = 0.8 + smalldis;
double bzDim = 0.2 + smalldis;
// Variables from DEM sim
double safe_x = 1.;
double z_adv_targ = 0.2;

double wheel_radius = 0.225;
double wheel_slip = 0.0;
double wheel_width = 0.200;
double grouser_wide = 0.005;
int grouser_num = 24;
std::string wheel_obj = "robot/viper/obj/nasa_viper_wheel.obj";

double total_time = 10.0;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 100;
bool write_marker_files = true;
int print_fps = 100;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Viper_Single_Wheel2";

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;

    double artificial_viscosity;

    // Physical parameters of experiment
    double total_mass;
    double slope_angle;
    double wheel_AngVel;
    double gravity_G;
    double grouser_height;
    int sim_number;
};

// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "FSI Cone Penetrometer Demo");

    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(params.ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(params.initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(params.d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)",
                               params.boundary_type);
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", params.viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", params.kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));

    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(params.artificial_viscosity));

    cli.AddOption<double>("Physics", "total_mass", "Total mass", std::to_string(params.total_mass));
    cli.AddOption<double>("Physics", "slope_angle", "Slope angle", std::to_string(params.slope_angle));
    cli.AddOption<double>("Physics", "wheel_AngVel", "Wheel angular velocity", std::to_string(params.wheel_AngVel));
    cli.AddOption<double>("Physics", "gravity_G", "Gravity", std::to_string(params.gravity_G));
    cli.AddOption<double>("Physics", "grouser_height", "Grouser height", std::to_string(params.grouser_height));
    cli.AddOption<int>("Simulation", "sim_number", "Simulation number", std::to_string(params.sim_number));

    if (!cli.Parse(argc, argv))
        return false;

    params.ps_freq = cli.GetAsType<int>("ps_freq");
    params.initial_spacing = cli.GetAsType<double>("initial_spacing");
    params.d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    params.time_step = cli.GetAsType<double>("time_step");
    params.boundary_type = cli.GetAsType<std::string>("boundary_type");
    params.viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    params.kernel_type = cli.GetAsType<std::string>("kernel_type");
    params.artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    params.total_mass = cli.GetAsType<double>("total_mass");
    params.slope_angle = cli.GetAsType<double>("slope_angle");
    params.wheel_AngVel = cli.GetAsType<double>("wheel_AngVel");
    params.gravity_G = cli.GetAsType<double>("gravity_G");
    params.grouser_height = cli.GetAsType<double>("grouser_height");
    params.sim_number = cli.GetAsType<int>("sim_number");
    return true;
}

//------------------------------------------------------------------
// Function to save wheel to Paraview VTK files
//------------------------------------------------------------------
void WriteWheelVTK(const std::string& filename, ChTriangleMeshConnected& mesh, const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS,
                      ChFsiFluidSystemSPH& sysSPH,
                      ChFsiSystemSPH& sysFSI,
                      ChVector3d& wheel_IniPos,
                      ChVector3d& wheel_IniVel,
                      double wheel_AngVel,
                      bool render_wheel,
                      double grouser_height,
                      double kernelLength,
                      double total_mass,
                      double iniSpacing) {
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e9);
    cmaterial->SetFriction(1.f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0., 0., 0.));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetFixed(true);
    sysMBS.AddBody(ground);
    double multiplier = 1.0;
    chrono::utils::AddBoxContainer(ground, cmaterial,                                               //
                                   ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),                        //
                                   ChVector3d(multiplier * bxDim, multiplier * byDim, bzDim), 0.1,  //
                                   ChVector3i(2, 2, 2),                                             //
                                   false);
    ground->EnableCollision(true);

    // Add BCE particles attached on the walls into FSI system
    sysSPH.AddBoxContainerBCE(ground,                                                     //
                              ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),                   //
                              ChVector3d(multiplier * bxDim, multiplier * byDim, bzDim),  //
                              ChVector3i(2, 0, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0),
                       ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);         // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = 1500.;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);
    std::cout << "This is unused but... " << std::endl;
    std::cout << "Wheel Mass: " << mmass << std::endl;
    std::cout << "Wheel Density: " << mdensity << std::endl;
    std::cout << "Inertia Matrix: " << std::endl << minertia << std::endl;

    // Set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> wheel_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF
    // reference. Make the COG frame a principal frame.
    wheel->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(mdensity * principal_I);
    wheel->SetPosDt(wheel_IniVel);
    wheel->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // wheel material
    auto cmaterial2 = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial2->SetYoungModulus(1e9);
    cmaterial2->SetFriction(0.9f);
    cmaterial2->SetRestitution(0.4f);
    cmaterial2->SetAdhesion(0);
    // Set the absolute position of the body:
    wheel->SetFrameRefToAbs(ChFrame<>(ChVector3d(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    wheel->SetFixed(false);
    auto wheel_shape =
        chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial2, trimesh, false, false, 0.005);
    wheel->AddCollisionShape(wheel_shape);
    wheel->EnableCollision(false);
    if (render_wheel) {
        auto wheel_visual_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        wheel_visual_shape->SetMesh(trimesh);
        wheel->AddVisualShape(wheel_visual_shape);
    }

    sysMBS.AddBody(wheel);

    // Use custom function by Wei
    double inner_radius = wheel_radius;
    double outer_radius = wheel_radius + grouser_height;
    // Placeholder for BCE marker generation - will be updated in batch 2
    sysSPH.AddWheelBCE_Grouser(wheel, ChFrame<>(), inner_radius, wheel_width - iniSpacing, grouser_height, grouser_wide,
                               grouser_num, false);
    sysFSI.AddFsiBody(wheel);

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 3.0);
    chassis->SetPos(wheel->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    // Add geometry of the axle.
    // chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5,
    //                                  ChVector3d(0, 0, 0));
    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational
    // joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

int main(int argc, char* argv[]) {
    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    SimParams params = {/*ps_freq*/ 1,
                        /*initial_spacing*/ 0.01,
                        /*d0_multiplier*/ 1.2,
                        /*time_step*/ 2.5e-4,
                        /*boundary_type*/ "adami",
                        /*viscosity_type*/ "artificial_bilateral",
                        /*kernel_type*/ "cubic",
                        /*artificial_viscosity*/ 0.5,
                        /*total_mass*/ 17.5,
                        /*slope_angle*/ 0.0,
                        /*wheel_AngVel*/ 0.0,
                        /*gravity_G*/ 9.81,
                        /*grouser_height*/ 0.01,
                        /*sim_number*/ 0};

    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    std::cout << "Problem Specs:" << std::endl;

    std::cout << "ps_freq: " << params.ps_freq << std::endl;
    std::cout << "initial_spacing: " << params.initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << params.d0_multiplier << std::endl;
    std::cout << "time_step: " << params.time_step << std::endl;
    std::cout << "boundary_type: " << params.boundary_type << std::endl;
    std::cout << "viscosity_type: " << params.viscosity_type << std::endl;
    std::cout << "kernel_type: " << params.kernel_type << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;

    sysFSI.SetVerbose(verbose);

    std::cout << "Total Mass: " << params.total_mass << std::endl;
    std::cout << "Slope Angle: " << params.slope_angle << std::endl;
    std::cout << "Wheel Angular Velocity: " << params.wheel_AngVel << std::endl;
    std::cout << "Gravity Magnitude: " << params.gravity_G << std::endl;
    std::cout << "Grouser Height: " << params.grouser_height << std::endl;
    params.slope_angle = params.slope_angle / 180.0 * CH_PI;

    std::cout << "verbose: " << verbose << std::endl;

    // Create formatted output directory path with appropriate precision
    if (output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    std::stringstream ss;
    ss << std::fixed;
    ss << out_dir << "/ps_" << params.ps_freq;
    ss << "_s_" << std::setprecision(3) << params.initial_spacing;
    ss << "_d0_" << std::setprecision(1) << params.d0_multiplier;
    ss << "_av_" << std::setprecision(1) << params.artificial_viscosity << "/";
    out_dir = ss.str();

    if (output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    std::string sim_number_str = std::to_string(params.sim_number);
    out_dir = out_dir + sim_number_str + "/";

    // Output the result to verify
    std::cout << "Output directory: " << out_dir << std::endl;

    if (output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
            return 1;
        }
    }

    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    double gravity_G = -params.gravity_G;
    ChVector3d gravity = ChVector3d(gravity_G * sin(params.slope_angle), 0, gravity_G * cos(params.slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Set Parameters
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    mat_props.density = 1760;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.793;
    mat_props.mu_fric_2 = 0.793;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;

    sysSPH.SetElasticSPH(mat_props);

    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = params.initial_spacing;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.num_proximity_search_steps = params.ps_freq;

    if (params.kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (params.kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    }
    if (params.boundary_type == "holmes") {
        sph_params.boundary_type = BoundaryType::HOLMES;
    } else {
        sph_params.boundary_type = BoundaryType::ADAMI;
    }

    // Set viscosity type
    if (params.viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    }

    double iniSpacing = params.initial_spacing;
    double dT = params.time_step;
    double kernelLength = params.initial_spacing * params.d0_multiplier;

    // Initial Position of wheel
    ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius * 2.0, 0.0, wheel_radius + 10 * iniSpacing);
    ChVector3d wheel_IniVel(0.0, 0.0, 0.0);

    // Set the computational domain limits
    ChVector3d cMin(-bxDim / 2 - 3 * iniSpacing, -byDim / 2, -bzDim - 30 * iniSpacing);
    ChVector3d cMax(bxDim / 2 + 3 * iniSpacing, byDim / 2, bzDim + 30 * iniSpacing);
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::Y);

    // Initialize the SPH particles
    chrono::utils::ChGridSampler<> sampler(iniSpacing);
    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(bxDim / 2 - iniSpacing, byDim / 2 - iniSpacing, bzDim / 2 - iniSpacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = (int)points.size();
    double gz = params.gravity_G;
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysSPH.GetDensity() * gz * (-points[i].z() + bzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(points[i], rho_ini, pre_ini, sysSPH.GetViscosity());
    }

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysSPH, sysFSI, wheel_IniPos, wheel_IniVel, params.wheel_AngVel, render,
                     params.grouser_height, kernelLength, params.total_mass, params.initial_spacing);

    sysSPH.SetActiveDomain(ChVector3d(0.6, 0.6, 0.8));
    sysSPH.SetActiveDomainDelay(1.0);
    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];
    auto reaction = actuator->GetReaction2();
    ChVector3d force = reaction.force;
    ChVector3d torque = motor->GetReaction1().torque;
    ChVector3d w_pos = wheel->GetPos();
    ChVector3d w_vel = wheel->GetPosDt();
    ChVector3d angvel = wheel->GetAngVelLocal();

    // Save wheel mesh
    ChTriangleMeshConnected wheel_mesh;
    wheel_mesh.LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
    }

    myFile << "time\t"
           << "x\t"
           << "y\t"
           << "z\t"
           << "vx\t"
           << "vy\t"
           << "vz\t"
           << "ax\t"
           << "ay\t"
           << "az\t"
           << "fx\t"
           << "fy\t"
           << "fz\t"
           << "tx\t"
           << "ty\t"
           << "tz\n";

    myDBP_Torque << "time\t"
                 << "fx\t"
                 << "tz\n";

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;
    // Create a run-time visualizer
#ifdef CHRONO_VSG
    // Set up real-time visualization of the FSI system
    if (render) {
        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Chrono::CRM single wheel test");
        visVSG->SetWindowSize(720, 720);
        visVSG->AddCamera(ChVector3d(-bxDim / 2. + 1, -5 * byDim, 5 * bzDim), ChVector3d(-bxDim / 2. + 1, 0., 0));
        visVSG->SetLightIntensity(0.9);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int print_steps = (unsigned int)round(1 / (print_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    // Some things that even DEM domes
    double x1 = wheel->GetPos().x();
    double z1 = x1 * std::sin(params.slope_angle);
    double x2, z2, z_adv = 0.;
    int counter = 0;
    ChTimer timer;
    timer.start();
    while (time < total_time) {
        double adv = x2 - x1;
        if (current_step % print_steps == 0) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            // Compute slip
            // double linear_velocity = w_vel.x();
            double angular_velocity = angvel.z();
            // double slip = 1 - (std::abs(linear_velocity) / (std::abs(angular_velocity) * (wheel_radius +
            // grouser_height))); Compute slip like DEM
            double slip = 1 - (adv / (std::abs(angular_velocity) * (wheel_radius + params.grouser_height) * time));
            // Print slip
            std::cout << "Slip: " << slip << std::endl;

            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        if (z_adv >= z_adv_targ) {
            break;
        }

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";
            if (write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
                WriteWheelVTK(filename, wheel_mesh, wheel->GetFrameRefToAbs());
            }
        }

        // Render SPH particles
#ifdef CHRONO_VSG
        if (render && current_step % render_steps == 0) {
            if (!vis->Run())
                break;
            vis->Render();
        }
#endif

        // Get the infomation of the wheel
        reaction = actuator->GetReaction2();
        force = reaction.force;
        torque = motor->GetReaction1().torque;
        w_pos = wheel->GetPos();
        w_vel = wheel->GetPosDt();
        angvel = wheel->GetAngVelLocal();

        x2 = w_pos.x();
        z_adv = x2 * std::sin(params.slope_angle) - z1;
        if (x2 > safe_x) {
            break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(dT);
        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "Runtime: " << timer() << " seconds\n" << std::endl;
    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Simulation finished" << std::endl;

    if (output) {
        myFile.close();
        myDBP_Torque.close();
    }

    return 0;
}