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
// Author: Huzaifa Unjhawala
// =============================================================================

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "chrono_models/robot/moonranger/Moonranger.h"

#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/ChConfig.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::moonranger;

MoonrangerWheelType wheel_type = MoonrangerWheelType::RealWheel;

double smalldis = 1.0e-9;
// double bxDim = 4.0 + smalldis;
// double byDim = 0.6 + smalldis;
// double bzDim = 4.0 + smalldis;

double bxDim = 1.8 + smalldis;
double byDim = 0.6 * 2 + smalldis;
double bzDim = 0.15 + smalldis;
// Variables from DEM sim
double safe_x = 0.;
double z_adv_targ = 0.2;

double wheel_radius = 0.085;
double wheel_slip = 0.0;
double wheel_width = 0.06;
int grouser_num = 18;
std::string wheel_obj = "robot/moonranger/obj/moonranger_wheel.obj";

double total_time = 30.0;

// From the class
const double wheel_x = 0.2222;
const double wheel_y = 0.29207;
const double wheel_z = -0.1805;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_MoonRanger_DBP";

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

std::shared_ptr<Moonranger> rover;

// Helpers to write all the data required for slip and some other extra stuff
void WriteHeaders(std::ofstream& spring_InitPos,
                  std::ofstream& myFile,
                  std::ofstream& extraWheelInfo,
                  ChSystemSMC& sysMBS);

void WriteData(std::ofstream& myFile, std::ofstream& extraWheelInfo, std::shared_ptr<Moonranger> rover, double time);

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChFsiSystemSPH& sysFSI,
                      ChVector3d init_loc,
                      double wheel_AngVel,
                      bool render_wheel,
                      double wheel_radius,
                      double grouser_height,
                      double wheel_wide,
                      double grouser_wide,
                      int grouser_num,
                      double iniSpacing = 0.01) {
    ChFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();
    ChSystem& sysMBS = sysFSI.GetMultibodySystem();
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

    chrono::utils::AddBoxContainer(ground, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),          //
                                   ChVector3d(1.2 * bxDim, 1.2 * byDim, bzDim), 0.1,  //
                                   ChVector3i(2, 2, 2),                               //
                                   false);
    ground->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    sysSPH.AddBoxContainerBCE(ground,                                       //
                              ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),     //
                              ChVector3d(1.2 * bxDim, 1.2 * byDim, bzDim),  //
                              ChVector3i(2, 2, -1));
    // init_loc.z() is the height of the rover chassis from the ground - so we are compensating for it sinking by
    // grouser height. the 0.005 is the height from which the rover is dropped
    // double box_z_pos_above_zero = init_loc.z() - 0.005 - grouser_height;
    double box_z_pos_above_zero =
        0.331377;  // Emperically obtained height by running the simulation and seeing how much the rover body sinks
    // Add a box and attach a spring to it and the rover
    auto springBox =
        chrono_types::make_shared<ChBodyEasyBox>(0.01, 0.1, (box_z_pos_above_zero + bzDim / 2) * 2., 1000, true, true);
    springBox->SetPos(ChVector3d(-1.2 * bxDim / 2, 0., box_z_pos_above_zero));
    springBox->SetRot(ChQuaternion<>(1, 0, 0, 0));
    springBox->EnableCollision(false);
    springBox->SetFixed(true);
    springBox->AddVisualShape(
        chrono_types::make_shared<ChVisualShapeBox>(0.01, 0.1, (box_z_pos_above_zero + bzDim / 2) * 2));
    sysMBS.AddBody(springBox);

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

    // Manualy calculate inertias
    double wheel_mass = 0.028;
    ;  // From IRIS demo on Luning github
    double wheel_IYY = wheel_mass * wheel_radius * wheel_radius / 2.0;
    double wheel_IXX = wheel_mass / 12 * (3 * wheel_radius * wheel_radius + wheel_width * wheel_width);

    auto driver = chrono_types::make_shared<MoonrangerSpeedDriver>(0.0, wheel_AngVel, 0.0);
    rover = chrono_types::make_shared<Moonranger>(&sysMBS, wheel_type);
    rover->SetSpeedDriver(driver);

    rover->Initialize(ChFrame<>(init_loc, QUNIT));
    std::vector<std::shared_ptr<ChBodyAuxRef>> Wheels;
    Wheels.push_back(rover->GetWheel(MoonrangerWheelID::LF)->GetBody());
    Wheels.push_back(rover->GetWheel(MoonrangerWheelID::RF)->GetBody());
    Wheels.push_back(rover->GetWheel(MoonrangerWheelID::LB)->GetBody());
    Wheels.push_back(rover->GetWheel(MoonrangerWheelID::RB)->GetBody());

    auto Body_1 = rover->GetChassis()->GetBody();
    std::cout << "Rover mass: " << rover->GetRoverMass() << std::endl;
    std::cout << "Wheel mass: " << rover->GetWheelMass() << std::endl;
    std::cout << "Chassis Vel: " << Body_1->GetPosDt() << std::endl;

    // Create a spring between the box and the rover
    auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
    spring_1->Initialize(springBox, Body_1, true, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));
    spring_1->SetRestLength(abs(springBox->GetPos().x() - Body_1->GetPos().x()));
    spring_1->SetSpringCoefficient(200);
    spring_1->SetDampingCoefficient(0);
    sysMBS.AddLink(spring_1);

    spring_1->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.1, 80, 15));
    // Use custom function for grousers
    double inner_radius = wheel_radius;
    double outer_radius = wheel_radius + grouser_height;
    // Get wheel pos and also add the grosers and stuff to the fsi sysmte
    for (int i = 0; i < 4; i++) {
        std::cout << "Wheel Pos: " << Wheels[i]->GetPos() << std::endl;
        if (i == 1 || i == 3) {
            sysSPH.AddWheelBCE_Grouser(Wheels[i], ChFrame<>(VNULL, QuatFromAngleZ(CH_PI)), inner_radius,
                                       wheel_wide - iniSpacing, grouser_height, grouser_wide, grouser_num, false, true,
                                       false);  // last true is for hollow wheel
        } else {
            sysSPH.AddWheelBCE_Grouser(Wheels[i], ChFrame<>(VNULL, QUNIT), inner_radius, wheel_wide - iniSpacing,
                                       grouser_height, grouser_wide, grouser_num, false, true,
                                       false);  // last true is for hollow wheel
        }
        std::cout << "Wheel Vel: " << Wheels[i]->GetPosDt() << std::endl;
        sysFSI.AddFsiBody(Wheels[i]);
    }

    return;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& sim_number_str,
                     double& wheel_AngVel,
                     double& grouser_height,
                     double& grouser_width,  // New parameter
                     int& neighbor_search,
                     bool& verbose,
                     bool& output,
                     bool& render,
                     double& initial_spacing,
                     double& kernel_multiplier) {
    ChCLI cli(argv[0], "FSI Single Wheel Acceleration Benchmark");

    cli.AddOption<std::string>("Simulation", "s,sim_number", "Simulation number", "0");
    cli.AddOption<double>("Simulation", "w,angular_vel", "Wheel angular velocity", "0.0");
    cli.AddOption<double>("Simulation", "grouser_height", "Grouser height", "0.01");
    cli.AddOption<double>("Simulation", "grouser_width", "Grouser width", "0.0015");  // New option
    cli.AddOption<int>("Simulation", "n,neighbor_search", "Neighbor search steps", "1");
    cli.AddOption<double>("Simulation", "i,initial_spacing", "Initial particle spacing", "0.01");
    cli.AddOption<double>("Simulation", "k,kernel_multiplier", "Kernel length multiplier", "1.2");

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output", "false");
    cli.AddOption<bool>("Output", "o,output", "Enable output", "false");
    cli.AddOption<bool>("Visualization", "r,render", "Enable run-time visualization", "false");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    sim_number_str = cli.GetAsType<std::string>("sim_number");
    wheel_AngVel = cli.GetAsType<double>("angular_vel");
    grouser_height = cli.GetAsType<double>("grouser_height");
    grouser_width = cli.GetAsType<double>("grouser_width");  // New parameter
    neighbor_search = cli.GetAsType<int>("neighbor_search");
    initial_spacing = cli.GetAsType<double>("initial_spacing");
    kernel_multiplier = cli.GetAsType<double>("kernel_multiplier");

    verbose = !cli.GetAsType<bool>("quiet");
    output = cli.GetAsType<bool>("output");
    render = cli.GetAsType<bool>("render");
    std::cout << "Render: " << render << std::endl;
    return true;
}

void SaveWheelVTK(ChFsiSystemSPH& sysFSI, double mTime) {
    std::string wheel_dir = out_dir + "/wheel";
    std::string filename;
    static int frame_number = -1;
    frame_number++;

    // Create output directory for wheel VTK files if it doesn't exist
    if (!filesystem::create_directory(filesystem::path(wheel_dir))) {
        std::cerr << "Error creating directory " << wheel_dir << std::endl;
        return;
    }

    // Save the wheels to VTK files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetWheel(MoonrangerWheelID::LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetWheel(MoonrangerWheelID::RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetWheel(MoonrangerWheelID::LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetWheel(MoonrangerWheelID::RB)->GetBody();
        }

        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();
        if (i == 1 || i == 3) {
            body_rot.Cross(body_rot, QuatFromAngleZ(CH_PI));
        }

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = GetChronoDataFile(wheel_obj);
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));
        mmesh->RepairDuplicateVertexes(1e-9);

        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));

        filename = wheel_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
        std::ofstream file;
        file.open(filename);
        file << "# vtk DataFile Version 2.0" << std::endl;
        file << "VTK from simulation" << std::endl;
        file << "ASCII" << std::endl;
        file << "DATASET UNSTRUCTURED_GRID" << std::endl;
        auto nv = mmesh->GetCoordsVertices().size();
        file << "POINTS " << nv << " float" << std::endl;
        for (auto& v : mmesh->GetCoordsVertices())
            file << v.x() << " " << v.y() << " " << v.z() << std::endl;
        auto nf = mmesh->GetIndicesVertexes().size();
        file << "CELLS " << nf << " " << 4 * nf << std::endl;
        for (auto& f : mmesh->GetIndicesVertexes())
            file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
        file << "CELL_TYPES " << nf << std::endl;
        for (size_t ii = 0; ii < nf; ii++)
            file << "5 " << std::endl;
        file.close();
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Output frame:  " << frame_number << std::endl;
    std::cout << " Time:          " << mTime << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}

int main(int argc, char* argv[]) {
    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Set the simulation stepsize
    sysFSI.SetStepSizeCFD(2.5e-4);
    sysFSI.SetStepsizeMBD(2.5e-4);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Parse command-line arguments
    std::string sim_number_str;
    double wheel_AngVel;
    double grouser_height;
    double grouser_width;  // New parameter
    int neighbor_search;
    bool verbose;
    double initial_spacing;
    double kernel_multiplier = 1.2;
    // Save data as csv files to see the results off-line using Paraview
    bool output = false;
    double out_fps = 1;

    int print_fps = 100;

    // Enable/disable run-time visualization
    bool render = false;
    float render_fps = 100;

    if (!GetProblemSpecs(argc, argv, sim_number_str, wheel_AngVel, grouser_height, grouser_width, neighbor_search,
                         verbose, output, render, initial_spacing, kernel_multiplier)) {
        return 1;
    }
    sysFSI.SetVerbose(verbose);

    std::cout << "Sim Number: " << sim_number_str << std::endl;
    std::cout << "Wheel Angular Velocity: " << wheel_AngVel << std::endl;
    std::cout << "Grouser Height: " << grouser_height << std::endl;
    std::cout << "Grouser Width: " << grouser_width << std::endl;  // New output

    std::cout << "verbose: " << verbose << std::endl;
    out_dir = out_dir + std::to_string(neighbor_search) + "_0" + "/";

    if (output) {
        // Create oputput directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    out_dir = out_dir + sim_number_str + "/";

    // Output the result to verify
    std::cout << "Output directory: " << out_dir << std::endl;
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (output) {
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

        if (!filesystem::create_directory(filesystem::path(out_dir + "/images"))) {
            std::cerr << "Error creating directory " << out_dir + "/images" << std::endl;
            return 1;
        }
    }

    sysSPH.ReadParametersFromFile(
        GetChronoDataFile("fsi/input_json/demo_FSI_SingleWheelTest_MoonRanger_Granular.json"));
    sysSPH.SetNumProximitySearchSteps(neighbor_search);
    ChVector3d active_domain_box(0.3, 0.12, 0.3);
    sysSPH.SetActiveDomain(active_domain_box);
    sysSPH.SetActiveDomainDelay(1.0);

    // Set initial spacing and kernel length after reading JSON file
    sysSPH.SetInitialSpacing(initial_spacing);
    sysSPH.SetKernelMultiplier(kernel_multiplier);

    double iniSpacing = sysSPH.GetInitialSpacing();

    // Set the terrain container size
    sysSPH.SetContainerDim(ChVector3d(1.2 * bxDim, 1.2 * byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    // This is where that G matrix is set
    sysSPH.SetConsistentDerivativeDiscretization(false, false);

    // Set the SPH method
    sysSPH.SetSPHMethod(SPHMethod::WCSPH);
    sysSPH.SetOutputLevel(OutputLevel::CRM_FULL);

    sysSPH.SetBoundaryType(BoundaryType::ADAMI);
    sysSPH.SetKernelType(KernelType::CUBIC_SPLINE);
    sysSPH.SetViscosityType(ViscosityType::ARTIFICIAL_BILATERAL);

    // Set up the periodic boundary condition
    // Since we are using an active domain, the periodic domain can be exactly the same as the container
    ChVector3d cMin(-(1.2 * bxDim) / 2 - 20 * iniSpacing, -(1.5 * byDim) / 2 - 20 * iniSpacing,
                    -bzDim - 30 * iniSpacing);
    ChVector3d cMax((1.2 * bxDim) / 2 + 20 * iniSpacing, (1.5 * byDim) / 2 + 20 * iniSpacing,
                    2 * bzDim + 30 * iniSpacing);
    sysSPH.SetBoundaries(cMin, cMax);

    // Initialize the SPH particles
    chrono::utils::ChGridSampler<> sampler(iniSpacing);
    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(bxDim / 2 - iniSpacing, byDim / 2 - iniSpacing, bzDim / 2 - iniSpacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = (int)points.size();

    double gz = gravity.z();
    double atmospheric_pressure = 0;
    for (int i = 0; i < numPart; i++) {
        double pre_ini =
            sysSPH.GetDensity() * std::abs(gz) * (-(points[i].z() + bzDim / 2) + bzDim) + atmospheric_pressure;
        sysSPH.AddSPHParticle(points[i], sysSPH.GetDensity(), 0, sysSPH.GetViscosity(),
                              ChVector3d(0),         // initial velocity
                              ChVector3d(-pre_ini),  // tauxxyyzz
                              ChVector3d(0)          // tauxyxzyz
        );
    }

    // Rover initial location
    ChVector3d init_loc(-bxDim / 2.0 + wheel_x * 2, 0, bzDim / 2.0 + grouser_height + wheel_radius - wheel_z + 0.005);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysFSI, init_loc, wheel_AngVel, render, wheel_radius, grouser_height, wheel_width, grouser_width,
                     grouser_num, iniSpacing);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    double dT = sysFSI.GetStepSizeCFD();

#ifdef CHRONO_VSG
    ChVisualSystem::Type vis_type;
    // Set up real-time visualization of the FSI system
    vis_type = ChVisualSystem::Type::VSG;
    std::shared_ptr<ChFsiVisualization> visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    if (render) {
        visFSI->SetTitle("Chrono::CRM single wheel test");
        visFSI->SetSize(720, 720);
        visFSI->AddCamera(ChVector3d(-bxDim / 2. + 1, -5 * byDim, 5 * bzDim), ChVector3d(-bxDim / 2. + 1, 0., 0));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetImageOutputDirectory(out_dir + "images");
        visFSI->SetImageOutput(1);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }
#endif

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int print_steps = (unsigned int)round(1 / (print_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    ChTimer timer;
    timer.start();

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream extraWheelInfo;
    std::ofstream spring_InitPos;
    // if (output) {
    myFile.open(out_dir + "/results.txt", std::ios::trunc);
    extraWheelInfo.open(out_dir + "/extra_wheel_info.txt", std::ios::trunc);
    spring_InitPos.open(out_dir + "/spring_init_pos.txt", std::ios::trunc);
    // }

    // Call the function to write headers
    WriteHeaders(spring_InitPos, myFile, extraWheelInfo, sysMBS);
    while (time < total_time) {
        rover->Update();

        if (output && current_step % output_steps == 0) {
            std::cout << "rover->GetChassis()->GetBody()->GetPos(): " << rover->GetChassis()->GetBody()->GetPos()
                      << std::endl;
            std::cout << "-------- Output" << std::endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            SaveWheelVTK(sysFSI, time);  // Save wheel VTK files
        }
        // For now write all the data possible at every timestep
        WriteData(myFile, extraWheelInfo, rover, time);  // Write data to files

#ifdef CHRONO_VSG
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }
#endif
        // Call the FSI solver
        sysFSI.DoStepDynamics(dT);
        time += dT;
        current_step++;
    }

    timer.stop();
    std::cout << "Runtime: " << timer() << " seconds\n" << std::endl;
    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Simulation finished" << std::endl;

    return 0;
}

void WriteHeaders(std::ofstream& spring_InitPos,
                  std::ofstream& myFile,
                  std::ofstream& extraWheelInfo,
                  ChSystemSMC& sysMBS) {
    // Write initial box position
    auto spring_box = sysMBS.GetBodies()[1];
    spring_InitPos << "x"
                   << "\t"
                   << "y"
                   << "\t"
                   << "z" << std::endl;
    spring_InitPos << spring_box->GetPos().x() << "\t" << spring_box->GetPos().y() << "\t" << spring_box->GetPos().z();

    // Write header for results
    myFile << "time"
           << "\t"
           << "x"
           << "\t"
           << "y"
           << "\t"
           << "z"
           << "\t"
           << "vx"
           << "\t"
           << "vy"
           << "\t"
           << "vz" << std::endl;

    // Write header for extra wheel info
    extraWheelInfo << "time"
                   << "\t"
                   << "LF_omega_x"
                   << "\t"
                   << "LF_omega_y"
                   << "\t"
                   << "LF_omega_z"
                   << "\t"
                   << "RF_omega_x"
                   << "\t"
                   << "RF_omega_y"
                   << "\t"
                   << "RF_omega_z"
                   << "\t"
                   << "LB_omega_x"
                   << "\t"
                   << "LB_omega_y"
                   << "\t"
                   << "LB_omega_z"
                   << "\t"
                   << "RB_omega_x"
                   << "\t"
                   << "RB_omega_y"
                   << "\t"
                   << "RB_omega_z"
                   << "\t"
                   << "LF_torque_x"
                   << "\t"
                   << "LF_torque_y"
                   << "\t"
                   << "LF_torque_z"
                   << "\t"
                   << "RF_torque_x"
                   << "\t"
                   << "RF_torque_y"
                   << "\t"
                   << "RF_torque_z"
                   << "\t"
                   << "LB_torque_x"
                   << "\t"
                   << "LB_torque_y"
                   << "\t"
                   << "LB_torque_z"
                   << "\t"
                   << "RB_torque_x"
                   << "\t"
                   << "RB_torque_y"
                   << "\t"
                   << "RB_torque_z"
                   << "\t" << std::endl;
}

void WriteData(std::ofstream& myFile, std::ofstream& extraWheelInfo, std::shared_ptr<Moonranger> rover, double time) {
    // Write rover position and velocity
    ChVector3d rover_pos = rover->GetChassis()->GetBody()->GetPos();
    ChVector3d rover_vel = rover->GetChassis()->GetBody()->GetPosDt();
    myFile << time << "\t" << rover_pos.x() << "\t" << rover_pos.y() << "\t" << rover_pos.z() << "\t" << rover_vel.x()
           << "\t" << rover_vel.y() << "\t" << rover_vel.z() << std::endl;

    // Write wheel angular velocities and torques
    ChVector3d LF_ang_vel = rover->GetWheel(MoonrangerWheelID::LF)->GetBody()->GetAngVelLocal();
    ChVector3d RF_ang_vel = rover->GetWheel(MoonrangerWheelID::RF)->GetBody()->GetAngVelLocal();
    ChVector3d LB_ang_vel = rover->GetWheel(MoonrangerWheelID::LB)->GetBody()->GetAngVelLocal();
    ChVector3d RB_ang_vel = rover->GetWheel(MoonrangerWheelID::RB)->GetBody()->GetAngVelLocal();
    ChVector3d LF_torque = rover->GetWheelTracTorque(MoonrangerWheelID::LF);
    ChVector3d RF_torque = rover->GetWheelTracTorque(MoonrangerWheelID::RF);
    ChVector3d LB_torque = rover->GetWheelTracTorque(MoonrangerWheelID::LB);
    ChVector3d RB_torque = rover->GetWheelTracTorque(MoonrangerWheelID::RB);

    extraWheelInfo << time << "\t" << LF_ang_vel.x() << "\t" << LF_ang_vel.y() << "\t" << LF_ang_vel.z() << "\t"
                   << RF_ang_vel.x() << "\t" << RF_ang_vel.y() << "\t" << RF_ang_vel.z() << "\t" << LB_ang_vel.x()
                   << "\t" << LB_ang_vel.y() << "\t" << LB_ang_vel.z() << "\t" << RB_ang_vel.x() << "\t"
                   << RB_ang_vel.y() << "\t" << RB_ang_vel.z() << "\t" << LF_torque.x() << "\t" << LF_torque.y() << "\t"
                   << LF_torque.z() << "\t" << RF_torque.x() << "\t" << RF_torque.y() << "\t" << RF_torque.z() << "\t"
                   << LB_torque.x() << "\t" << LB_torque.y() << "\t" << LB_torque.z() << "\t" << RB_torque.x() << "\t"
                   << RB_torque.y() << "\t" << RB_torque.z() << std::endl;
}