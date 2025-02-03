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
// Author: Huzaifa Mustafa Unjhawala
// =============================================================================
// Cone Penetrometer Validation Problem involving immersing a cone at a specified velocity
// and measureing the force on the cone tip
// Reference Paper for Cohesive soil:
// https://www.sciencedirect.com/science/article/pii/S0022489815000865
// Thesis reference for similar test in GRC-1
// https://uwmadison.box.com/s/606t8ppn48kpp5y53c6wp5tftjm27sfb - Page 58
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFluidSystemSPH.h"
// #include "chrono_fsi/sph/ChFsiProblemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;

#ifdef CHRONO_VSG
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
#endif

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};

// -----------------------------------------------------------------------------
// Material properties of the 15 materials
std::vector<double> G_shears = {5.21e6, 5.10e6, 5.21e6, 5.10e6, 5.10e6, 4.79e6, 4.72e6, 4.90e6,
                                4.72e6, 4.96e6, 5.03e6, 5.27e6, 5.34e6, 4.90e6, 4.90e6};  // in Pa
// std::vector<double> G_shears = {5.21e5, 5.10e6, 5.21e6, 5.10e6, 5.10e6, 4.79e6, 4.72e6, 4.90e6,
//                                 4.72e6, 4.96e6, 5.03e6, 5.27e6, 5.34e6, 4.90e6, 4.90e6};  // in Pa
double nu_poisson = 0.495;
// double nu_poisson = 0.3;
std::vector<double> cohesions = {25.5e3, 13.8e3, 22.1e3, 14.5e3, 20.7e3, 15.9e3, 9.65e3, 17.9e3,
                                 13.1e3, 26.9e3, 30.3e3, 34.5e3, 37.9e3, 35.2e3, 24.8e3};  // In Pa
std::vector<double> densities = {1827, 1799, 1827, 1799, 1799, 1688, 1661, 1716,
                                 1661, 1744, 1771, 1855, 1882, 1716, 1716};  // In kg/m^3

// Cone material
struct solid_material {
    double youngs_modulus = 193e9;
    double friction_coefficient = 0.7;
    double restitution = 0.05;
    double adhesion = 0;
};

// Add cone properties struct near material properties
struct ConeProperties {
    double length = 0.03782;    // 3.782 cm
    double diameter = 0.02027;  // 2.027 cm
    double mass = 0.03245;      // 32.45g
};

// -----------------------------------------------------------------------------

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;

    // Physics parameters
    double mu_s;
    double mu_2;
    double artificial_viscosity;
    double penetration_depth;

    // Output/rendering parameters
    bool verbose;
    bool output;
    double output_fps;
    bool snapshots;
    bool render;
    double render_fps;
    bool write_marker_files;
};
void SimulateMaterial(int i, const SimParams& params, const ConeProperties& coneProp);
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
    cli.AddOption<double>("Physics", "mu_s", "Friction at no plastic flow", std::to_string(params.mu_s));
    cli.AddOption<double>("Physics", "mu_2", "Max friction after initiation of plastic flow",
                          std::to_string(params.mu_2));

    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(params.artificial_viscosity));
    cli.AddOption<double>("Physics", "penetration_depth",
                          "Penetration depth at which we would like to take the readings",
                          std::to_string(params.penetration_depth));

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
    params.mu_s = cli.GetAsType<double>("mu_s");
    params.mu_2 = cli.GetAsType<double>("mu_2");
    params.penetration_depth = cli.GetAsType<double>("penetration_depth");
    return true;
}

int main(int argc, char* argv[]) {
    ConeProperties coneProp;  // Create cone instance

    SimParams params = {/*ps_freq*/ 1,
                        /*initial_spacing*/ 0.002,
                        /*d0_multiplier*/ 1.3,
                        /*time_step*/ 5e-5,
                        /*boundary_type*/ "adami",
                        /*viscosity_type*/ "artificial_bilateral",
                        /*kernel_type*/ "cubic",
                        /*mu_s*/ 0.1,
                        /*mu_2*/ 0.1,
                        /*artificial_viscosity*/ 0.2,
                        /*penetration_depth*/ coneProp.length + 0,  // Now uses cone struct
                        /*verbose*/ true,
                        /*output*/ true,
                        /*output_fps*/ 100,
                        /*snapshots*/ true,
                        /*render*/ true,
                        /*render_fps*/ 400,
                        /*write_marker_files*/ false};

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
    std::cout << "mu_s: " << params.mu_s << std::endl;
    std::cout << "mu_2: " << params.mu_2 << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;
    std::cout << "penetration_depth: " << params.penetration_depth << std::endl;

    int num_materials = 15;
    for (int i = 0; i < num_materials; i++) {
        SimulateMaterial(i, params, coneProp);
    }
}

void SimulateMaterial(int i, const SimParams& params, const ConeProperties& coneProp) {
    double penetration_velocity = 0.00254;  // 0.1 inch/s
    double t_end = (coneProp.length + params.penetration_depth) / penetration_velocity + 1;
    std::cout << "t_end: " << t_end << std::endl;

    double container_diameter = 0.394 / 2;  // Actual dimension is 39.4 cm
    double container_height = 0.178 / 2;    // Actual dimension is 17.8 cm

    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);
    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.81);
    sysFSI.SetGravitationalAcceleration(gravity);

    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    // -------------------------------------------------------------------------
    // Material and SPH parameters
    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFluidSystemSPH::SPHParameters sph_params;

    mat_props.density = densities[i];
    mat_props.Young_modulus = G_shears[i] * 2 * (1 + nu_poisson);
    mat_props.Poisson_ratio = nu_poisson;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = params.mu_s;
    mat_props.mu_fric_2 = params.mu_2;
    mat_props.average_diam = 0.002;
    mat_props.cohesion_coeff = cohesions[i];

    sysSPH.SetElasticSPH(mat_props);

    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = params.initial_spacing;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.xsph_coefficient = 0.5;
    sph_params.shifting_coefficient = 1.0;
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

    sysSPH.SetSPHParameters(sph_params);
    // -------------------------------------------------------------------------

    // ==============================
    // Create container and granular material
    // ==============================
    // Create a container
    // sand clearance
    double clearance = 0.2 * container_height;
    double bxDim = container_diameter;
    double byDim = container_diameter;
    double fzDim = container_height;
    double bzDim = fzDim + clearance;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + 0.05 + params.initial_spacing) * 1.2);
    sysSPH.SetBoundaries(cMin, cMax);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Create granular material
    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(params.initial_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - params.initial_spacing, byDim / 2 - params.initial_spacing,
                          fzDim / 2 - params.initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-p.z() + fzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0));
    }

    solid_material solid_mat;
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(solid_mat.youngs_modulus);
    cmaterial->SetFriction(solid_mat.friction_coefficient);
    cmaterial->SetRestitution(solid_mat.restitution);
    cmaterial->SetAdhesion(solid_mat.adhesion);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);
    // Add BCE particles attached on the walls into FSI system
    sysSPH.AddBoxContainerBCE(box,                                            //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 2, -1));

    // ==============================
    // Create cone
    // ==============================

    auto cone = chrono_types::make_shared<ChBody>();
    cone->SetPos(ChVector3d(0, 0, bzDim - clearance + coneProp.length));
    cone->SetPosDt(ChVector3d(0, 0, -penetration_velocity));
    cone->SetMass(coneProp.mass / 2);
    ChQuaternion<> cone_rot = Q_FLIP_AROUND_X;
    cone->SetRot(cone_rot);

    double volume = ChCone::GetVolume(coneProp.length, coneProp.diameter);
    double cone_mass = coneProp.mass;
    // Mass/2 because we also have a cylinder that takes up half the mass
    ChMatrix33<> inertia = cone_mass / 2 * ChCone::GetGyration(coneProp.length, coneProp.diameter);
    cone->SetInertia(inertia);
    cone->SetFixed(false);
    sysMBS.AddBody(cone);

    chrono::utils::AddConeGeometry(cone.get(), cmaterial, coneProp.diameter / 2, coneProp.length);
    cone->GetCollisionModel()->SetSafeMargin(params.initial_spacing);

    sysFSI.AddFsiBody(cone);
    sysSPH.AddConeBCE(cone, ChFrame<>(VNULL, QUNIT), coneProp.diameter / 2, coneProp.length, true, true);

    // Add cylinder on top of cone like in the experiment
    // This is to prevent soild from falling on top of the cone, pushing it down
    auto cyl = chrono_types::make_shared<ChBody>();
    double cyl_length = 0.1;
    double cyl_radius = coneProp.diameter / 2 - 0.2 * (coneProp.diameter / 2);
    cyl->SetPos(ChVector3d(0, 0, cone->GetPos().z() + cyl_length / 2));
    cyl->SetRot(ChQuaternion<>(1, 0, 0, 0));
    cyl->SetMass(coneProp.mass / 2);
    sysMBS.AddBody(cyl);
    chrono::utils::AddCylinderGeometry(cyl.get(), cmaterial, cyl_radius, cyl_length);
    cyl->GetCollisionModel()->SetSafeMargin(params.initial_spacing);
    sysFSI.AddFsiBody(cyl);
    // The subtraction to prevent overlap from the cone
    sysSPH.AddCylinderBCE(cyl, ChFrame<>(VNULL, QUNIT), cyl_radius, cyl_length - 2 * params.initial_spacing, true,
                          true);
    // Constraint cylinder to cone
    auto constraint = chrono_types::make_shared<ChLinkLockLock>();
    constraint->Initialize(cone, cyl, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(constraint);

    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (params.output || params.snapshots) {
        // Base output directory
        std::string base_dir = GetChronoOutputPath() + "FSI_ConePenetrometer/";
        if (!filesystem::create_directory(filesystem::path(base_dir))) {
            std::cerr << "Error creating directory " << base_dir << std::endl;
            return;
        }

        // Create nested directories
        std::stringstream penetration_depth_stream;
        penetration_depth_stream << std::fixed << std::setprecision(3) << params.penetration_depth;

        std::vector<std::string> subdirs = {"penetrationDepth_" + penetration_depth_stream.str(), "material_" + i,
                                            "boundaryType_" + params.boundary_type,
                                            "viscosityType_" + params.viscosity_type,
                                            "kernelType_" + params.kernel_type};

        for (const auto& subdir : subdirs) {
            base_dir += subdir + "/";
            if (!filesystem::create_directory(filesystem::path(base_dir))) {
                std::cerr << "Error creating directory " << base_dir << std::endl;
                return;
            }
        }

        // Add flat structure
        std::stringstream ss;
        ss << "ps_" << params.ps_freq;
        ss << "_s_" << params.initial_spacing;
        ss << "_d0_" << params.d0_multiplier;
        ss << "_t_" << params.time_step;
        ss << "_av_" << params.artificial_viscosity;
        out_dir = base_dir + ss.str();

        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return;
        }

        if (params.output) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
                std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
                return;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
                std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
                return;
            }
        }

        if (params.snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return;
            }
        }
    }

    // Create a run-time visualizer
#ifdef CHRONO_VSG
    std::shared_ptr<ChFsiVisualization> visFSI;
    if (params.render) {
        visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    }

    auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2);

    visFSI->SetTitle("FSI Cone Penetrometer");
    visFSI->SetSize(1280, 720);
    visFSI->AddCamera(ChVector3d(0, -3 * container_height, 0.75 * container_height),
                      ChVector3d(0, 0, 0.75 * container_height));
    visFSI->SetCameraMoveScale(0.1f);
    visFSI->SetLightIntensity(0.9);
    visFSI->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(true);
    visFSI->EnableRigidBodyMarkers(true);
    visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetSPHColorCallback(col_callback);
    visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
    visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
    visFSI->AttachSystem(&sysMBS);
    visFSI->Initialize();
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();

    std::string out_file = out_dir + "/force_vs_time.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Add comma-separated header to the output file
    ofile << "Time,Force-x,Force-y,Force-z,position-x,position-y,penetration-depth" << std::endl;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Calculate current penetration depth
        double current_depth = -(cone->GetPos().z() - (bzDim - clearance + coneProp.length));

        // Check if penetration depth is reached within tolerance
        const double depth_tolerance = 1e-6;  // Small tolerance for floating-point comparison
        if (std::abs(current_depth - params.penetration_depth) < depth_tolerance) {
            std::cout << "Penetration depth reached" << std::endl;
            std::cout << "Current depth: " << current_depth << std::endl;
            std::cout << "Target depth: " << params.penetration_depth << std::endl;
            std::cout << "Time: " << time << std::endl;
            std::cout << "Cone now fixed" << std::endl;
            cone->SetPosDt(ChVector3d(0, 0, 0));
        }
        if (params.output && time >= out_frame / params.output_fps) {
            if (params.write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
            }
            out_frame++;
        }
#ifdef CHRONO_VSG
        if (params.render && time >= render_frame / params.render_fps) {
            if (!visFSI->Render())
                break;

            if (params.snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif
        ofile << time << "," << cone->GetAppliedForce().x() << "," << cone->GetAppliedForce().y() << ","
              << cone->GetAppliedForce().z() << "," << cone->GetPos().x() << "," << cone->GetPos().y() << ","
              << current_depth << std::endl;
        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    ofile.close();
}
