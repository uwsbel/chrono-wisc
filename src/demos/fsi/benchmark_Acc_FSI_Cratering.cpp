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
// Author: Wei Hu, Huzaifa Mustafa Unjhawala
// =============================================================================
// Cratering validation problem involving spherical impactors with different
// densities falling from different heights (zero velocity) on CRM soil.
//
// Reference solution:
// https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff
//
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

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

const double sphere_radius = 0.0125;

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};

// -----------------------------------------------------------------------------

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     int& ps_freq,
                     double& sphere_density,
                     double& Hdrop,
                     double& initial_spacing,
                     double& d0_multiplier,
                     double& time_step,
                     std::string& boundary_type,
                     std::string& viscosity_type,
                     std::string& kernel_type) {
    ChCLI cli(argv[0], "FSI Cratering Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<double>("Geometry", "sphere_density", "Sphere density", std::to_string(sphere_density));
    cli.AddOption<double>("Geometry", "Hdrop", "Drop height", std::to_string(Hdrop));
    cli.AddOption<double>("Geometry", "initial_spacing", "Initial spacing", std::to_string(initial_spacing));
    cli.AddOption<double>("Physics", "d0_multiplier", "D0 multiplier", std::to_string(d0_multiplier));

    cli.AddOption<std::string>("Physics", "boundary_type", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", "artificial_unilateral");
    cli.AddOption<std::string>("Physics", "kernel_type", "Kernel type (cubic/wendland)", "cubic");
    cli.AddOption<double>("Physics", "time_step", "Time step", std::to_string(time_step));
    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    ps_freq = cli.GetAsType<int>("ps_freq");
    sphere_density = cli.GetAsType<double>("sphere_density");
    Hdrop = cli.GetAsType<double>("Hdrop");
    initial_spacing = cli.GetAsType<double>("initial_spacing");
    d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    time_step = cli.GetAsType<double>("time_step");
    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    kernel_type = cli.GetAsType<std::string>("kernel_type");
    return true;
}

int main(int argc, char* argv[]) {
    // Default values
    double t_end = 2.0;
    bool verbose = true;
    bool output = true;
    double output_fps = 100;
    bool snapshots = true;
    int ps_freq = 1;
    double sphere_density = 700;
    double Hdrop = 0.5;
    bool render = true;
    double render_fps = 400;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_unilateral";
    double initial_spacing = 0.0025;
    double d0_multiplier = 1.3;
    double time_step = 5e-5;
    std::string kernel_type = "cubic";

    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, t_end, ps_freq, sphere_density, Hdrop, initial_spacing, d0_multiplier, time_step,
                         boundary_type, viscosity_type, kernel_type)) {
        return 1;
    }

    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1510;
    mat_props.Young_modulus = 2e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.3;
    mat_props.mu_fric_2 = 0.48;
    mat_props.average_diam = 0.001;
    mat_props.cohesion_coeff = 0;  // default

    sysSPH.SetElasticSPH(mat_props);

    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = d0_multiplier;
    sph_params.artificial_viscosity = 0.01;
    sph_params.xsph_coefficient = 0.5;
    sph_params.shifting_coefficient = 1.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.num_proximity_search_steps = ps_freq;
    if (kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    } else {
        std::cerr << "Invalid kernel type: " << kernel_type << std::endl;
        return 1;
    }

    // Set boundary type
    if (boundary_type == "holmes") {
        sph_params.boundary_type = BoundaryType::HOLMES;
    } else {
        sph_params.boundary_type = BoundaryType::ADAMI;
    }

    // Set viscosity type
    if (viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    }

    sysSPH.SetSPHParameters(sph_params);

    double g = 9.81;
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, -g));
    sysMBS.SetGravitationalAcceleration(sysSPH.GetGravitationalAcceleration());

    sysFSI.SetVerbose(verbose);
    sysSPH.SetNumProximitySearchSteps(ps_freq);

    // Dimension of the space domain
    double bxDim = 0.14;
    double byDim = 0.1;
    double fzDim = 0.15;
    double bzDim = fzDim;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ////ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + Hdrop + sphere_radius + initial_spacing) * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + sphere_radius + initial_spacing) * 1.2);
    sysSPH.SetBoundaries(cMin, cMax);

    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(initial_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - initial_spacing, byDim / 2 - initial_spacing, fzDim / 2 - initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles to the fluid system
    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-p.z() + fzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0));
    }

    // Create MBD and BCE particles for the solid domain
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.3f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

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

    // Create a falling sphere
    double volume = ChSphere::GetVolume(sphere_radius);
    double mass = sphere_density * volume;
    auto inertia = mass * ChSphere::GetGyration(sphere_radius);
    double impact_vel = std::sqrt(2 * Hdrop * g);

    ////double sphere_z_pos = Hdrop + fzDim + sphere_radius + 0.5 * initial_spacing;
    ////double sphere_z_vel = 0;

    double sphere_z_pos = fzDim + sphere_radius + 0.5 * initial_spacing;
    double sphere_z_vel = impact_vel;

    auto sphere = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(sphere);
    sphere->SetPos(ChVector3d(0, 0, sphere_z_pos));
    sphere->SetPosDt(ChVector3d(0, 0, -sphere_z_vel));
    sphere->SetMass(mass);
    sphere->SetInertia(inertia);

    chrono::utils::AddSphereGeometry(sphere.get(), cmaterial, sphere_radius);
    sphere->GetCollisionModel()->SetSafeMargin(initial_spacing);

    sysFSI.AddFsiBody(sphere);
    sysSPH.AddSphereBCE(sphere, ChFrame<>(VNULL, QUNIT), sphere_radius, true, true);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (output || snapshots) {
        out_dir = GetChronoOutputPath() + "FSI_Cratering/";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        std::stringstream ss;
        ss << viscosity_type << "_" << boundary_type << "_" << kernel_type;
        ss << "_ps" << ps_freq;
        ss << "_d" << sphere_density;
        ss << "_h" << Hdrop;
        ss << "_s" << initial_spacing;
        ss << "_d0" << d0_multiplier;
        ss << "_t" << time_step;
        out_dir = out_dir + ss.str();
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
        }

        if (snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
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

        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, impact_vel / 2);

        visFSI->SetTitle("FSI Cratering");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, -3 * byDim, 0.75 * bzDim), ChVector3d(0, 0, 0.75 * bzDim));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->SetLightIntensity(0.9);
        visFSI->SetLightDirection(-CH_PI_2, CH_PI / 6);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();

    std::string out_file = out_dir + "/sphere_penetration_depth.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            out_frame++;
        }

        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Write penetration depth to file
        double d_pen = fzDim + sphere_radius + 0.5 * initial_spacing - sphere->GetPos().z();
        ofile << time << " " << d_pen << " " << sphere->GetPos().x() << " " << sphere->GetPos().y() << " "
              << sphere->GetPos().z() << " " << sphere->GetPosDt().x() << " " << sphere->GetPosDt().y() << " "
              << sphere->GetPosDt().z() << std::endl;
        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        time += dT;

        sim_frame++;
    }
    timer.stop();
    // Create Output JSON file
    rapidjson::Document doc;
    sysSPH.OutputParameterJSON(out_dir + "/parameters.json", t_end, time_step, viscosity_type, boundary_type, ps_freq,
                               d0_multiplier, doc);
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}