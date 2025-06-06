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
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// -----------------------------------------------------------------------------

const double sphere_radius = 0.0125;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

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
//------------------------------------------------------------------
// Function to generate a UV sphere mesh and save to VTK file
//------------------------------------------------------------------
void WriteSphereVTK(const std::string& filename, std::shared_ptr<ChBody> body, double radius, int resolution = 16) {
    // Generate a sphere mesh with UV coordinates
    ChTriangleMeshConnected mesh;
    std::vector<ChVector3d>& vertices = mesh.GetCoordsVertices();
    std::vector<ChVector3i>& indices = mesh.GetIndicesVertexes();

    // Create vertices using spherical coordinates
    for (int i = 0; i <= resolution; i++) {
        double phi = CH_PI * i / resolution;
        for (int j = 0; j <= resolution; j++) {
            double theta = 2 * CH_PI * j / resolution;
            double x = radius * sin(phi) * cos(theta);
            double y = radius * sin(phi) * sin(theta);
            double z = radius * cos(phi);
            vertices.push_back(ChVector3d(x, y, z));
        }
    }

    // Create triangular faces
    for (int i = 0; i < resolution; i++) {
        for (int j = 0; j < resolution; j++) {
            int p1 = i * (resolution + 1) + j;
            int p2 = p1 + 1;
            int p3 = (i + 1) * (resolution + 1) + j;
            int p4 = p3 + 1;

            // Add two triangles for each grid cell
            indices.push_back(ChVector3i(p1, p2, p3));
            indices.push_back(ChVector3i(p2, p4, p3));
        }
    }

    // Now write the mesh to VTK file, transforming it to the body's position
    std::ofstream outf(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "Sphere VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;

    // Write vertices transformed by body frame
    ChFrame<> frame = body->GetFrameRefToAbs();
    outf << "POINTS " << vertices.size() << " float" << std::endl;
    for (const auto& v : vertices) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }

    // Write triangular cells
    int nf = static_cast<int>(indices.size());
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (const auto& f : indices) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }

    // Write cell types (5 = VTK_TRIANGLE)
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5" << std::endl;
    }
    outf.close();
}

int main(int argc, char* argv[]) {
    // Default values
    double t_end = 2.0;
    bool verbose = true;
    bool output = true;
    bool write_marker_files = false;
    double output_fps = 400;
    bool snapshots = false;
    int ps_freq = 1;
    double sphere_density = 700;
    double Hdrop = 0.2;
    bool render = false;
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
    ChFsiFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1510;
    mat_props.Young_modulus = 2e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.3;
    mat_props.mu_fric_2 = 0.48;
    mat_props.average_diam = 0.001;
    mat_props.cohesion_coeff = 0;  // default

    sysSPH.SetElasticSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = d0_multiplier;
    sph_params.artificial_viscosity = 0.01;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
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
    double init_spacing = initial_spacing;
    ChVector3d cMin(-bxDim / 2 - 3 * init_spacing, -byDim / 2 - 3 * init_spacing, -bzDim * 1.2);
    ////ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + Hdrop + sphere_radius + init_spacing) * 1.2);
    ChVector3d cMax(bxDim / 2 + 3 * init_spacing, byDim / 2 + 3 * init_spacing,
                    (bzDim + sphere_radius + init_spacing) * 1.2);
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::NONE);

    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(init_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - init_spacing, byDim / 2 - init_spacing, fzDim / 2 - init_spacing);
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

    ////double sphere_z_pos = Hdrop + fzDim + sphere_radius + 0.5 * init_spacing;
    ////double sphere_z_vel = 0;

    double sphere_z_pos = fzDim + sphere_radius + 0.5 * init_spacing;
    double sphere_z_vel = impact_vel;

    auto sphere = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(sphere);
    sphere->SetPos(ChVector3d(0, 0, sphere_z_pos));
    sphere->SetPosDt(ChVector3d(0, 0, -sphere_z_vel));
    sphere->SetMass(mass);
    sphere->SetInertia(inertia);

    chrono::utils::AddSphereGeometry(sphere.get(), cmaterial, sphere_radius);
    sphere->GetCollisionModel()->SetSafeMargin(init_spacing);

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
        ss << viscosity_type << "_" << boundary_type;
        ss << "_ps" << ps_freq;
        ss << "_d" << sphere_density;
        ss << "_h" << Hdrop;
        ss << "_s" << init_spacing;
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
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, impact_vel / 2);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Cratering");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -3 * byDim, 0.75 * bzDim), ChVector3d(0, 0, 0.75 * bzDim));
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
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();
    
    double rtf_average = 0.0;
    unsigned int rtf_count = 0;

    std::string out_file = out_dir + "/sphere_penetration_depth.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                std::stringstream vtk_filename;
                vtk_filename << out_dir << "/vtk/sphere_" << std::setw(5) << std::setfill('0') << out_frame << ".vtk";
                WriteSphereVTK(vtk_filename.str(), sphere, sphere_radius);
            }
            out_frame++;
        }

        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Write penetration depth to file
        double d_pen = fzDim + sphere_radius + 0.5 * init_spacing - sphere->GetPos().z();
        ofile << time << " " << d_pen << " " << sphere->GetPos().x() << " " << sphere->GetPos().y() << " "
              << sphere->GetPos().z() << " " << sphere->GetPosDt().x() << " " << sphere->GetPosDt().y() << " "
              << sphere->GetPosDt().z() << std::endl;
        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        double rtf = sysFSI.GetRtf();
        rtf_average = (rtf_average * rtf_count + rtf) / (rtf_count + 1);
        rtf_count++;
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;
    std::cout << "Average RTF: " << rtf_average << std::endl;
    std::cout << "Simulation finished" << std::endl;
    
    // Write runtime information to a file
    if (output) {
        std::ofstream runtime_file(out_dir + "/runtime.txt");
        runtime_file << "Runtime: " << timer() << " seconds\n" << std::endl;
        runtime_file << "Simulation time: " << time << std::endl;
        runtime_file << "Average RTF: " << rtf_average << std::endl;
        runtime_file << "ps_freq: " << ps_freq << std::endl;
        runtime_file << "Simulation finished";
        runtime_file.close();
    }

    return 0;
}