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
// Cone Penetration Validation Problem involving a cone falling from a height
// onto a soil surface.
// Reference solution:
// https://sbel.wisc.edu/documents/TR-2016-04.pdf
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

#ifdef CHRONO_VSG
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
#endif
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};

struct sand_material {
    double density_max = 1780;
    double density_min = 1520;
    double Young_modulus = 2e6;
    double Poisson_ratio = 0.3;
    double mu_I0 = 0.04;
    double mu_fric_s = 0.7;
    double mu_fric_2 = 0.7;
    double average_diam = 0.0007;
    double cohesion_coeff = 0;
};

struct bead_material {
    double density_max = 1630;
    double density_min = 1500;
    double Young_modulus = 2e6;
    double Poisson_ratio = 0.3;
    double mu_I0 = 0.04;
    double mu_fric_s = 0.7;
    double mu_fric_2 = 0.7;
    double average_diam = 0.003;
    double cohesion_coeff = 0;
};

struct cone_60 {
    double length = 0.02210;
    double diameter = 0.01986;
    double mass = 0.1357;
};

struct cone_30 {
    double length = 0.03436;
    double diameter = 0.00921;
    double mass = 0.1411;
};

struct solid_material {
    double youngs_modulus = 193e9;
    double friction_coefficient = 0.7;
    double restitution = 0.05;
    double adhesion = 0;
};

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     int& ps_freq,
                     double& initial_spacing,
                     double& d0_multiplier,
                     double& time_step,
                     std::string& boundary_type,
                     std::string& viscosity_type,
                     std::string& kernel_type,
                     std::string& gran_material,
                     int& rel_density,
                     int& cone_type,
                     double& container_depth,
                     double& Hdrop,
                     double& artificial_viscosity) {
    ChCLI cli(argv[0], "FSI Cone Penetration Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(time_step));

    cli.AddOption<std::string>("Physics", "gran_material", "Granular material type (sand/bead)", gran_material);
    cli.AddOption<int>("Geometry", "rel_density", "Relative density(0/1)",
                       std::to_string(rel_density));  // See linked document at top to see what this means
    cli.AddOption<int>("Geometry", "cone_type", "Cone type (1 - 30 degrees/2 - 60 degrees)", std::to_string(cone_type));
    cli.AddOption<double>("Geometry", "container_depth", "Container depth (m)", std::to_string(container_depth));
    cli.AddOption<double>("Geometry", "Hdrop", "Drop height (times cone length - 0/0.5/1)", std::to_string(Hdrop));
    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(artificial_viscosity));

    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    ps_freq = cli.GetAsType<int>("ps_freq");
    initial_spacing = cli.GetAsType<double>("initial_spacing");
    d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    time_step = cli.GetAsType<double>("time_step");
    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    kernel_type = cli.GetAsType<std::string>("kernel_type");
    gran_material = cli.GetAsType<std::string>("gran_material");
    rel_density = cli.GetAsType<int>("rel_density");
    cone_type = cli.GetAsType<int>("cone_type");
    container_depth = cli.GetAsType<double>("container_depth");
    Hdrop = cli.GetAsType<double>("Hdrop");
    artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    return true;
}

void CalculateConeProperties(const double length,
                             const double diameter,
                             const double mass,
                             double Hdrop,
                             double g,
                             double fzDim,
                             double initial_spacing,
                             double& volume,
                             double& cone_mass,
                             ChMatrix33<>& inertia,
                             double& cone_z_pos,
                             double& cone_z_vel,
                             double& cone_length,
                             double& cone_diameter) {
    volume = ChCone::GetVolume(length, diameter);
    cone_mass = mass;
    inertia = mass * ChCone::GetGyration(length, diameter);
    double impact_vel = std::sqrt(2 * Hdrop * length * g);
    cone_z_pos = fzDim + length + 0.5 * initial_spacing;
    cone_z_vel = impact_vel;
    cone_length = length;
    cone_diameter = diameter;
}

int main(int argc, char* argv[]) {
    double t_end = 1.0;
    int ps_freq = 1;
    double initial_spacing = 0.002;
    double d0_multiplier = 1.3;
    double time_step = 5e-5;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_unilateral";
    std::string kernel_type = "cubic";

    bool verbose = true;
    bool output = true;
    double output_fps = 100;
    bool snapshots = true;
    bool render = true;
    double render_fps = 400;

    std::string gran_material = "sand";  // This can also be "bead"
    int rel_density = 0;                 // This means that the density is rho_min of measured - if 0, then its rho_max
    int cone_type = 1;                   // This means 30 deg cone
    double container_depth = 0.1;        // This is  in meters
    double Hdrop = 0.5;                  // This is 0.5 times length of cone
    double artificial_viscosity = 0.5;
    if (!GetProblemSpecs(argc, argv, t_end, ps_freq, initial_spacing, d0_multiplier, time_step, boundary_type,
                         viscosity_type, kernel_type, gran_material, rel_density, cone_type, container_depth, Hdrop,
                         artificial_viscosity)) {
        return 1;
    }

    bool write_marker_files = false;
    // Print all problem specs
    std::cout << "Problem Specs:" << std::endl;
    std::cout << "t_end: " << t_end << std::endl;
    std::cout << "ps_freq: " << ps_freq << std::endl;
    std::cout << "initial_spacing: " << initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << d0_multiplier << std::endl;
    std::cout << "time_step: " << time_step << std::endl;
    std::cout << "boundary_type: " << boundary_type << std::endl;
    std::cout << "viscosity_type: " << viscosity_type << std::endl;
    std::cout << "kernel_type: " << kernel_type << std::endl;
    std::cout << "gran_material: " << gran_material << std::endl;
    std::cout << "rel_density: " << rel_density << std::endl;
    std::cout << "cone_type: " << cone_type << std::endl;
    std::cout << "container_depth: " << container_depth << std::endl;
    std::cout << "Hdrop: " << Hdrop << std::endl;
    std::cout << "artificial_viscosity: " << artificial_viscosity << std::endl;
    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFluidSystemSPH::SPHParameters sph_params;

    if (gran_material == "sand") {
        sand_material sand_mat;
        mat_props.density = (sand_mat.density_max - sand_mat.density_min) * rel_density + sand_mat.density_max;
        mat_props.Young_modulus = sand_mat.Young_modulus;
        mat_props.Poisson_ratio = sand_mat.Poisson_ratio;
        mat_props.mu_I0 = sand_mat.mu_I0;
        mat_props.mu_fric_s = sand_mat.mu_fric_s;
        mat_props.mu_fric_2 = sand_mat.mu_fric_2;
        mat_props.average_diam = sand_mat.average_diam;
        mat_props.cohesion_coeff = sand_mat.cohesion_coeff;  // default
    } else if (gran_material == "bead") {
        bead_material bead_mat;
        mat_props.density = (bead_mat.density_max - bead_mat.density_min) * rel_density + bead_mat.density_max;
        mat_props.Young_modulus = bead_mat.Young_modulus;
        mat_props.Poisson_ratio = bead_mat.Poisson_ratio;
        mat_props.mu_I0 = bead_mat.mu_I0;
        mat_props.mu_fric_s = bead_mat.mu_fric_s;
        mat_props.mu_fric_2 = bead_mat.mu_fric_2;
        mat_props.average_diam = bead_mat.average_diam;
        mat_props.cohesion_coeff = bead_mat.cohesion_coeff;  // default
    } else {
        std::cerr << "Invalid gran_material: " << gran_material << std::endl;
        return 1;
    }
    sysSPH.SetElasticSPH(mat_props);

    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = d0_multiplier;
    sph_params.artificial_viscosity = artificial_viscosity;
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

    // ==============================
    // Create container and granular material
    // ==============================
    // Create a container
    // sand clearance
    double clearance = 0.2 * container_depth;
    double bxDim = 0.1;
    double byDim = 0.1;
    double fzDim = container_depth;
    double bzDim = fzDim + clearance;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + 0.05 + initial_spacing) * 1.2);
    sysSPH.SetBoundaries(cMin, cMax);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Create granular material
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

    // Set Material
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
    double volume, mass;
    ChMatrix33<> inertia;
    double cone_z_pos, cone_z_vel, cone_length, cone_diameter;
    if (cone_type == 1) {
        cone_30 cone_30;
        CalculateConeProperties(cone_30.length, cone_30.diameter, cone_30.mass, Hdrop, g, fzDim, initial_spacing,
                                volume, mass, inertia, cone_z_pos, cone_z_vel, cone_length, cone_diameter);

    } else if (cone_type == 2) {
        cone_60 cone_60;
        CalculateConeProperties(cone_60.length, cone_60.diameter, cone_60.mass, Hdrop, g, fzDim, initial_spacing,
                                volume, mass, inertia, cone_z_pos, cone_z_vel, cone_length, cone_diameter);
    }

    auto cone = chrono_types::make_shared<ChBody>();

    cone->SetPos(ChVector3d(0, 0, cone_z_pos));
    cone->SetPosDt(ChVector3d(0, 0, -cone_z_vel));
    cone->SetMass(mass);
    ChQuaternion<> cone_rot = Q_FLIP_AROUND_X;
    cone->SetRot(cone_rot);
    cone->SetInertia(inertia);
    sysMBS.AddBody(cone);
    chrono::utils::AddConeGeometry(cone.get(), cmaterial, cone_diameter / 2, cone_length);
    cone->GetCollisionModel()->SetSafeMargin(initial_spacing);

    sysFSI.AddFsiBody(cone);
    sysSPH.AddConeBCE(cone, ChFrame<>(VNULL, QUNIT), cone_diameter / 2, cone_length, true, true);

    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (output || snapshots) {
        // Base output directory
        std::string base_dir = GetChronoOutputPath() + "FSI_ConePenetration/";
        if (!filesystem::create_directory(filesystem::path(base_dir))) {
            std::cerr << "Error creating directory " << base_dir << std::endl;
            return 1;
        }

        // Create nested directories
        std::stringstream hdrop_stream;
        hdrop_stream << std::fixed << std::setprecision(1) << Hdrop;

        std::vector<std::string> subdirs = {"Hdrop_" + hdrop_stream.str(),
                                            "granMaterial_" + gran_material,
                                            "relDensity_" + std::to_string(rel_density),
                                            "coneType_" + std::to_string(cone_type),
                                            "boundaryType_" + boundary_type,
                                            "viscosityType_" + viscosity_type,
                                            "kernelType_" + kernel_type};

        for (const auto& subdir : subdirs) {
            base_dir += subdir + "/";
            if (!filesystem::create_directory(filesystem::path(base_dir))) {
                std::cerr << "Error creating directory " << base_dir << std::endl;
                return 1;
            }
        }

        // Add flat structure
        std::stringstream ss;
        ss << "ps_" << ps_freq;
        ss << "_s_" << initial_spacing;
        ss << "_d0_" << d0_multiplier;
        ss << "_t_" << time_step;
        ss << "_av_" << artificial_viscosity;
        out_dir = base_dir + ss.str();

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
#ifdef CHRONO_VSG
    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    }

    auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2);

    visFSI->SetTitle("FSI Cone Penetration");
    visFSI->SetSize(1280, 720);
    visFSI->AddCamera(ChVector3d(0, -3 * byDim, 0.75 * bzDim), ChVector3d(0, 0, 0.75 * bzDim));
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

    std::string out_file = out_dir + "/cone_penetration_depth.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Add comma-separated header to the output file
    ofile << "Time,PenetrationDepth,ConePosX,ConePosY,ConePosZ,ConeVelX,ConeVelY,ConeVelZ" << std::endl;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
            }
            out_frame++;
        }
#ifdef CHRONO_VSG
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
#endif
        // Write penetration depth to file
        // This is done so as to match the penetration depth in the experiment - see
        // https://sbel.wisc.edu/documents/TR-2016-04.pdf
        double d_pen = fzDim + 0.5 * initial_spacing + cone_length - cone->GetPos().z();
        ofile << time << "," << d_pen << "," << cone->GetPos().x() << "," << cone->GetPos().y() << ","
              << cone->GetPos().z() << "," << cone->GetPosDt().x() << "," << cone->GetPosDt().y() << ","
              << cone->GetPosDt().z() << std::endl;
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
