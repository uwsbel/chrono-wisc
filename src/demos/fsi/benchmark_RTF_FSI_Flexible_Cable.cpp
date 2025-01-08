// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Wei Hu, Pei Li, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif
#include "chrono_fsi/sph/utils/ChUtilsTimingOutput.h"
#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------
// Physics problem type
PhysicsProblem problem_type = PhysicsProblem::CRM;
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Dimension of the domain
double smalldis = 1.0e-9;
double bxDim = 3.0 + smalldis;
double byDim = 0.2 + smalldis;
double bzDim = 2.0 + smalldis;

// Dimension of the fluid domain
double fxDim = 1.0 + smalldis;
double fyDim = 0.2 + smalldis;
double fzDim = 1.0 + smalldis;

// Dimension of the cable
double length_cable = 0.8 + smalldis;
double loc_x = -0.3;
int num_cable_element = 15;

// Material Properties
double E = 8e9;
double density = 8000;
double BeamRayleighDamping = 0.02;

// RTF benchmark so set this always to false
bool output = false;
double output_fps = 20;
// Set to true only for debugging - Run benchmark with render = false
bool render = false;
double render_fps = 400;
// Set to true only for debugging - Run benchmark with snapshots = false
bool snapshots = false;
// Only prints at initialization so can be kept at true without affecting performance
bool verbose = true;

// -----------------------------------------------------------------------------

std::shared_ptr<fea::ChMesh> Create_MB_FE(ChSystemSMC& sysMBS, ChFsiSystemSPH& sysFSI);
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     int& ps_freq,
                     std::string& boundary_type,
                     std::string& viscosity_type,
                     double& d0_multiplier);
// -----------------------------------------------------------------------------

class PositionVisibilityCallback : public ChParticleCloud::VisibilityCallback {
  public:
    PositionVisibilityCallback() {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        auto p = cloud.GetParticlePos(n);
        return p.y() > 0;
    };
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Parse command line arguments
    double t_end = 2.0;

    int ps_freq = 1;
    std::string boundary_type = "adami";
    std::string viscosity_type =
        (problem_type == PhysicsProblem::CFD) ? "artificial_unilateral" : "artificial_bilateral";
    double d0_multiplier = 1.2;
    if (!GetProblemSpecs(argc, argv, t_end, ps_freq, boundary_type, viscosity_type, d0_multiplier)) {
        return 1;
    }

    double initial_spacing = (problem_type == PhysicsProblem::CFD) ? 0.02 : 0.01;

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetVerbose(verbose);
    // Set integration step size
    double step_size = (problem_type == PhysicsProblem::CFD) ? 2e-5 : 2.5e-4;
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    // Set fluid phase properties
    switch (problem_type) {
        case PhysicsProblem::CFD: {
            ChFluidSystemSPH::FluidProperties fluid_props;
            fluid_props.density = 1000;
            fluid_props.viscosity = 5.0;

            sysSPH.SetCfdSPH(fluid_props);

            break;
        }
        case PhysicsProblem::CRM: {
            ChFluidSystemSPH::ElasticMaterialProperties mat_props;
            mat_props.density = 1700;
            mat_props.Young_modulus = 1e6;
            mat_props.Poisson_ratio = 0.3;
            mat_props.mu_I0 = 0.03;
            mat_props.mu_fric_s = 0.5;
            mat_props.mu_fric_2 = 0.5;
            mat_props.average_diam = 0.005;
            mat_props.cohesion_coeff = 0;

            sysSPH.SetElasticSPH(mat_props);

            break;
        }
    }

    // Set SPH solution parameters
    ChFluidSystemSPH::SPHParameters sph_params;

    switch (problem_type) {
        case PhysicsProblem::CFD:
            sph_params.sph_method = SPHMethod::WCSPH;
            sph_params.initial_spacing = initial_spacing;
            sph_params.h_multiplier = d0_multiplier;
            sph_params.max_velocity = 10;
            sph_params.xsph_coefficient = 0.5;
            sph_params.shifting_coefficient = 0.0;
            sph_params.kernel_threshold = 0.8;
            sph_params.artificial_viscosity = 0.02;
            sph_params.use_delta_sph = true;
            sph_params.delta_sph_coefficient = 0.1;
            sph_params.num_proximity_search_steps = ps_freq;

            break;

        case PhysicsProblem::CRM:
            sph_params.sph_method = SPHMethod::WCSPH;
            sph_params.initial_spacing = initial_spacing;
            sph_params.h_multiplier = d0_multiplier;
            sph_params.xsph_coefficient = 0.5;
            sph_params.shifting_coefficient = 1.0;
            sph_params.kernel_threshold = 0.8;
            sph_params.artificial_viscosity = 0.5;
            sph_params.num_proximity_search_steps = ps_freq;

            break;
    }

    if (boundary_type == "holmes")
        sph_params.boundary_type = BoundaryType::HOLMES;
    else
        sph_params.boundary_type = BoundaryType::ADAMI;

    if (viscosity_type == "laminar")
        sph_params.viscosity_type = ViscosityType::LAMINAR;
    else if (viscosity_type == "artificial_bilateral")
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    else
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;

    sysSPH.SetSPHParameters(sph_params);

    // Set simulation domain
    sysSPH.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    auto initSpace0 = sysSPH.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-5 * bxDim, -byDim / 2.0 - initSpace0 / 2.0, -5 * bzDim);
    ChVector3d cMax = ChVector3d(5 * bxDim, byDim / 2.0 + initSpace0 / 2.0, 5 * bzDim);
    sysSPH.SetBoundaries(cMin, cMax);

    // Set SPH discretization type, consistent or inconsistent
    sysSPH.SetConsistentDerivativeDiscretization(false, false);

    // Create SPH particles of fluid region
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - initSpace0, fyDim / 2, fzDim / 2 - initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysSPH.AddSPHParticle(points[i]);
    }
    // Create solids
    auto mesh = Create_MB_FE(sysMBS, sysFSI);

    // Initialize FSI system
    sysFSI.Initialize();

    SetChronoOutputPath("BENCHMARK1_RTF/");
    // Create oputput directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Cable/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphMethodTypeString() + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            cerr << "Error creating directory " << out_dir + "/particles" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            cerr << "Error creating directory " << out_dir + "/fsi" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            cerr << "Error creating directory " << out_dir + "/vtk" << endl;
            return 1;
        }
    }

    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
            return 1;
        }
    }

    // Create a run-tme visualizer
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
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
                visFSI->AddCamera(ChVector3d(0, -2, 0.75), ChVector3d(0, 0, 0.75));
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
                visFSI->AddCamera(ChVector3d(0, -3, 0.75), ChVector3d(0, 0, 0.75));
#endif
                break;
            }
        }

        visFSI->SetTitle("Chrono::FSI flexible cable");
        visFSI->SetSize(1280, 720);
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableFlexBodyMarkers(true);
        visFSI->SetColorFlexBodyMarkers(ChColor(1, 1, 1));
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, 2.5));
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<PositionVisibilityCallback>());
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

// Set MBS solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sysMBS.SetSolver(mkl_solver);
#else
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sysMBS.SetSolver(solver);
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);
#endif

    // Simulation loop
    double dT = sysFSI.GetStepSizeCFD();
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    // Initial position of top most node
    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(0));
    std::cout << "Initial position of top node: " << node->GetPos().x() << " " << node->GetPos().y() << " "
              << node->GetPos().z() << std::endl;
    ChVector3d init_pos = node->GetPos();

    // Reset all the timers
    sysFSI.GetFluidSystemSPH().ResetTimers();
    double timer_step = 0;
    double timer_CFD = 0;
    double timer_MBS = 0;
    double timer_FSI = 0;
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysSPH.PrintParticleToFile(out_dir + "/particles");
            sysSPH.PrintFsiInfoToFile(out_dir + "/fsi", time);

            std::ostringstream filename;
            filename << out_dir << "/vtk/flex_body." << std::setw(5) << std::setfill('0') << out_frame + 1 << ".vtk";
            fea::ChMeshExporter::WriteFrame(mesh, out_dir + "/Flex_MESH.vtk", filename.str());

            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        sysFSI.DoStepDynamics(dT);

        time += dT;
        sim_frame++;
        timer_step += sysFSI.GetTimerStep();
        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBS += sysFSI.GetTimerMBS();
        timer_FSI += sysFSI.GetTimerFSI();
    }
    // Create Output JSON file
    rapidjson::Document doc;
    // Generate filename
    std::ostringstream d0_str;
    d0_str << std::fixed << std::setprecision(1) << d0_multiplier;
    std::string d0_formatted = d0_str.str();
    d0_formatted.erase(d0_formatted.find_last_not_of('0') + 1, std::string::npos);
    if (d0_formatted.back() == '.')
        d0_formatted.pop_back();
    std::string json_file_path = out_dir + "/rtf_" + viscosity_type + "_" + boundary_type + "_ps" +
                                 std::to_string(ps_freq) + "_d0" + d0_formatted + ".json";
    OutputParameterJSON(json_file_path, &sysFSI, t_end, step_size, viscosity_type, boundary_type, ps_freq,
                        d0_multiplier, doc);
    OutputTimingJSON(json_file_path, timer_step, timer_CFD, timer_MBS, timer_FSI, &sysFSI, doc);

    return 0;
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<fea::ChMesh> Create_MB_FE(ChSystemSMC& sysMBS, ChFsiSystemSPH& sysFSI) {
    ChFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();

    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sysMBS.AddBody(ground);

    // FSI representation of walls
    sysSPH.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 0, -1));

    auto initSpace0 = sysSPH.GetInitialSpacing();

    // Create an FEA mesh representing a cantilever beam modeled with ANCF cable elements
    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    std::vector<std::vector<int>> _1D_elementsNodes_mesh;

    auto msection_cable = chrono_types::make_shared<ChBeamSectionCable>();
    msection_cable->SetDiameter(initSpace0);
    msection_cable->SetYoungModulus(E);
    msection_cable->SetDensity(density);
    msection_cable->SetRayleighDamping(BeamRayleighDamping);

    ChBuilderCableANCF builder;
    std::vector<std::vector<int>> node_nbrs;
    builder.BuildBeam(mesh,                                  // FEA mesh with nodes and elements
                      msection_cable,                        // section material for cable elements
                      num_cable_element,                     // number of elements in the segment
                      ChVector3d(loc_x, 0.0, length_cable),  // beam start point
                      ChVector3d(loc_x, 0.0, initSpace0),    // beam end point
                      _1D_elementsNodes_mesh,                // node indices
                      node_nbrs                              // neighbor node indices
    );

    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(builder.GetLastBeamNodes().back());
    auto pos_const = chrono_types::make_shared<ChLinkNodeFrame>();
    pos_const->Initialize(node, ground);
    sysMBS.Add(pos_const);

    auto dir_const = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    dir_const->Initialize(node, ground);
    dir_const->SetDirectionInAbsoluteCoords(node->GetSlope1());
    sysMBS.Add(dir_const);

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    // Add the mesh to the FSI system (only these meshes interact with the fluid)
    sysSPH.SetBcePattern1D(BcePatternMesh1D::STAR, false);
    sysFSI.AddFsiMesh(mesh);

    return mesh;
}

// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     int& ps_freq,
                     std::string& boundary_type,
                     std::string& viscosity_type,
                     double& d0_multiplier) {
    ChCLI cli(argv[0], "Flexible cable FSI demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]");
    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search");
    cli.AddOption<std::string>("Physics", "boundary_type", "Boundary condition type (holmes/adami)");
    cli.AddOption<std::string>("Physics", "viscosity_type",
                               "Viscosity type (laminar/artificial_unilateral/artificial_bilateral)");
    cli.AddOption<double>("Physics", "d0_multiplier", "SPH density multiplier");

    if (argc == 1) {
        cout << "Required parameters missing. See required parameters and descriptions below:\n\n";
        cli.Help();
        return false;
    }

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    t_end = cli.GetAsType<double>("t_end");
    ps_freq = cli.GetAsType<int>("ps_freq");
    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    d0_multiplier = cli.GetAsType<double>("d0_multiplier");

    return true;
}