// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Huzaifa Unjhawala
// =============================================================================
//
// ART Vehicle on CRM terrain pulling a load
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;
using namespace chrono::vehicle::artcar;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// ===================================================================================================================
// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 2;
double terrain_width = 1;
double terrain_height = 0.10;

enum class WheelType { SIMPLE, MESH, BCE_MARKERS };
std::string wheel_BCE_csvfile = "vehicle/artcar/wheel_straight.csv";
WheelType wheel_type = WheelType::BCE_MARKERS;

void CreateFSIWheels(std::shared_ptr<ARTcar> vehicle, CRMTerrain& terrain, WheelType wheel_type);
// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ----------------
    // Problem settings
    // ----------------

    double tend = 30;
    bool verbose = true;

    // Visualization settings
    bool render = true;                    // use run-time visualization
    double render_fps = 200;               // rendering FPS
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers

    // CRM material properties
    double density = 1700;
    double cohesion = 1e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // CRM (moving) active box dimension
    double active_box_dim = 0.5;
    double settling_time = 0;

    // Set SPH spacing
    double spacing = 0.01;

    // SPH integration sacheme
    IntegrationScheme integration_scheme = IntegrationScheme::RK2;

    // --------------
    // Create vehicle
    // --------------

    cout << "Create vehicle..." << endl;
    double vehicle_init_height = 0.5;
    bool fea_tires;
    auto artCar = chrono_types::make_shared<ARTcar>();
    artCar->SetContactMethod(ChContactMethod::SMC);
    artCar->SetChassisFixed(false);
    artCar->SetInitPosition(ChCoordsys<>(ChVector3d(0.5, 0, vehicle_init_height), QUNIT));
    artCar->SetTireType(TireModelType::RIGID);
    artCar->SetMaxMotorVoltageRatio(0.16);
    artCar->SetStallTorque(3);
    artCar->SetTireRollingResistance(0.06);
    artCar->Initialize();
    auto sysMBS = artCar->GetSystem();
    sysMBS->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    artCar->SetChassisVisualizationType(VisualizationType::MESH);
    artCar->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    artCar->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    artCar->SetWheelVisualizationType(VisualizationType::MESH);
    artCar->SetTireVisualizationType(VisualizationType::MESH);

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 5e-4;
    ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

    int num_threads_chrono = 8;
    int num_threads_collision = 1;
    int num_threads_eigen = 7;
    int num_threads_pardiso = 0;

    SetChronoSolver(*sysMBS, solver_type, integrator_type, num_threads_pardiso);
    sysMBS->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    // ----------------------
    // Create the CRM terrain
    // ----------------------

    CRMTerrain terrain(*sysMBS, spacing);
    auto sysFSI = terrain.GetFsiSystemSPH();
    terrain.SetVerbose(verbose);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    terrain.SetStepSizeCFD(step_size);

    // Register the vehicle with the CRM terrain
    terrain.RegisterVehicle(&artCar->GetVehicle());

    // Set SPH parameters and soil material properties
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    // Set SPH solver parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = integration_scheme;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.free_surface_threshold = 0.8;
    sph_params.artificial_viscosity = 0.2;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.kernel_type = KernelType::WENDLAND;
    terrain.SetSPHParameters(sph_params);

    // Set output level from SPH simulation
    terrain.SetOutputLevel(OutputLevel::STATE);

    // Add vehicle wheels as FSI solids
    CreateFSIWheels(artCar, terrain, wheel_type);
    terrain.SetActiveDomain(ChVector3d(active_box_dim));
    terrain.SetActiveDomainDelay(settling_time);

    // Construct the terrain
    cout << "Create terrain..." << endl;
    // Create a rectangular terrain patch
    terrain.Construct({terrain_length, terrain_width, terrain_height},  // length X width X height
                      ChVector3d(terrain_length / 2, 0, 0),             // patch center
                      BoxSide::ALL & ~BoxSide::Z_POS                    // all boundaries, except top
    );

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // Set maximum vehicle X location (based on CRM patch size with 0.5m tolerance)
    double x_max = aabb.max.x() - 0.5;

    // -----------------------------
    // Set up output
    // -----------------------------
    std::string out_dir = GetChronoOutputPath() + "ARTcar_PullTest/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    std::string out_file = out_dir + "/results.txt";
    utils::ChWriterCSV csv(" ");

    // -----------------------------
    // Create run-time visualization
    // -----------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(aabb.min.z(), aabb.max.z());
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);

        // Wheeled vehicle VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        visVSG->AttachVehicle(&artCar->GetVehicle());
        visVSG->AttachPlugin(visFSI);
        visVSG->SetWindowTitle("Wheeled vehicle on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->EnableSkyBox();
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        visVSG->SetCameraAngleDeg(40);
        visVSG->SetChaseCamera(VNULL, 3.0, 2.0);
        visVSG->SetChaseCameraPosition(ChVector3d(0, 4, 0.5));

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // ---------------
    // Simulation loop
    // ---------------

    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    bool braking = false;

    cout << "Start simulation..." << endl;

    while (time < tend) {
        const auto& veh_loc = artCar->GetVehicle().GetPos();

        // Create driver inputs
        DriverInputs driver_inputs;

        // Ramp up throttle from 0 to 100% over 1 second
        if (time < 1.0) {
            driver_inputs.m_throttle = time;  // Linear ramp from 0 to 1 over 1 second
            driver_inputs.m_braking = 0;
        } else {
            driver_inputs.m_throttle = 1.0;  // Full throttle after 1 second
            driver_inputs.m_braking = 0;
        }

        // Stop vehicle before reaching end of terrain patch (with 0.5m tolerance)
        if (veh_loc.x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
            if (!braking) {
                cout << "Start braking..." << endl;
                tend = time + 2;
                braking = true;
            }
        }

        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Synchronize systems
        vis->Synchronize(time, driver_inputs);
        terrain.Synchronize(time);
        artCar->GetVehicle().Synchronize(time, driver_inputs, terrain);

        // Advance system state
        vis->Advance(step_size);
        terrain.Advance(step_size);

        csv << time << artCar->GetVehicle().GetPos() << artCar->GetVehicle().GetSpeed() << endl;

        time += step_size;
        sim_frame++;
    }

    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string title = "Vehicle ref frame height";
    gplot.SetTitle(title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(out_file, 1, 4, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}

void CreateFSIWheels(std::shared_ptr<ARTcar> artCar, CRMTerrain& terrain, WheelType wheel_type) {
    for (auto& axle : artCar->GetVehicle().GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto sysFSI = terrain.GetFsiSystemSPH();
            if (wheel_type == WheelType::SIMPLE) {
                // Create just a cylindrical wheel of set radius
                auto wheel_radius = wheel->GetTire()->GetRadius();
                auto wheel_width = wheel->GetTire()->GetWidth();

                std::cout << "Wheel radius: " << wheel_radius << " Wheel width: " << wheel_width << std::endl;

                auto wheel_mass = wheel->GetTire()->GetTireMass();
                ChCylinder cylinder(wheel_radius, wheel_width);
                auto mass = wheel_mass * cylinder.GetVolume();
                auto inertia = wheel_mass * cylinder.GetGyration();
                auto geometry_simple = chrono_types::make_shared<utils::ChBodyGeometry>();
                geometry_simple->materials.push_back(ChContactMaterialData());
                geometry_simple->coll_cylinders.push_back(
                    utils::ChBodyGeometry::CylinderShape(VNULL, Q_ROTATE_Z_TO_Y, cylinder, 0));
                terrain.AddRigidBody(wheel->GetSpindle(), geometry_simple, false, false);
            } else if (wheel_type == WheelType::MESH) {
                std::string mesh_filename = vehicle::GetDataFile("artcar/tire.obj");
                auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
                geometry->materials.push_back(ChContactMaterialData());
                geometry->coll_meshes.push_back(
                    utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, mesh_filename, VNULL));
                terrain.AddRigidBody(wheel->GetSpindle(), geometry, false, false);
            } else {
                std::cout << "Adding wheel BCE markers from csv file: " << wheel_BCE_csvfile << std::endl;
                std::vector<ChVector3d> BCE_wheel;
                std::ifstream file(GetChronoDataFile(wheel_BCE_csvfile));
                std::string line;
                std::getline(file, line);  // skip the first line
                while (std::getline(file, line)) {
                    std::stringstream iss(line);
                    std::string val;
                    std::vector<double> values;
                    while (std::getline(iss, val, ',')) {
                        values.push_back(std::stod(val));
                    }
                    BCE_wheel.push_back(ChVector3d(values[0], values[1], values[2]));
                }
                auto sysFSI = terrain.GetFsiSystemSPH();
                sysFSI->AddFsiBody(wheel->GetSpindle(), BCE_wheel, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y), false);
                std::cout << "Added wheel BCE markers to FSI system" << std::endl;
            }
        }
    }
}
