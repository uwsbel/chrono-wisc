// =============================================================================
// PROJECT CHRONO - Gravity Tilt Extension
// =============================================================================
// Description:
//  Modified version of the Polaris CRM terrain demo. The vehicle first settles
//  under normal gravity, then the gravity vector tilts gradually along the
//  cross-slope (Y) axis to simulate a lateral slope.
// =============================================================================
#include <iostream>
#include <iomanip>
#include <fstream>
#include <memory>
#include <cmath>


#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "demos/vehicle/WheeledVehicleJSON.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"
#include "demos/SetChronoSolver.h"
#include "chrono/assets/ChVisualShapeBox.h"


using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace std;

struct SimulationConfig {
    bool render = true;
    double render_fps = 200;
    bool visualization_sph = true;
    bool visualization_boundary = false;
    bool visualization_rigid = true;
    bool chase_cam = true;

    double terrain_length = 5.0;
    double terrain_width = 4.0;
    double vehicle_init_height = 0.3;
    double tend = 30.0;
    double tilt_start_time = 1.0;
    double tilt_duration = 5.0;
    double tilt_angle_deg = 15.0;
    double step_size = 1e-3;
    bool verbose = true;
    double density = 2100;
    double cohesion = 1e3;
    double friction = 1.0;
    double youngs_modulus = 3e6;
    double poisson_ratio = 0.3;

    bool use_airless = false;
};

void CreateFSIWheels(WheeledVehicle* vehicle,
                      CRMTerrain& terrain) {
    std::string mesh_filename = vehicle::GetDataFile("Polaris/meshes/Polaris_tire_collision_scaled.obj");
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename, VNULL));

    // Vehicle wheels
    std::cout << "!!!!!!!!!!" << std::endl;
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire_fea = std::dynamic_pointer_cast<ChDeformableTire>(wheel->GetTire());
            std::cout << "TIRE FEA:" << tire_fea << std::endl;
            if (tire_fea) {
                auto mesh = tire_fea->GetMesh();
                terrain.AddFeaMesh(mesh, false);
            } else {
                terrain.AddRigidBody(wheel->GetSpindle(), geometry, false);
            }
        }
    }


}

int main() {
    SimulationConfig config;
    // === CSV Logging ===
    std::ofstream csv_file("chassis_data_" + std::to_string(config.tilt_angle_deg) +".csv");

    csv_file << "time,lin_vel_x,lin_vel_y,lin_vel_z,ang_vel_x,ang_vel_y,ang_vel_z,"
                "lin_acc_x,lin_acc_y,lin_acc_z,ang_acc_x,ang_acc_y,ang_acc_z\n";

    // Create physical system
    ChSystem* sys = new ChSystemSMC();
    SetChronoSolver(*sys, ChSolver::Type::PARDISO_MKL, ChTimestepper::Type::HHT, std::min(8, ChOMP::GetNumProcs()));
    sys->SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 1);
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create terrain
    CRMTerrain terrain(*sys, 0.04);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -1.69));
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -1.69));

    terrain.SetStepSizeCFD(config.step_size);
 
    // Configure terrain soil and SPH parameters
    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = config.density;
    mat_props.Young_modulus = config.youngs_modulus;
    mat_props.Poisson_ratio = config.poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = config.friction;
    mat_props.mu_fric_2 = config.friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = config.cohesion;
    terrain.SetElasticSPH(mat_props);

    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = 0.04;
    sph_params.d0_multiplier = 1.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::ADAMI;
    terrain.SetSPHParameters(sph_params);

    //terrain.DisableMBD();

   
    terrain.SetOutputLevel(OutputLevel::CRM_FULL);

    // Create vehicle
    WheeledVehicle* vehicle = new WheeledVehicle(sys, vehicle::GetDataFile("Polaris/Polaris_LTV.json"));
    vehicle->Initialize(ChCoordsys<>(ChVector3d(2.5, 0.0, config.vehicle_init_height), QUNIT));
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    auto engine = ReadEngineJSON(vehicle::GetDataFile("Polaris/Polaris_EngineSimpleMap.json"));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile("Polaris/Polaris_AutomaticTransmissionSimpleMap.json"));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Initialize deformable tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON("data/vehicle/Polaris/Polaris_RigidMeshTire.json");
            if(!config.use_airless) {
                vehicle->InitializeTire(tire,wheel, VisualizationType::MESH);
            } else {
            auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
                tire->SetRimRadius(0.19);
                tire->SetHeight(0.19);
                tire->SetWidth(0.25);
                tire->SetAlpha(0.05);
                tire->SetOuterRingThickness(0.015);
                tire->SetYoungsModulusSpokes(8e5);
                tire->SetYoungsModulusOuterRing(1e9);
                tire->SetPoissonsRatio(0.3);
                tire->SetDivWidth(3);
                tire->SetDivSpokeLength(3);
                tire->SetDivOuterRingPerSpoke(3);
                tire->SetNumberSpokes(16);
                tire->SetHubRelativeRotation(0);
                tire->SetSpokeCurvatureXPoint(0);
                tire->SetSpokeCurvatureZPoint(0);
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }
    }
    auto chassis = vehicle->GetChassisBody();
    
    // Create the cube body
    // auto cube = chrono_types::make_shared<ChBody>();
    // cube->SetMass(50.0);  // Approximate human weight in kg
    // cube->SetInertiaXX(ChVector3d(15.3, 16.95, 2.25)); // Adjust for realism
    // cube->SetPos(chassis->TransformPointLocalToParent(ChVector3d(0.05, 0.48, 0.2)));  // Adjust local position as needed
    // cube->SetRot(chassis->GetRot()); // Align orientation with chassis
    // cube->SetFixed(false);
    // sys->AddBody(cube);
    // auto cube2 = chrono_types::make_shared<ChBody>();

    //cube2->SetMass(50.0);  // Approximate human weight in kg
    //cube2->SetInertiaXX(ChVector3d(15.3, 16.95, 2.25)); // Adjust for realism
    //cube2->SetPos(chassis->TransformPointLocalToParent(ChVector3d(0.05, -0.48, 0.2)));  // Adjust local position as needed
    //cube2->SetRot(chassis->GetRot()); // Align orientation with chassis
    //cube2->SetFixed(false);
    //// sys->AddBody(cube2);


    //// Create fixed joint to attach cube to chassis
    //auto constraint = chrono_types::make_shared<ChLinkLockLock>();
    //constraint->Initialize(cube, chassis, ChFrame<> (cube->GetPos(), cube->GetRot()));
    //sys->AddLink(constraint);

    //auto constraint2 = chrono_types::make_shared<ChLinkLockLock>();
    //constraint2->Initialize(cube2, chassis, ChFrame<> (cube2->GetPos(), cube2->GetRot()));
    //sys->AddLink(constraint2);

    
    // Optional: visualization
    //auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(0.3,0.3,0.3));
    //cube->AddVisualShape(box_shape);
    //cube2->AddVisualShape(box_shape);


    terrain.Construct({config.terrain_length, config.terrain_width, 0.25},
                      ChVector3d(config.terrain_length / 2, 0, 0),
                      BoxSide::ALL & ~BoxSide::Z_POS);


    // Register tires with terrain
    CreateFSIWheels(vehicle, terrain);
    terrain.Initialize();
    std::shared_ptr<ChFsiVisualization> visFSI;
    #if defined(CHRONO_OPENGL) || defined(CHRONO_VSG)
    if (config.render) {
        #ifdef CHRONO_OPENGL
            visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&terrain.GetSystemFSI());
        #elif defined(CHRONO_VSG)
            visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&terrain.GetSystemFSI());
        #endif
        visFSI->SetTitle("Wheeled vehicle on CRM deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, 3, 1.5), ChVector3d(0, -1, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(config.visualization_sph);
        visFSI->EnableBoundaryMarkers(config.visualization_boundary);
        visFSI->EnableRigidBodyMarkers(config.visualization_rigid);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->AttachSystem(vehicle->GetSystem());
        visFSI->SetLightIntensity(0.7);
        visFSI->Initialize();
    }
    #endif

    // Time loop
    double time = 0;
    int sim_frame = 0, render_frame = 0;
    ChTimer timer;
    while (time < config.tend) {
         // Get current velocities
         ChVector3d lin_vel = vehicle->GetChassisBody()->GetPosDt();
         ChVector3d ang_vel = vehicle->GetChassisBody()->GetAngVelParent();

         // Compute accelerations (finite difference)
         ChVector3d lin_acc = vehicle->GetChassisBody()->GetPosDt2();
         ChVector3d ang_acc = vehicle->GetChassisBody()->GetAngAccParent();


        if (config.render && time >= render_frame / config.render_fps) {
            std::cout << "start " << render_frame << std::endl;
            if (config.chase_cam) {
                ChVector3d cam_loc = vehicle->GetPos() + ChVector3d(-4, 4, 1.5);
                visFSI->UpdateCamera(cam_loc, vehicle->GetPos());
            }
            if (!visFSI->Render())
                break;
            std::ostringstream filename;
            filename << "./snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << ".png";
            visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            render_frame++;
            std::cout << "..fin" << std::endl;
        }
        // Apply gravity tilt after settle phase
        if (time >= config.tilt_start_time) {
            double tilt_elapsed = time - config.tilt_start_time;
            double alpha = std::min(tilt_elapsed / config.tilt_duration, 1.0);
            double tilt_radians = (config.tilt_angle_deg) * CH_DEG_TO_RAD;
            double current_angle = (tilt_radians) * alpha;

            ChVector3d new_gravity = ChVector3d(0, -std::sin(current_angle), -std::cos(current_angle)) * 1.69;
            //ChVector3d new_gravity = ChVector3d(-std::sin(current_angle), 0*-std::sin(current_angle), -std::cos(current_angle)) * 1.69;
            
            sys->SetGravitationalAcceleration(new_gravity);
            terrain.SetGravitationalAcceleration(new_gravity);

            if (config.verbose && sim_frame % 10 == 0) {
                cout << "[Tilt] Time: " << time << "s, Tilt: " << current_angle * 180 / CH_PI << " deg, gy: " << new_gravity.y() << " gz: " << new_gravity.z() << "\n";
            }

            if (config.verbose && sim_frame % 10 == 0) {
                std::cout << std::fixed << std::setprecision(4);
                std::cout << "[Chassis] Time: " << time << " s\n";
                std::cout << "  Linear Velocity     : (" << lin_vel.x() << ", " << lin_vel.y() << ", " << lin_vel.z() << ")\n";
                std::cout << "  Angular Velocity    : (" << ang_vel.x() << ", " << ang_vel.y() << ", " << ang_vel.z() << ")\n";
                std::cout << "  Linear Acceleration : (" << lin_acc.x() << ", " << lin_acc.y() << ", " << lin_acc.z() << ")\n";
                std::cout << "  Angular Acceleration: (" << ang_acc.x() << ", " << ang_acc.y() << ", " << ang_acc.z() << ")\n";
            }

        }
        std::cout << " Time " << time << std::endl;
        csv_file << time << ","
                 << lin_vel.x() << "," << lin_vel.y() << "," << lin_vel.z() << ","
                 << ang_vel.x() << "," << ang_vel.y() << "," << ang_vel.z() << ","
                 << lin_acc.x() << "," << lin_acc.y() << "," << lin_acc.z() << ","
                 << ang_acc.x() << "," << ang_acc.y() << "," << ang_acc.z() << "\n";

        vehicle->Synchronize(time, {}, terrain);
        terrain.Synchronize(time);
        
        vehicle->Advance(config.step_size);
        terrain.Advance(config.step_size);
        sys->DoStepDynamics(config.step_size);

        // Set correct real-time factor for FSI
        timer.reset();
        timer.start();
        timer.stop();
        double rtf = timer() / config.step_size;
        terrain.GetSystemFSI().SetRtf(rtf);


        time += config.step_size;
        sim_frame++;


    }

    delete sys;
    csv_file.close();
    return 0;
}
