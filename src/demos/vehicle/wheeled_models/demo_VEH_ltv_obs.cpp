#include "chrono/utils/ChUtils.h"
#include <iomanip>
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChContactMaterial.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/vehicle/WheeledVehicleJSON.h"
#include "demos/SetChronoSolver.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        // if (eff_time > 0.0 && eff_time <= 6.0)
        //     m_throttle = 0.95;
        // else if (eff_time > 6.0)
        //     m_throttle = 0.6;

        if (eff_time > 0.0) {
            m_throttle = (4.5 - m_vehicle.GetSpeed()) * 0.3;
            m_throttle = std::min(m_throttle, 1.0);
            m_throttle = std::max(m_throttle, 0.0);
        }

        if (eff_time < 0.0)
            m_steering = 0;
        else
            m_steering = 0.1 * std::sin(CH_2PI * (eff_time - 0) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Trailer model selection (use only with HMMWV, Sedan, or UAZ)
bool add_trailer = true;
auto trailer_model = UT_Model();

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle position and orientation (adjust for selected terrain)
double init_height = 0.02;
ChVector3d initLoc(0, 0, init_height);
double initYaw = -2 * CH_DEG_TO_RAD;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Render frequency
double render_fps = 100;

// End time (used only if no run-time visualization)
double t_end = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select vehicle model (see WheeledVehicleJSON.h)
    auto models = WheeledVehicleJSON::List();

    const auto& vehicle_model = models[2].first;

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_model->VehicleJSON()), contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, QuatFromAngleZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model->EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model->TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (unsigned int i = 0; i < vehicle.GetNumberAxles(); i++) {
        for (auto& wheel : vehicle.GetAxle(i)->GetWheels()) {
            auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.13);              // Default is 0.225
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.2);                  // Default is 0.225
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.24);                  // Default is 0.4
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);                  // Default is 0.05
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e9);     // Default is 76e9
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(1e9);  // Default is 76e9
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);           // Default is 0.2
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);                  // Default is 3
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);            // Default is 3
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);

            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->EnableContact(true);
            auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
            int collision_family = 7;
            double collision_dim = 0.06;
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(0.06);
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactSurfaceType(surface_type, collision_dim,
                                                                                         collision_family);
            auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
            mat->SetKn(1e10);
            mat->SetKt(1e9);
            mat->SetFriction(0.99);
            std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactMaterial(mat);
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto sys = vehicle.GetSystem();

    // Create the trailer system (build into same ChSystem)
    std::shared_ptr<WheeledTrailer> trailer;
    if (add_trailer) {
        trailer = chrono_types::make_shared<WheeledTrailer>(sys, vehicle::GetDataFile(trailer_model.TrailerJSON()));
        trailer->Initialize(vehicle.GetChassis());
        trailer->SetChassisVisualizationType(VisualizationType::MESH);
        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetWheelVisualizationType(VisualizationType::MESH);
        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = chrono_types::make_shared<ANCFAirlessTire3443B>("Airless3443B");
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetRimRadius(0.13);           // Default is 0.225
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetHeight(0.2);               // Default is 0.225
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetWidth(0.24);               // Default is 0.4
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetAlpha(0.05);               // Default is 0.05
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusSpokes(1e9);  // Default is 76e9
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetYoungsModulusOuterRing(
                    1e9);                                                                      // Default is 76e9
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetPoissonsRatio(0.3);  // Default is 0.2
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivWidth(3);         // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivSpokeLength(3);   // Default is 3
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetDivOuterRingPerSpoke(3);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->EnableContact(true);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetOuterRingThickness(0.05);
                auto surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
                int collision_family = 7;
                double collision_dim = 0.05;
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactSurfaceType(
                    surface_type, collision_dim, collision_family);

                auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
                mat->SetKn(1e10);
                // You might want to try below settings if the tire penetrates too much
                // mat->SetKt(1e9);
                // mat->SetFriction(0.99);
                std::dynamic_pointer_cast<ANCFAirlessTire3443B>(tire)->SetContactMaterial(mat);
                trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }
    }

    // Add obstacles
    // Create obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rock;
    std::vector<std::string> rock_meshfile = {
        "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock3.obj", "robot/curiosity/rocks/rock3.obj"   //
    };
    std::vector<ChVector3d> rock_pos = {
        ChVector3d(1.9, -1.1, 0.3), ChVector3d(1.9, +1.1, 0.3),  // Original: 0.5 + 1.5 = 2.0
        ChVector3d(4.8, -1.0, 0.3), ChVector3d(4.8, +1.0, 0.3),  // Original: 2.0 + 1.5 = 3.5
        ChVector3d(6.9, -1.0, 0.3), ChVector3d(6.9, +1.0, 0.3)   // Original: 3.5 + 1.5 = 5.0
    };
    std::vector<double> rock_scale = {
        0.5,  0.5,   //
        0.55, 0.55,  //
        0.55, 0.55   //
    };
    double rock_density = 300000;
    std::shared_ptr<ChContactMaterialSMC> rock_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    rock_mat->SetKn(5e20);
    rock_mat->SetKt(1e10);
    rock_mat->SetFriction(0.99);

    for (int i = 0; i < 6; i++) {
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(rock_meshfile[i]), true, true);
        mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(rock_scale[i]));

        double mass;
        ChVector3d cog;
        ChMatrix33<> inertia;
        mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        sys->Add(body);
        body->SetFixed(false);
        body->SetFrameRefToAbs(ChFrame<>(ChVector3d(rock_pos[i]), QuatFromAngleX(CH_PI_2)));
        body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        body->SetMass(mass * rock_density);
        body->SetInertiaXX(rock_density * principal_I);

        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_mat, mesh, false, false, 0.005);
        body->AddCollisionShape(ct_shape);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->AllowCollisionsWith(1);
        body->GetCollisionModel()->AllowCollisionsWith(7);
        body->GetCollisionModel()->AllowCollisionsWith(13);

        body->EnableCollision(true);

        auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vis_shape->SetMesh(mesh);
        vis_shape->SetBackfaceCull(true);
        body->AddVisualShape(vis_shape);

        rock.push_back(body);
    }

    // Associate a collision system
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys->SetNumThreads(12);
    // Enable collision between the two families

    // Create the terrain
    // RigidTerrain terrain(sys, vehicle::GetDataFile(rigidterrain_file));
    // terrain.Initialize();

    // ------------------
    SCMTerrain terrain(sys);
    terrain.SetSoilParameters(4e7,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              20,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Optionally, enable bulldozing effects.
    ////terrain.EnableBulldozing(true);      // inflate soil at the border of the rut
    ////terrain.SetBulldozingParameters(55,   // angle of friction for erosion of displaced material at rut border
    ////                                0.8,  // displaced material vs downward pressed material.
    ////                                5,    // number of erosion refinements per timestep
    ////                                10);  // number of concentric vertex selections subject to erosion

    // Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(vehicle.GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(8, 4, 1));
    // add moving patch for each obstacles
    for (int i = 0; i < 6; i++)
        terrain.AddMovingPatch(rock[i], VNULL, ChVector3d(2.0, 2.0, 2.0));

    ChVector3d init_loc;
    ChVector2d patch_size;
    init_loc = ChVector3d(-15.0, -6.0, 0.6);
    patch_size = ChVector2d(40.0, 16.0);
    // increase the resolution here!
    terrain.Initialize(patch_size.x(), patch_size.y(), 0.1);
    // Control visualization of SCM terrain
    terrain.GetMesh()->SetWireframe(true);

    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);

    // Set solver and integrator
    // With trailer run 1e-3, without trailer run 2e-3
    double step_size = 1e-3;
    auto solver_type = ChSolver::Type::PARDISO_MKL;
    auto integrator_type = ChTimestepper::Type::HHT;
    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::string title = "Vehicle demo - JSON specification - " + vehicle_model->ModelName();
    std::shared_ptr<ChVehicleVisualSystem> vis;

    auto driver = MyDriver(vehicle, 0.25);

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(), 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);

            // Create the interactive Irrlicht driver system
            auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
            driver_irr->SetSteeringDelta(0.02);
            driver_irr->SetThrottleDelta(0.02);
            driver_irr->SetBrakingDelta(0.06);
            driver_irr->Initialize();

            vis = vis_irr;
            // driver = driver_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachTerrain(&terrain);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(), 3.0);
            vis_vsg->SetChaseCameraPosition(ChVector3d(4.0, 7.0, 2.0));
            vis_vsg->SetWindowSize(ChVector2i(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            // Create the interactive VSG driver system

            vis = vis_vsg;
            // driver = driver_vsg;
#endif
            break;
        }
    }

    // Initialize output directories
    const std::string out_dir = GetChronoOutputPath() + "WHEELED_JSON";
    const std::string veh_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(veh_dir))) {
        std::cout << "Error creating directory " << veh_dir << std::endl;
        return 1;
    }

    const std::string img_dir = veh_dir + "/IMG";
    if (!filesystem::create_directory(filesystem::path(img_dir))) {
        std::cout << "Error creating directory " << img_dir << std::endl;
        return 1;
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(veh_dir + "/component_list.json");

    vehicle.LogSubsystemTypes();

    // Optionally, enable output from selected vehicle subsystems
    ////vehicle.SetSuspensionOutput(0, true);
    ////vehicle.SetSuspensionOutput(1, true);
    ////vehicle.SetOutput(ChVehicleOutput::ASCII, veh_dir, "output", 0.1);

    // Simulation loop
    vehicle.EnableRealtime(false);

    int sim_frame = 0;
    int render_frame = 0;
    while (true) {
        double time = sys->GetChTime();

        if (vis) {
            if (!vis->Run())
                break;

            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".jpg";
                vis->WriteImageToFile(filename.str());
                render_frame++;
            }
        } else if (time > t_end) {
            break;
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();
        // Update modules (process inputs from other modules)
        // driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        driver.Synchronize(time);
        if (add_trailer)
            trailer->Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        // driver->Advance(step_size);
        vehicle.Advance(step_size);
        driver.Advance(step_size);
        if (add_trailer)
            trailer->Advance(step_size);
        terrain.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        sim_frame++;
    }

    return 0;
}
