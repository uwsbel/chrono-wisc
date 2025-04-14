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
// Chrono benchmark of VIPER rover models on SCM terrain
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::viper;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "BENCHMARK3_RTF/SCM_Viper/";

// SCM grid spacing - Matches the CRM spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// Enable visualization
bool render = false;
double render_fps = 100;

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

// Return customized wheel material parameters
std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create output directories
    std::string benchmark_dir = GetChronoOutputPath() + "BENCHMARK3_RTF/";
    if (!filesystem::create_directory(filesystem::path(benchmark_dir))) {
        std::cerr << "Error creating directory " << benchmark_dir << std::endl;
        return 1;
    }

    std::string scm_dir = benchmark_dir + "SCM_Viper/";
    if (!filesystem::create_directory(filesystem::path(scm_dir))) {
        std::cerr << "Error creating directory " << scm_dir << std::endl;
        return 1;
    }

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create a Chrono physical system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    if (use_custom_mat)
        rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(ChVector3d(-5, 0, -0.2), QUNIT));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = rover->GetChassis()->GetBody();

    // Create the SCM terrain
    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&sys);

    // Displace/rotate the terrain reference plane
    terrain->SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5)));

    // Use a regular gridx
    double length = 4;
    double width = 2;
    terrain->Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain->RegisterSoilParametersCallback(my_params);
    } else {
        terrain->SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain->EnableBulldozing(true);
        terrain->SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Add moving patches for each wheel
    if (enable_moving_patch) {
        terrain->AddMovingPatch(Wheel_1, ChVector3d(0, 0, 0), ChVector3d(0.4,0.15, 0.4));
        terrain->AddMovingPatch(Wheel_2, ChVector3d(0, 0, 0), ChVector3d(0.4,0.15, 0.4));
        terrain->AddMovingPatch(Wheel_3, ChVector3d(0, 0, 0), ChVector3d(0.4,0.15, 0.4));
        terrain->AddMovingPatch(Wheel_4, ChVector3d(0, 0, 0), ChVector3d(0.4,0.15, 0.4));
    }

    // Set visualization parameters
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);
    terrain->SetMeshWireframe(true);

    // Create the run-time visualization interface
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    if (render) {
        auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
        vis_vsg->AttachSystem(&sys);
        vis_vsg->SetWindowTitle("Viper Rover on SCM Terrain");
        vis_vsg->SetWindowSize(1280, 800);
        vis_vsg->SetWindowPosition(100, 100);
        vis_vsg->EnableSkyBox();
        vis_vsg->SetCameraAngleDeg(40);
        vis_vsg->SetLightIntensity(1.0f);
        vis_vsg->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
        vis_vsg->AddGuiColorbar("Pressure yield [Pa]", 0.0, 20000.0);
        vis_vsg->Initialize();

        vis = vis_vsg;
    }
#else
    render = false;
#endif

    // Simulation parametersx
    double step_size = 2.5e-4;
    double end_time = 2.0;
    double current_time = 0.0;
    int current_step = 0;

    // Timing variables
    double total_sim_time = 0.0;
    double total_real_time = 0.0;

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    // Simulation loop
    while (current_time < end_time) {
        rover->Update();
        sys.DoStepDynamics(step_size);
        current_time += step_size;
        current_step++;

        // Update timing
        auto current_real_time = std::chrono::high_resolution_clock::now();
        total_real_time = std::chrono::duration<double>(current_real_time - start_time).count();
        total_sim_time = current_time;

        // Run-time visualization
        if (render) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->SetCameraTarget(Body_1->GetPos());
            vis->Render();
            vis->EndScene();
        }
    }

    // Print results
    std::cout << "\nBenchmark Results:" << std::endl;
    std::cout << "-----------------" << std::endl;
    std::cout << "Terrain size: " << length << " x " << width << " m" << std::endl;
    std::cout << "Active box size: " << 0.5 << " x " << 2 * wheel_range << " x " << 2 * wheel_range << " m" << std::endl;
    std::cout << "Total simulated time: " << total_sim_time << " s" << std::endl;
    std::cout << "Total real time: " << total_real_time << " s" << std::endl;
    std::cout << "Real-time factor (RTF): " <<  total_real_time / total_sim_time << std::endl;

    // Write results to file
    std::ofstream outfile(scm_dir + "benchmark_results.txt");
    if (outfile.is_open()) {
        outfile << "Benchmark Results" << std::endl;
        outfile << "----------------" << std::endl;
        outfile << "Terrain type: SCM" << std::endl;
        outfile << "Terrain size: " << length << " x " << width << " m" << std::endl;
        outfile << "Active box size: " << 0.5 << " x " << 2 * wheel_range << " x " << 2 * wheel_range << " m" << std::endl;
        outfile << "Mesh resolution: " << mesh_resolution << " m" << std::endl;
        outfile << "Bulldozing enabled: " << (enable_bulldozing ? "yes" : "no") << std::endl;
        outfile << "Moving patch enabled: " << (enable_moving_patch ? "yes" : "no") << std::endl;
        outfile << "Variable parameters: " << (var_params ? "yes" : "no") << std::endl;
        outfile << "Wheel type: " << (wheel_type == ViperWheelType::RealWheel ? "RealWheel" : "SimpleWheel") << std::endl;
        outfile << "Custom material: " << (use_custom_mat ? "yes" : "no") << std::endl;
        outfile << "Total simulated time: " << total_sim_time << " s" << std::endl;
        outfile << "Total real time: " << total_real_time << " s" << std::endl;
        outfile << "Real-time factor (RTF): " <<  total_real_time / total_sim_time << std::endl;
        outfile.close();
        std::cout << "Results written to: " << scm_dir << "benchmark_results.txt" << std::endl;
    } else {
        std::cerr << "Error opening output file" << std::endl;
        return 1;
    }

    return 0;
} 