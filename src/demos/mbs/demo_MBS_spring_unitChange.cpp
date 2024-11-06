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
// Authors: Radu Serban
// =============================================================================
//
// Simple example demonstrating the use of ChLinkTSDA.
//
// Two bodies, connected with identical (but modeled differently) spring-dampers
// are created side by side.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// Unit multipler - 0.004 mm now corresponds to 1 unit of length
// Thus 1 m corresponds to 250 units of length
// so all m values should be multiplied by 250
double unit_multiplier = 250;

// =============================================================================

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

double rest_length = 1.5 * unit_multiplier;
double spring_coef = 50;  // no change
double damping_coef = 1;  // no change

// =============================================================================

// Functor class implementing the force for a ChLinkTSDA link.
// In this simple demonstration, we just reimplement the default linear spring-damper.
class MySpringForce : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double restlength,      // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        double force = -spring_coef * (length - restlength) - damping_coef * vel;
        return force;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create the ground body with two visualization spheres
    // -----------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    {
        auto sph_1 = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        ground->AddVisualShape(sph_1, ChFrame<>(ChVector3d(-1 * unit_multiplier, 0, 0), QUNIT));

        auto sph_2 = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        ground->AddVisualShape(sph_2, ChFrame<>(ChVector3d(+1 * unit_multiplier, 0, 0), QUNIT));
    }

    // Create a body suspended through a ChLinkTSDA (default linear)
    // -------------------------------------------------------------

    auto body_1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_1);
    body_1->SetPos(ChVector3d(-1 * unit_multiplier, -3 * unit_multiplier, 0));
    body_1->SetFixed(false);
    body_1->EnableCollision(false);
    body_1->SetMass(1);
    body_1->SetInertiaXX(ChVector3d(1 * unit_multiplier * unit_multiplier, 1 * unit_multiplier * unit_multiplier,
                                    1 * unit_multiplier * unit_multiplier));

    // Attach a visualization asset.
    auto box_1 =
        chrono_types::make_shared<ChVisualShapeBox>(1 * unit_multiplier, 1 * unit_multiplier, 1 * unit_multiplier);
    box_1->SetColor(ChColor(0.6f, 0, 0));
    body_1->AddVisualShape(box_1);

    // Create the spring between body_1 and ground. The spring end points are
    // specified in the body relative frames.
    auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
    spring_1->Initialize(body_1, ground, true, ChVector3d(0, 0, 0), ChVector3d(-1 * unit_multiplier, 0, 0));
    spring_1->SetRestLength(rest_length);
    spring_1->SetSpringCoefficient(spring_coef);
    spring_1->SetDampingCoefficient(damping_coef);
    sys.AddLink(spring_1);

    // Attach a visualization asset.
    spring_1->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.1, 80, 15));

    // Create a body suspended through a ChLinkTSDA (custom force functor)
    // -------------------------------------------------------------------

    auto body_2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_2);
    body_2->SetPos(ChVector3d(1 * unit_multiplier, -3 * unit_multiplier, 0));
    body_2->SetFixed(false);
    body_2->EnableCollision(false);
    body_2->SetMass(1);
    body_2->SetInertiaXX(ChVector3d(1 * unit_multiplier * unit_multiplier, 1 * unit_multiplier * unit_multiplier,
                                    1 * unit_multiplier * unit_multiplier));

    // Attach a visualization asset.
    auto box_2 =
        chrono_types::make_shared<ChVisualShapeBox>(1 * unit_multiplier, 1 * unit_multiplier, 1 * unit_multiplier);
    box_2->SetColor(ChColor(0, 0, 0.6f));
    body_2->AddVisualShape(box_2);

    // Create the spring between body_2 and ground. The spring end points are
    // specified in the body relative frames.
    auto force = chrono_types::make_shared<MySpringForce>();

    auto spring_2 = chrono_types::make_shared<ChLinkTSDA>();
    spring_2->Initialize(body_2, ground, true, ChVector3d(0, 0, 0), ChVector3d(1 * unit_multiplier, 0, 0));
    spring_2->SetRestLength(rest_length);
    spring_2->RegisterForceFunctor(force);
    sys.AddLink(spring_2);

    // Attach a visualization asset.
    spring_2->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.1 * unit_multiplier, 80, 15));

    // Create the run-time visualization system
    // ----------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("ChLinkTSDA demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 0, 6 * unit_multiplier));
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowSize(ChVector2i(800, 600));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetWindowTitle("Chrono VSG Springs");
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->AddCamera(ChVector3d(-200, 200, 12));
            vis_vsg->SetCameraAngleDeg(90);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    int frame = 0;

    double timestep = 0.001;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (frame % 100 == 0) {
            // std::cout << "Time: " << sys.GetChTime() << " s" << std::endl;
            // std::cout << "Spring 1 length: " << spring_1->GetLength() << " units" << std::endl;
            // std::cout << "Spring 1 velocity: " << spring_1->GetVelocity() << " units/s" << std::endl;
            // std::cout << "Spring 1 force: " << spring_1->GetForce() << " kg units/s^2" << std::endl;
            // std::cout << "Spring 2 length: " << spring_2->GetLength() << " units" << std::endl;
            // std::cout << "Spring 2 velocity: " << spring_2->GetVelocity() << " units/s" << std::endl;
            // std::cout << "Spring 2 force: " << spring_2->GetForce() << " kg units/s^2" << std::endl;

            // std::cout << std::endl;

            std::cout << "Time: " << sys.GetChTime() << " s" << std::endl;
            std::cout << "Spring 1 length: " << spring_1->GetLength() / unit_multiplier << " m" << std::endl;
            std::cout << "Spring 1 velocity: " << spring_1->GetVelocity() / unit_multiplier << " m/s" << std::endl;
            std::cout << "Spring 1 force: " << spring_1->GetForce() / unit_multiplier << " kg m/s^2" << std::endl;
            std::cout << "Spring 2 length: " << spring_2->GetLength() / unit_multiplier << " m" << std::endl;
            std::cout << "Spring 2 velocity: " << spring_2->GetVelocity() / unit_multiplier << " m/s" << std::endl;
            std::cout << "Spring 2 force: " << spring_2->GetForce() / unit_multiplier << " kg m/s^2" << std::endl;
        }

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);

        frame++;
    }

    return 0;
}
