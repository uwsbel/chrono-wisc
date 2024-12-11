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
// Authors: Milad Rakhsha, Radu Serban, Michael Taylor
// =============================================================================
//
// Demo on using ANCF shell 3423T elements
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3423T.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/physics/ChLoadContainer.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << " ANCF Shell 3423 T Element demo with implicit integration  \n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";

    //Create a demo of an "I" shape using 4 elements.  2 regular Shell 3423 elements on the top and bottom and 2 Shell 3423 T elements in the middle.

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plates
    double horiz_plate_len = 0.2;
    double vert_plate_len = 0.2;
    double plate_width = 0.1;
    double plate_thickness = 0.01;

    double TOffset = 0; // 0 = "I" in the center of the horizontal plates

    // Create the nodes for the top plate
    auto node_1Top = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0.0, -0.5 * plate_width, 0.5 * vert_plate_len),
                                                              ChVector3d(0.0, 0.0, 1.0));
    auto node_2Top = chrono_types::make_shared<ChNodeFEAxyzD>(
        ChVector3d(horiz_plate_len, -0.5 * plate_width, 0.5 * vert_plate_len), ChVector3d(0.0, 0.0, 1.0));
    auto node_3Top = chrono_types::make_shared<ChNodeFEAxyzD>(
        ChVector3d(horiz_plate_len, 0.5 * plate_width, 0.5 * vert_plate_len), ChVector3d(0.0, 0.0, 1.0));
    auto node_4Top = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0.0, 0.5 * plate_width, 0.5 * vert_plate_len),
                                                              ChVector3d(0.0, 0.0, 1.0));


    // Creat the nodes for the middle of the "I" that both "T"s will connect to
    auto node_B = chrono_types::make_shared<ChNodeFEAxyzD>(
        ChVector3d(0.5 * horiz_plate_len, -0.5 * plate_width, 0.0), ChVector3d(1.0, 0.0, 0.0));
    auto node_C = chrono_types::make_shared<ChNodeFEAxyzD>(
        ChVector3d(0.5 * horiz_plate_len, 0.5 * plate_width, 0.0),
                                                           ChVector3d(1.0, 0.0, 0.0));

    // Create the nodes for the bottom plate (Note that node 3 is under node 1, etc. so that the position vector gradient for the T-Up element points along the positive x axis
    auto node_3Bot = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0.0, -0.5 * plate_width, -0.5 * vert_plate_len),
                                                              ChVector3d(0.0, 0.0, 1.0));
    auto node_4Bot = chrono_types::make_shared<ChNodeFEAxyzD>(
        ChVector3d(horiz_plate_len, -0.5 * plate_width, -0.5 * vert_plate_len), ChVector3d(0.0, 0.0, 1.0));
    auto node_1Bot = chrono_types::make_shared<ChNodeFEAxyzD>(
        ChVector3d(horiz_plate_len, 0.5 * plate_width, -0.5 * vert_plate_len), ChVector3d(0.0, 0.0, 1.0));
    auto node_2Bot = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0.0, 0.5 * plate_width, -0.5 * vert_plate_len),
                                                              ChVector3d(0.0, 0.0, 1.0));

    // Fix one edge of the top plate
    node_1Top->SetFixed(true);
    node_4Top->SetFixed(true);

    // Add the nodes to mesh
    mesh->AddNode(node_1Top);
    mesh->AddNode(node_2Top);
    mesh->AddNode(node_3Top);
    mesh->AddNode(node_4Top);
    mesh->AddNode(node_B);
    mesh->AddNode(node_C);
    mesh->AddNode(node_1Bot);
    mesh->AddNode(node_2Bot);
    mesh->AddNode(node_3Bot);
    mesh->AddNode(node_4Bot);

    //Print the locations of the nodes and thier position vector gradients to help with debugging
    std::cout << "node_1Top " << node_1Top->GetPos() << ", " << node_1Top->GetSlope1() << std::endl; 
    std::cout << "node_2Top " << node_2Top->GetPos() << ", " << node_2Top->GetSlope1() << std::endl; 
    std::cout << "node_3Top " << node_3Top->GetPos() << ", " << node_3Top->GetSlope1() << std::endl; 
    std::cout << "node_4Top " << node_4Top->GetPos() << ", " << node_4Top->GetSlope1() << std::endl; 
    std::cout << "node_B " <<    node_B->GetPos()    << ", " << node_B->GetSlope1()    << std::endl;
    std::cout << "node_C " <<    node_C->GetPos()    << ", " << node_C->GetSlope1()    << std::endl;
    std::cout << "node_1Bot " << node_1Bot->GetPos() << ", " << node_1Bot->GetSlope1() << std::endl; 
    std::cout << "node_2Bot " << node_2Bot->GetPos() << ", " << node_2Bot->GetSlope1() << std::endl; 
    std::cout << "node_3Bot " << node_3Bot->GetPos() << ", " << node_3Bot->GetSlope1() << std::endl; 
    std::cout << "node_4Bot " << node_4Bot->GetPos() << ", " << node_4Bot->GetSlope1() << std::endl; 


    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho = 500;
    ChVector3d E(2.1e7, 2.1e7, 2.1e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    ChVector3d G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);


    //Create the top plate
    auto elementTop = chrono_types::make_shared<ChElementShellANCF_3423>();
    elementTop->SetNodes(node_1Top, node_2Top, node_3Top, node_4Top);
    // Set element dimensions
    elementTop->SetDimensions(horiz_plate_len, plate_width);
    // Add a single layers with a fiber angle of 0 degrees.
    elementTop->AddLayer(plate_thickness, 0 * CH_DEG_TO_RAD, mat);
    // Set other element properties
    elementTop->SetAlphaDamp(0.0);  // Structural damping for this element
    // Add element to mesh
    mesh->AddElement(elementTop);

    // Create the bottom plate
    auto elementBot = chrono_types::make_shared<ChElementShellANCF_3423>();
    elementBot->SetNodes(node_1Bot, node_2Bot, node_3Bot, node_4Bot);
    // Set element dimensions
    elementBot->SetDimensions(horiz_plate_len, plate_width);
    // Add a single layers with a fiber angle of 0 degrees.
    elementBot->AddLayer(plate_thickness, 0 * CH_DEG_TO_RAD, mat);
    // Set other element properties
    elementBot->SetAlphaDamp(0.0);  // Structural damping for this element
    // Add element to mesh
    mesh->AddElement(elementBot);

    // Create the T-Down element attached to the top plate
    auto elementTDown = chrono_types::make_shared<ChElementShellANCF_3423T>();
    elementTDown->SetNodes(node_1Top, node_2Top, node_B, node_C, node_3Top, node_4Top);
    // Set element dimensions SetDimensions(double lenX, double lenY, double lenXT, double Toffset, double TAng)
    elementTDown->SetDimensions(vert_plate_len, plate_width, horiz_plate_len, TOffset, CH_PI_4);
    // Add a single layers with a fiber angle of 0 degrees.
    elementTDown->AddLayer(plate_thickness, 0 * CH_DEG_TO_RAD, mat);
    // Set other element properties
    elementTDown->SetAlphaDamp(0.0);  // Structural damping for this element
    // Add element to mesh
    mesh->AddElement(elementTDown);

    // Create the T-Up element attached to the bottom plate
    auto elementTUp = chrono_types::make_shared<ChElementShellANCF_3423T>();
    elementTUp->SetNodes(node_1Bot, node_2Bot, node_C, node_B, node_3Bot, node_4Bot);
    // Set element dimensions SetDimensions(double lenX, double lenY, double lenXT, double Toffset, double TAng)
    elementTUp->SetDimensions(vert_plate_len, plate_width, horiz_plate_len, TOffset, -CH_PI_4);
    // Add a single layers with a fiber angle of 0 degrees.
    elementTUp->AddLayer(plate_thickness, 0 * CH_DEG_TO_RAD, mat);
    // Set other element properties
    elementTUp->SetAlphaDamp(0.0);  // Structural damping for this element
    // Add element to mesh
    mesh->AddElement(elementTUp);



    // Add the mesh to the system
    sys.Add(mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto visualizemeshA = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    visualizemeshA->SetColorscaleMinMax(0.0, 5.50);
    visualizemeshA->SetShrinkElements(true, 0.85);
    visualizemeshA->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(visualizemeshA);

    auto visualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    visualizemeshB->SetWireframe(true);
    visualizemeshB->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(visualizemeshB);

    auto visualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    visualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshC->SetSymbolsThickness(0.004);
    mesh->AddVisualShapeFEA(visualizemeshC);

    auto visualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    visualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshD->SetSymbolsScale(1);
    visualizemeshD->SetColorscaleMinMax(-0.5, 5);
    visualizemeshD->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizemeshD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Shells 3423T",
                                         ChVector3d(0.75 * horiz_plate_len, 3 * plate_width, 0.0),
                                         ChVector3d(0.5 * horiz_plate_len, 0.0, 0.0));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    //// Set up solver
    //auto solver = chrono_types::make_shared<ChSolverMINRES>();
    //sys.SetSolver(solver);
    //solver->SetMaxIterations(100);
    //solver->SetTolerance(1e-10);
    //solver->EnableDiagonalPreconditioner(true);

    //// Set up integrator
    //auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    //sys.SetTimestepper(stepper);
    //// Alternative way of changing the integrator:
    //////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    //////auto stepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());

    //stepper->SetAlpha(-0.2);
    //stepper->SetMaxIters(5);
    //stepper->SetAbsTolerances(1e-2);
    //stepper->SetStepControl(true);
    //stepper->SetMinStepSize(1e-4);
    //////stepper->SetVerbose(true);



    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Set up integrator
    auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    sys.SetTimestepper(stepper);
    // Alternative way of changing the integrator:
    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto stepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());

    stepper->SetAlpha(-0.2);
    stepper->SetMaxIters(50);
    stepper->SetAbsTolerances(1e-5);
    //stepper->SetStepControl(true);
    //stepper->SetMinStepSize(1e-4);
    stepper->SetModifiedNewton(true);
    ////stepper->SetVerbose(true);


    // Simulation loop

    double time = 0;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.001);
        time += 0.001;
        //std::cout << "Time = " << time << std::endl;
    }

    return 0;
}
