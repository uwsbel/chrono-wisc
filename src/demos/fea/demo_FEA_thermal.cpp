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
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA visualization using Irrlicht
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChContinuumThermal.h"
#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkNodeFrame.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumThermal>();
    mmaterial->SetSpecificHeatCapacity(2);
    mmaterial->SetThermalConductivity(200);

    //
    // Add some TETAHEDRONS:
    //

    // Load a .node file and a .ele  file from disk, defining a complicate tetrahedron mesh.
    // This is much easier than creating all nodes and elements via C++ programming.
    // You can generate these files using the TetGen tool.
    try {
        ChMeshFileLoader::FromTetGenFile(my_mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                         GetChronoDataFile("fea/beam.ele").c_str(), mmaterial);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }

    for (unsigned int inode = 0; inode < my_mesh->GetNumNodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            mnode->SetPos(mnode->GetPos() * ChVector3d(3, 1, 3));
        }
    }

    //
    // Set some BOUNDARY CONDITIONS on nodes:
    //

    // Impose load on the 180th node
    auto mnode3 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(180));
    mnode3->SetLoad(20);  // thermal load: heat flux [W] into node

    // Impose field on two top nodes (remember the SetFixed(true); )
    auto mnode1 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(my_mesh->GetNumNodes() - 1));
    mnode1->SetFixed(true);
    mnode1->SetFieldVal(0.5);  // field: temperature [K]
    auto mnode2 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(my_mesh->GetNumNodes() - 2));
    mnode2->SetFixed(true);
    mnode2->SetFieldVal(0.5);  // field: temperature [K]

    // Impose field on the base points:
    for (unsigned int inode = 0; inode < my_mesh->GetNumNodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            if (mnode->GetPos().y() < 0.01) {
                mnode->SetFixed(true);
                mnode->SetFieldVal(10);  // field: temperature [K]
            }
        }
    }

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    // Do not forget AddVisualShapeFEA() at the end!

    // Paint the colored mesh with temperature scale (NODE_FIELD_VALUE is the scalar field of the Poisson problem)
    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_FIELD_VALUE);
    mvisualizemesh->SetColormapRange(-1, 12);
    mvisualizemesh->SetShrinkElements(false, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    // This will paint the wireframe
    auto mvisualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshB->SetWireframe(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshB);

    // This will paint the heat flux as line vectors
    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_VECT_DP);
    mvisualizemeshC->SetSymbolsScale(0.003);
    mvisualizemeshC->SetDefaultSymbolsColor(ChColor(0.1f, 0.2f, 0.2f));
    mvisualizemeshC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "FEA thermal", ChVector3d(0, 0.7, -1),
                                         ChVector3d(0, 0.4, 0));

    // SIMULATION LOOP

    // Use MINRES solver to handle stiffness matrices.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(150);
    solver->SetTolerance(1e-6);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    // Note: if you are interested only in a single LINEAR STATIC solution
    // (not a transient thermal solution, but rather the steady-state solution),
    // at this point you can uncomment the following line:
    //
    //  sys.DoStaticLinear();
    //
    // Also, in the following while() loop, remove  application.DoStep();
    // so you can spin the 3D view and look at the solution.

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);

        if (sys.GetChTime() > 5)
            break;
    }

    // Print some node temperatures..
    std::cout << "time = " << sys.GetChTime() << std::endl;
    for (unsigned int inode = 0; inode < my_mesh->GetNumNodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            if (mnode->GetPos().x() < 0.01) {
                std::cout << "Node at y=" << mnode->GetPos().y() << " has T=" << mnode->GetFieldVal() << "\n";
            }
        }
    }

    return 0;
}
