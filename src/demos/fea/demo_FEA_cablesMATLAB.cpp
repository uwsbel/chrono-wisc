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
// FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
//       Uses the Chrono MATLAB module
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_matlab/ChMatlabEngine.h"
#include "chrono_matlab/ChSolverMatlab.h"

#include "FEAvisualization.h"
#include "FEAcables.h"

using namespace chrono;
using namespace fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create one of the available models (defined in FEAcables.h)
    ////auto model = Model1(sys, my_mesh);
    ////auto model = Model2(sys, my_mesh);
    auto model = Model3(sys, my_mesh);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    mvisualizebeamA->SetColormapRange(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the run-time visualization system
    auto vis =
        CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Cables FEM (Matlab)", ChVector3d(0, 0.6, -1.0));

    // Change solver to Matlab external linear solver.
    ChMatlabEngine matlab_engine;
    auto matlab_solver = chrono_types::make_shared<ChSolverMatlab>(matlab_engine);
    sys.SetSolver(matlab_solver);

    // Change type of integrator:
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    ////sys.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxIters(2);
        mystepper->SetAbsTolerances(1e-6);
    }

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
        ////model.PrintBodyPositions();
    }

    return 0;
}
