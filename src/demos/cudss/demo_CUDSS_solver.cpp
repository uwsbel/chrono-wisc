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
// Authors: Ganesh Arivoli
// =============================================================================
//
// Example usage of ChSolverCuDSS
// Demonstrates how to use the cuDSS GPU solver in a Chrono simulation
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#ifdef CHRONO_CUDSS
    #include "chrono_cudss/ChSolverCuDSS.h"
#endif

using namespace chrono;

int main() {
#ifdef CHRONO_CUDSS
    std::cout << "Creating Chrono system with cuDSS GPU solver\n";

    // Create a Chrono physical system (SMC contact: direct solvers are not valid for NSC/VI)
    ChSystemSMC system;

    // Create and configure the cuDSS solver
    auto cudss_solver = chrono_types::make_shared<ChSolverCuDSS>();
    std::cout << "Solver object created\n";

    // Enable sparsity pattern lock for better performance in time-stepping
    cudss_solver->LockSparsityPattern(true);

    // Use the sparsity pattern learner (enabled by default)
    cudss_solver->UseSparsityPatternLearner(true);

    // Set the solver
    system.SetSolver(cudss_solver);
    std::cout << "Solver set on system\n";

    // Set up your simulation objects here
    // Example: Add a simple body
    auto body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000);
    body->SetPos(ChVector3d(0, 5, 0));
    system.Add(body);

    // Simulation parameters
    double time_step = 0.01;
    double end_time = 1.0;

    std::cout << "Running simulation with cuDSS solver...\n";

    // Simulation loop
    while (system.GetChTime() < end_time) {
        system.DoStepDynamics(time_step);

        if (((int)(system.GetChTime() / time_step)) % 10 == 0) {
            std::cout << "Time: " << system.GetChTime()
                      << "  Position: " << body->GetPos().y() << "\n";
        }
    }

    std::cout << "Simulation completed successfully!\n";
    return 0;

#else
    std::cerr << "ERROR: Chrono was not built with cuDSS support\n";
    std::cerr << "Please rebuild with -DCH_ENABLE_MODULE_CUDSS=ON\n";
    return 1;
#endif
}
