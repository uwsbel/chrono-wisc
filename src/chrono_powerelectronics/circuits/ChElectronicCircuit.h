// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

#ifndef CHELECTRONICSCIRCUITS_H
#define CHELECTRONICSCIRCUITS_H

// =========================
// ======== Headers ========
// =========================

#include "../utils/NetlistStrings.h"

// =======================
// ======== Class ========
// =======================

using namespace std;
using namespace CircuitParserIO;

class ChElectronicCircuit {  
public:

    // typedef struct CircuitFiles {
    //     string netlist;
    //     string input;
    //     string output;
    //     string pwl_sources;
    //     string param_ic;
    //     string python_simulator;
    // };

    CircuitFiles files;
    ChElectronicsCosimulation cosim;
    string python_simulator = "MainPython_Spice_1";

    double t_step;
    double t_end;

    std::vector<double> result;

    double t_sampling_electronic_counter = 0;
    double t_sim_electronics = 0;
    double t_sim_mechanics = 0;

    ChElectronicCircuits(CircuitDirs files, double t_step, double t_end) {
        this->files = files;
        this->t_step = t_step;
        this->t_end = t_end;
    }

    void Initialize() {
        cosim.Initialize(files, "CircuitAnalysis");
    }


    virtual void PreInitialize() {};
    virtual void PostInitialize() {};

    virtual void PreAdvance () {};

    void Advance() {
        cosim.RunSpice(python_simulator, "CircuitAnalysis", t_step, t_end);
        this->result = Circuit1.Cosimulate(INPUT_values, t_sim_electronics, t_step_electronic, T_sampling_electronic);
    }

    virtual void PostAdvance () {};


};

#endif // CHELECTRONICSCIRCUITS_H