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

class ChElectronicCircuits {  
public:

    typedef struct CircuitFiles {
        string netlist;
        string input;
        string output;
        string pwl_sources;
        string param_ic;
        string python_simulator;
    };

    CircuitFiles files;
    ChElectronicsCosimulation cosim;

    ChElectronicCircuits(CircuitFiles files) {
        this->files = files;
    }

    void Initialize() {
        
    }

    void Advance () {

    }


};

#endif // CHELECTRONICSCIRCUITS_H