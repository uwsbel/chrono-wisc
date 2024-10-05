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

#include "../ChElectronicsCosimResult.h"
#include "../ChElectronicsCosimulation.h"

// =======================
// ======== Class ========
// =======================

using namespace std;

class ChElectronicCircuit {  
public:

    // typedef struct CircuitDirs {
    //     string netlist;
    //     string input;
    //     string output;
    //     string pwl_sources;
    //     string param_ic;
    // };

    ChElectronicCircuit(std::string netlist, double t_step, double t_end) {
        this->netlist = netlist;
        this->t_step = t_step;
        this->t_end = t_end;
    }

    virtual void PreInitialize() {};
    virtual void PostInitialize() {};

    virtual void PreAdvance () {};

    
    virtual void PostAdvance () {};

    virtual void Initialize() final {
        this->PreInitialize();
        cosim.Initialize(netlist, python_simulator);
        this->PostInitialize();
    }

    virtual void Advance(double dt_mbs) final {
        this->PreAdvance();
        t_sim_electronics += dt_mbs;
        this->result = cosim.RunSpice(python_simulator, t_step, t_end);
        cosim.Cosimulate(cosim.GetResult_V(), t_step, t_end);
        this->PostAdvance();
    }

    std::map<std::string,std::vector<double>> GetResult() {
        return this->result;
    }

private:

    std::string netlist;
    ChElectronicsCosimulation cosim;
    string python_simulator = "MainPython_Spice_1";

    double t_step;
    double t_end;


    double t_sim_electronics = 0;
    std::map<std::string,std::vector<double>> result;

protected:
    std::vector<std::vector<double>> flow_in;

};

#endif // CHELECTRONICSCIRCUITS_H