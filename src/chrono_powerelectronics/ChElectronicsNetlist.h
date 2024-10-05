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

#ifndef CHELECTRONICSNETLIST_H
#define CHELECTRONICSNETLIST_H

// =========================
// ======== Headers ========
// =========================
#include "ChElectronicsCosimResult.h"
#include <map>
#include <iostream>
#include <fstream> 
#include <sstream> 
#include <string>   

typedef std::vector<std::string> Netlist_V;
typedef std::map<std::string,std::pair<double,double>> PWLSourceMap;
typedef std::map<std::string,double> FlowInMap;
typedef std::map<std::string,double> VoltageMap;

class ChElectronicsNetlist {
public:

    Netlist_V netlist_file;
    
    /* Initialization */    
    void InitNetlist(std::string file, double t_step, double t_end);

    Netlist_V ReadNetlistFile(std::string file);

    // TODO: Get initial PWL source states (assumed constant from t_0->t_f)
    PWLSourceMap GetInitPWLSources() {
        return {
           {"VgenVAR", { 0., 0., }},
           {"VbackemfCVAR", {0., 0.,}},
           {"VSW1VAR", {0., 0., }},
           {"VgenPWMVAR", { 0., 0., }}
        };
    }

    /* Get initial conditions of flow-in variables at first time step
    */
    FlowInMap GetInitFlowInICs() {
        return {
           {"VgenVAR", 0. },
           {"VbackemfCVAR", 0. },
           {"LaC", 45.0e-6},
           {"RaC", 25.1},
           {"VSW1VAR", 0. },
           {"VgenPWMVAR",  0.}
        };
    }


    /* Cosimulation / Pre-warming */
    void UpdateNetlist(CosimResults results, double t_step, double t_end);

    /* For every key in FlowInMap, initialize or update a .param par{...} string in the netlist */
    Netlist_V UpdateFlowInParams(Netlist_V netlist, FlowInMap map);           

    /* For every key in PWLSourceMap, initialize a PWL(...) string in the netlist */
    Netlist_V UpdatePWLSources(Netlist_V netlist, PWLSourceMap map, double t_step, double t_end);  // PWL(...)

    /* For every key in VoltageMap, initialize or update the corresponding V({key})=value string on the .ic line */
    Netlist_V UpdateVoltageICs(Netlist_V netlist, VoltageMap map); // .ic V(...)

    std::string AsString();

    /*
    *   Create a string of form 
    *   PWL(t_0 V_0 t_1 V_1, ..., t_f V_f) 
    *   V_i is a linearly interpolated value between V_0,V_f 
    */
    std::string GeneratePWLSequence(
        std::pair<double,double> V,  // Order: [V_0, V_f]
        double t_start, double t_end);


    // void SubstitutePWL();

    

};

#endif