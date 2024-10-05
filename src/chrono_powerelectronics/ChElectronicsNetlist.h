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
#include <algorithm>    // For std::find

typedef std::vector<std::string> Netlist_V;
typedef std::map<std::string,std::pair<double,double>> PWLSourceMap;
typedef std::map<std::string,double> FlowInMap;
typedef std::map<std::string,double> VoltageMap;

class ChElectronicsNetlist {
public:

    Netlist_V netlist_file;
    CosimResults last_results;

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

    std::string toLowerCase(const std::string& str) {
        std::string lower_str = str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                    [](unsigned char c){ return std::tolower(c); });
        return lower_str;
    }


    // For each key in GetInitPWLSources(), and from a cached result_last,
    // set V_0 and V_f accordingly
    PWLSourceMap GetPWLConds(CosimResults results) {
        PWLSourceMap init_pwl_sources = GetInitPWLSources();
        PWLSourceMap pwl_sources = init_pwl_sources;

        // Iterate through each key in the PWL source map
        for (auto& [key, values] : pwl_sources) {
            // Find the index corresponding to the key in the node_name vector
            auto it = std::find(results.branch_name.begin(), results.branch_name.end(), toLowerCase(key));
            
            if (it != results.branch_name.end()) {
                int idx = std::distance(results.branch_name.begin(), it);
                int size = results.branch_val[idx].size() - 1;

                // Update the initial and final values using the current and previous results
                if(this->last_results.branch_val.empty()) {
                    values.first = init_pwl_sources[key].first;
                } else {
                    values.first = this->last_results.branch_val[idx][size];  // V_0 (initial value from last result)
                }
                values.second = results.branch_val[idx][size];       // V_f (final value from current results)
            } else {
                // Handle the case where the key is not found in node_name (optional)
                std::cerr << "Warning: Key '" << key << "' not found in node_name." << std::endl;
            }
        }

        this->last_results = results;

        return pwl_sources;
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

    // For all nodes, set according to results.node_val
    // and results.node_name vectors

    VoltageMap GetVoltageConds(const CosimResults& results) {
        VoltageMap voltageMap;

        // Ensure that node_name and node_val sizes match
        if (results.node_name.size() != results.node_val.size()) {
            throw std::runtime_error("Mismatch between node names and voltage values per node");
        }

        // Check if there are any time steps to consider
        if (results.node_val.empty() || results.node_val.back().empty()) {
            throw std::runtime_error("No voltage data available");
        }

        // Populate the map with the node names and their corresponding voltage values for the last time step
        for (size_t i = 0; i < results.node_name.size(); ++i) {
            double last_time_step_voltage = results.node_val[i].back();
            voltageMap[results.node_name[i]] = last_time_step_voltage;
        }

        return voltageMap;
    }

    /* Cosimulation / Pre-warming */
    void UpdateNetlist(CosimResults results, FlowInMap map, double t_step, double t_end);

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