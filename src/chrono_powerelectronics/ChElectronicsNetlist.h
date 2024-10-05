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
typedef std::map<std::string,double> PWLInMap;
typedef std::map<std::string,double> FlowInMap;
typedef std::map<std::string,double> VoltageMap;

class ChElectronicsNetlist {
public:

    Netlist_V netlist_file;
    CosimResults last_results;
    FlowInMap initial_flowin_ics;
    PWLInMap  initial_pwl_in;
    PWLInMap last_pwl_in{};

    /* Initialization */    
    void InitNetlist(std::string file, double t_step, double t_end);

    Netlist_V ReadNetlistFile(std::string file);

    // TODO: Get initial PWL source states (assumed constant from t_0->t_f)

    void SetInitialPWLIn(PWLInMap map) {
        this->initial_pwl_in = map;
    }

    void SetInitialFlowInICs(FlowInMap map) {
        this->initial_flowin_ics = map;
    }


    std::string toLowerCase(const std::string& str) {
        std::string lower_str = str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                    [](unsigned char c){ return std::tolower(c); });
        return lower_str;
    }


    // For each key in pwl_in, and from last,
    // set V_0 and V_f in pwl_sources accordingly
    PWLSourceMap GetPWLConds(PWLInMap pwl_in) {
        PWLSourceMap pwl_sources;

        // Check if last_pwl_in is empty
        bool last_pwl_in_empty = last_pwl_in.empty();

        // Iterate through pwl_in from the last to the first
        for (auto it = pwl_in.rbegin(); it != pwl_in.rend(); ++it) {
            const std::string& key = it->first;
            double V_f = it->second;
            
            // Set V_0 based on last_pwl_in, or initialize with V_f if last_pwl_in is empty
            double V_0;
            if (last_pwl_in_empty) {
                V_0 = V_f; // If last_pwl_in is empty, initialize V_0 with V_f
            } else {
                V_0 = last_pwl_in[key]; // Otherwise, use the value from last_pwl_in
            }

            // Add the pair (V_0, V_f) to the pwl_sources map
            pwl_sources[key] = std::make_pair(V_0, V_f);
        }

        // Store the last pwl_in for future use
        this->last_pwl_in = pwl_in;

        return pwl_sources;
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
    void UpdateNetlist(CosimResults results, FlowInMap map, PWLInMap pwl_in, double t_step, double t_end);

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

    // typedef std::vector<std::string> Netlist_V;
    void WriteNetlist(const std::string& file) {
        // Open the file in write mode
        std::ofstream outFile(file);

        // Check if the file is successfully opened
        if (!outFile.is_open()) {
            std::cerr << "Error: Could not open the file " << file << " for writing." << std::endl;
            return;
        }

        // Write each line of the netlist to the file
        for (const auto& line : this->netlist_file) {
            outFile << line << std::endl;
        }

        // Close the file
        outFile.close();

        // Confirm the write operation
        std::cout << "Netlist successfully written to " << file << std::endl;
    }
    
};

#endif