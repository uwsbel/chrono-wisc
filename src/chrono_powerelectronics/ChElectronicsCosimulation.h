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

#ifndef CHELECTRONICSEXECUTOR_H
#define CHELECTRONICSEXECUTOR_H

// =========================
// ======== Headers ========
// =========================
#include "utils/NetlistStrings.h"
#include "ChElectronicsNetlist.h"
#include "utils/CircuitParser.h"

// =======================
// ======== Class ========
// =======================

class ChElectronicsCosimulation {  
public:
    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronicsCosimulation(std::string netlist, std::string input, std::string output, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string python_sim_dir, std::string method_name) {
        this->Initialize(CircuitParserIO::CircuitDirs{netlist, input, output, PWL_sources_name_var,
            Param_IC_name_var}, python_sim_dir, method_name);
    }

    ChElectronicsCosimulation() {

    }

    void Initialize(CircuitParserIO::CircuitDirs dirs, std::string python_sim_name, std::string method_name) 
    {
        parser_input = CircuitParserIO::CircuitDirs {
            dirs.NETLIST_name, dirs.INPUT_name, dirs.OUTPUT_name, dirs.PWL_sources_name,
            dirs.Param_IC_name
        };

        this->python_sim_dir = python_sim_name;   
        this->method_name = method_name;

        this->Initialize();
    }

    void SetDataDir(std::string data_dir) {
        this->data_dir = data_dir;
    }

    // ==========================================
    // ======== Methods: SPICE execution ========
    // ==========================================

        // ======== Method: allows to run a Spice Netlist simulation and solve the circuit ========
    void RunSpice(std::string File_name, std::string Method_name, double t_step, double t_end);

    // ======== Method: allows to initialize the NETLIST file and to extract the SPICE simulation results at every time step of the electronic call ========
    std::map<std::string,std::vector<double>> Cosimulate(std::vector<std::vector<double>>& INPUT_values, double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);

    void Initialize();

    double GetCurrStep() {
        return sim_step;
    }

private:

    py::object mymodule;                    // Import the Python module -> c_str() allows to convert a string to a char string, the File_name does not need the extension .py
    py::tuple data;                         // Call the desired method from the Python module

    int sim_step = 1;                               // Initialize the simulation step counter to 1
    double t_clock = 0.0;                            // Global clock of the simulation

    typedef struct CosimResults {
        std::vector<double> sim_time;                                // Contains the SPICE simulation time returned by the Python module, it is update at every call of the class
        std::vector<std::vector<double>> node_val;                   // Contains the voltage values at every SPICE circuit nodes returned by the Python module, it is update at every call of the class
        std::vector<std::string> node_name;                         // Contains the names of every SPICE circuit nodes returned by the Python module, it is update at every call of the class
        std::vector<std::vector<double>> branch_val;                 // Contains the current values at every SPICE circuit branches returned by the Python module, it is update at every call of the class
        std::vector<std::string> branch_name;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class
    };

    CircuitParserIO parser;
    CircuitParserIO::CircuitDefs parser_output;
    CircuitParserIO::CircuitDirs parser_input;

    std::string python_sim_dir;                   // Name of the Python module that run and pass the results
    std::string method_name;
    std::string data_dir;
    
    CosimResults results;


    std::vector<bool> PWL_sources_NETLIST_idx;      // Contains the lists of order indexes of PWL sources in accordance to the appearance in the NETLIST 
    std::vector<std::string> PWL_sources_NETLIST_reordered;   // Contains the lists of the PWL sources reordered in accordance to the appearance in the NETLIST

    // ======== Method: allows to initialize the PWL sources present in the circuit ========
    void PWL_sources_Initializer(std::vector<std::string>& PWL_sources_contents, std::vector<std::string>& NETLIST_contents);   // To fix generator co-simulation dynamics problematics

    // ======== Method: allows to initialize the inductances present in the circuit ========
    void Inductances_Initializer(std::vector<std::string>& NETLIST_contents); // To fix IC inductances current problematics during co-simulation
    
    void UpdateInductanceVoltage();

    void WriteNetlist(std::string file, std::vector<std::string> contents);

    // ======== Method: allows to initialize the NETLIST file for co-simulation ========
    void InitializeNetlist();

    void IncrementStep() {
        sim_step++;
    }

};

#endif // CHELECTRONICSEXECUTOR_H