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

// =======================
// ======== Class ========
// =======================
class ChElectronicsCosimulation {  
public:
    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronicsCosimulation(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var) {
        NETLIST_name = NETLIST_name_var;                            
        INPUT_name = INPUT_name_var;                              
        OUTPUT_name = OUTPUT_name_var;                             
        PWL_sources_name = PWL_sources_name_var;                        
        Param_IC_name = Param_IC_name_var;                           
        Python_simulator_name = Python_simulator_name_var;                   

    }

    std::vector<double> sim_time;                                // Contains the SPICE simulation time returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> node_val;                   // Contains the voltage values at every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::string> node_name;                         // Contains the names of every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> branch_val;                 // Contains the current values at every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<std::string> branch_name;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class
    
    std::vector<bool> PWL_sources_NETLIST_idx;      // Contains the lists of order indexes of PWL sources in accordance to the appearance in the NETLIST 
    int sim_step = 1;                               // Initialize the simulation step counter to 1
    double t_step_electronic;                        // Time step for the SPICE solver intergration
    double T_sampling_electronic;                    // Time window of the electronic (SPICE) simulation
    double sim_time_last;                            // Last element of the SPICE sim_time_vector to reallocate the local SPICE sim_time array, respect to the global simulation time line

    std::vector<std::string> INPUT_contents;                  // Contains the lists of the INPUT parameters 
    std::vector<std::string> OUTPUT_contents;                 // Contains the lists of the OUTPUT parameters
    std::vector<std::string> PWL_sources_contents;            // Contains the lists of the PWL sources
    std::vector<std::string> Param_IC_contents;               // Contains the lists of the IC values for the INPUT parameters
    std::vector<std::string> PWL_sources_NETLIST_reordered;   // Contains the lists of the PWL sources reordered in accordance to the appearance in the NETLIST
    
    std::string NETLIST_name;                            // Path where to find the Circuit_Netlist.cir file starting from the working-directory folder       
    std::string INPUT_name;                              // Path where to find the INPUT_Netlist.txt file starting from the working-directory folder
    std::string OUTPUT_name;                             // Path where to find the OUTPUT_Netlist.txt file starting from the working-directory folder
    std::string PWL_sources_name;                        // Path where to find the PWL_sources.txt file starting from the working-directory folder
    std::string Param_IC_name;                           // Path where to find the Param_IC.txt file starting from the working-directory folder
    std::string Python_simulator_name;                   // Name of the Python module that run and pass the results

    py::object mymodule;                    // Import the Python module -> c_str() allows to convert a string to a char string, the File_name does not need the extension .py
    py::tuple data;                         // Call the desired method from the Python module

    double t_clock = 0.0;                            // Global clock of the simulation
    std::vector<double> OUTPUT_value;                // OUTPUT values to return to the caller

    // ==========================================
    // ======== Methods: SPICE execution ========
    // ==========================================
        // ======== Method: allows to set the class t_clock global variable ========
    void Set_t_clock_var(double t_clock_var);

        // ======== Method: allows to read a NETLIST circuit ========
    std::vector<std::string> Read_NETLIST(std::string File_name);

        // ======== Method: allows to read the INPUT list file ========
    std::vector<std::string> Read_INPUT_Parameters(std::string File_name);

        // ======== Method: allows to read the OUTPUT list file ========
    std::vector<std::string> Read_OUTPUT_Parameters(std::string File_name);

        // ======== Method: allows to read the Piece Wise Linear sowrces list file ========
    std::vector<std::string> Read_PWL_sources(std::string File_name);

        // ======== Method: allows to read the Parameters Initial Conditions values list file ========
    std::vector<std::string> Read_Param_IC(std::string File_name);

        // ======== Method: allows to run a Spice Netlist simulation and solve the circuit ========
    void Run_Spice(std::string File_name, std::string Method_name, double t_step, double t_end);

        // ======== Method: allows to initialize the PWL sources present in the circuit ========
    void PWL_sources_Initializer(std::vector<std::string>& PWL_sources_contents, std::vector<std::string>& NETLIST_contents);   // To fix generator co-simulation dynamics problematics

        // ======== Method: allows to initialize the inductances present in the circuit ========
    void Inductances_Initializer(std::vector<std::string>& NETLIST_contents); // To fix IC inductances current problematics during co-simulation
    
        // ======== Method: allows to initialize the NETLIST file for co-simulation ========
    void NETLIST_Initializer();

        // ======== Method: allows to initialize the NETLIST file and to extract the SPICE simulation results at every time step of the electronic call ========
    void NETLIST_Cosimulator(std::vector<std::vector<double>>& INPUT_values, double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);

};

#endif // CHELECTRONICSEXECUTOR_H