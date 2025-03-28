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
#include "ChElectronicsManipulator.h"

// =======================
// ======== Class ========
// =======================
class ChElectronicsExecutor : public ChElectronicsManipulator {  
public:
    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronicsExecutor(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var);

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