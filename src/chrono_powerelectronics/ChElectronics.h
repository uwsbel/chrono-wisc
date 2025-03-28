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

#ifndef CHELECTRONICS_H
#define CHELECTRONICS_H

// =========================
// ======== Headers ========
// =========================
#if defined(_DEBUG)
#undef _DEBUG
#include <python.h>
#define _DEBUG
#else
#include <python.h>
#endif
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <filesystem>
#include <exception>
#include <iomanip>
#define _USE_MATH_DEFINES 
#include <math.h>
#include <numeric> // for std::accumulate
#include <stdexcept>
#include <iomanip>
#include <stdexcept>
#include <future>
#include <typeinfo>
// #include <chrono> // Header, which provides a high-resolution clock

#include "ChElectronics.h"

// ============================
// ======== Namespaces ========
// ============================
// using namespace std;
namespace py = pybind11;
namespace fs = std::filesystem;

class ChElectronics { 
public:
    // ============================
    // ======== Attributes ========
    // ============================
        // ======== Path of the SPICE model files ========
    std::string NETLIST_name;                            // Path where to find the Circuit_Netlist.cir file starting from the working-directory folder       
    std::string INPUT_name;                              // Path where to find the INPUT_Netlist.txt file starting from the working-directory folder
    std::string OUTPUT_name;                             // Path where to find the OUTPUT_Netlist.txt file starting from the working-directory folder
    std::string PWL_sources_name;                        // Path where to find the PWL_sources.txt file starting from the working-directory folder
    std::string Param_IC_name;                           // Path where to find the Param_IC.txt file starting from the working-directory folder
    std::string Python_simulator_name;                   // Name of the Python module that run and pass the results

        // ======== Contents of the SPICE model files ========
    std::vector<std::string> INPUT_contents;                  // Contains the lists of the INPUT parameters 
    std::vector<std::string> OUTPUT_contents;                 // Contains the lists of the OUTPUT parameters
    std::vector<std::string> PWL_sources_contents;            // Contains the lists of the PWL sources
    std::vector<std::string> Param_IC_contents;               // Contains the lists of the IC values for the INPUT parameters
    std::vector<std::string> PWL_sources_NETLIST_reordered;   // Contains the lists of the PWL sources reordered in accordance to the appearance in the NETLIST

        // ======== Global variables ========
    std::vector<bool> PWL_sources_NETLIST_idx;      // Contains the lists of order indexes of PWL sources in accordance to the appearance in the NETLIST 
    int sim_step = 1;                               // Initialize the simulation step counter to 1
    double t_step_electronic;                        // Time step for the SPICE solver intergration
    double T_sampling_electronic;                    // Time window of the electronic (SPICE) simulation
    double sim_time_last;                            // Last element of the SPICE sim_time_vector to reallocate the local SPICE sim_time array, respect to the global simulation time line

        // ======== SPICE solutions ========
    std::vector<double> sim_time;                                // Contains the SPICE simulation time returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> node_val;                   // Contains the voltage values at every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::string> node_name;                         // Contains the names of every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> branch_val;                 // Contains the current values at every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<std::string> branch_name;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class
    
        // ======== Calss INPUT & OUTPUT variables ========
    double t_clock = 0.0;                            // Global clock of the simulation
    std::vector<double> OUTPUT_value;                // OUTPUT values to return to the caller

        // ======== Python global variables ========  
    //py::scoped_interpreter guard{};       // The Python interpreter initialization must be done only once, and in the 'main'.cpp 
    py::object mymodule;                    // Import the Python module -> c_str() allows to convert a string to a char string, the File_name does not need the extension .py
    py::tuple data;                         // Call the desired method from the Python module

public: 
    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronics(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var);
};

#endif // CHELECTRONICS_H
