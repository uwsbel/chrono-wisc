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

// =========================
// ======== Headers ========
// =========================
#include "ChElectronicsCosimulation.h"


// ======== Method: allows to run a Spice Netlist simulation and solve the circuit ========
// CosimResults
std::map<std::string,std::vector<double>> ChElectronicsCosimulation::RunSpice(std::string file_name, double t_step, double t_end)
{
    std::vector<std::vector<double>> node_val;                   // Contains the voltage values at every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::string> node_name;                         // Contains the names of every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> branch_val;                 // Contains the current values at every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<std::string> branch_name;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<double> sim_time;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class

    // -------- Python call --------
    
    // Import the Python module    
    mymodule = py::module::import(file_name.c_str());            // c_str() allows to convert a string to a char string, the File_name does not need the extension .py 

    // Call the desired method from the Python module
    // std::cout << t_step << " " << t_end << std::endl;
    
    data = mymodule.attr("CircuitAnalysis")(t_step, t_end);

    // -------- Access the sim_time: (NumPy array (double array) from the tuple) --------
    sim_time.clear();
    py::array_t<double> doubleArray = data[0].cast<py::array_t<double>>();     // data[0] first element of the tuple file
    auto doubleBuffer = doubleArray.request();
    double* doublePtr = static_cast<double*>(doubleBuffer.ptr);

    for (ssize_t i = 0; i < doubleBuffer.size; ++i) {
        sim_time.push_back(doublePtr[i]);
    }

    // -------- Access the node_val: (list of NumPy arrays (array list) from the tuple) --------
    node_val.clear();
    py::list arrayList = data[1].cast<py::list>();
    // Print elements of each NumPy array in the list
    for (size_t i = 0; i < arrayList.size(); ++i) {
        py::array_t<double> npArray = arrayList[i].cast<py::array_t<double>>();
        auto npBuffer = npArray.request();
        double* npPtr = static_cast<double*>(npBuffer.ptr);
        std::vector<double> row;                                                      // Create the row of the matrix
        for (ssize_t j = 0; j < npBuffer.size; ++j) {
            row.push_back(npPtr[j]);                                            // Add the element to the row
        }
        node_val.push_back(row);                                                // Add the row to the matrix
    }
    
    // -------- Access the node_name: (list of strings from the tuple) --------
    node_name.clear();
    node_name = data[2].cast<std::vector<std::string>>();
    // Scope_String_Vector(node_name);               // DEBUG: Scope what desired for debug purposes


    // -------- Access the branch_val: (list of NumPy arrays (array list) from the tuple) --------
    branch_val.clear();
    py::list arrayList1 = data[3].cast<py::list>();
    // Print elements of each NumPy array in the list
    for (size_t i = 0; i < arrayList1.size(); ++i) {
        py::array_t<double> npArray1 = arrayList1[i].cast<py::array_t<double>>();
        auto npBuffer1 = npArray1.request();
        double* npPtr1 = static_cast<double*>(npBuffer1.ptr);
        std::vector<double> row1;                                                      // Create the row of the matrix
        for (ssize_t j = 0; j < npBuffer1.size; ++j) {
            row1.push_back(npPtr1[j]);                                          // Add the element to the row
        }
        branch_val.push_back(row1);                                             // Add the row to the matrix
    }

    // -------- Access the branch_name: (list of strings from the tuple) --------
    branch_name.clear();
    branch_name = data[4].cast<std::vector<std::string>>();
    //Scope_String_Vector(branch_name);               // DEBUG: Scope what desired for debug purposes  

    this->results = {
        sim_time, node_val, node_name, branch_val, branch_name
    };

    std::map<std::string, std::vector<double>> circuit_map;
    for (size_t i = 0; i < branch_name.size(); ++i) {
        circuit_map[branch_name[i]] = branch_val[i];
    }
    for (size_t i = 0; i < node_name.size(); ++i) {
        circuit_map[node_name[i]] = node_val[i];
    }

    IncrementStep();

    return circuit_map;
}

// void Cosimulate(CosimResults results, double t_step, double t_end)
void ChElectronicsCosimulation::Cosimulate(CosimResults results, FlowInMap flow_in, PWLInMap pwl_in, double t_step, double t_end) 
{
    this->netlist.UpdateNetlist(results,flow_in,pwl_in,t_step,t_end);
    this->netlist.WriteNetlist("../data/Circuit/MotorControl/Circuit_Netlist.cir.run");
}  

CosimResults ChElectronicsCosimulation::GetResult_V() {
    return this->results;
}