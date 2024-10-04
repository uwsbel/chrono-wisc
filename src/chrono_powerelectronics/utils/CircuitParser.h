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

#ifndef CHELECTRONICSCIRCUITPARSER_H
#define CHELECTRONICSCIRCUITPARSER_H

#include <stdlib.h>
#include "NetlistStrings.h"

class CircuitParserIO {
public:

    typedef struct CircuitDirs {
        std::string NETLIST_name;                            // Path where to find the Circuit_Netlist.cir file starting from the working-directory folder       
        std::string INPUT_name;                            // Path where to find the INPUT_Netlist.txt file starting from the working-directory folder
        std::string OUTPUT_name;                             // Path where to find the OUTPUT_Netlist.txt file starting from the working-directory folder
        std::string PWL_sources_name;                        // Path where to find the PWL_sources.txt file starting from the working-directory folder
        std::string Param_IC_name;                           // Path where to find the Param_IC.txt file starting from the working-directory folder
    };
    typedef struct CircuitDefs {
        std::vector<std::string> NETLIST_contents;
        std::vector<std::string> INPUT_contents;                  // Contains the lists of the INPUT parameters 
        std::vector<std::string> OUTPUT_contents;                 // Contains the lists of the OUTPUT parameters
        std::vector<std::string> PWL_sources_contents;            // Contains the lists of the PWL sources
        std::vector<std::string> Param_IC_contents;               // Contains the lists of the IC values for the INPUT parameters
    };


    CircuitDefs ParseCircuit(CircuitDirs files) {
        CircuitDefs parser_output;
        parser_output.NETLIST_contents = NetlistStrings::Read_File(files.NETLIST_name); 
        parser_output.INPUT_contents = NetlistStrings::Read_File(files.INPUT_name); 
        parser_output.OUTPUT_contents = NetlistStrings::Read_File(files.OUTPUT_name); 
        parser_output.PWL_sources_contents = NetlistStrings::Read_File(files.PWL_sources_name);  
        parser_output.Param_IC_contents = NetlistStrings::Read_File(files.Param_IC_name); 

        return parser_output;
    }

    void RefreshNetlist(CircuitDirs input, CircuitDefs& defs) {
        defs.NETLIST_contents = NetlistStrings::Read_File(input.NETLIST_name); 
    }
};

#endif