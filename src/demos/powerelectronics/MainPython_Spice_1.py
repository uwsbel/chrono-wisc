# ==================================================================================================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# ==================================================================================================================================================
# Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
# ==================================================================================================================================================

# ======== IMPORT THE LIBRERIES ========
import pickle
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import sys
import PySpice
import PySpice.Logging.Logging as Logging
from PySpice.Spice.Netlist import Circuit
from PySpice.Doc.ExampleTools import find_libraries
from PySpice.Spice.Library import SpiceLibrary
from PySpice.Spice.Parser import SpiceParser
from PySpice.Spice.NgSpice.Shared import NgSpiceShared
#import matlab.engine
import time
from scipy.io import savemat
import os
import json

logger=Logging.setup_logging()
libraries_path = find_libraries()
spice_library = SpiceLibrary(libraries_path)

ngspice = NgSpiceShared.new_instance()
        
def CircuitAnalysis(t_step,t_end):
    try:
        # ======== OPEN THE CIRCUIT ========
        with open('SPICE_circuit\\Models\\Circuit_Netlist.cir') as f:
            circuit = f.read()

        # ======== PARSE THE CIRCUIT ========
        source = str(circuit)
        parser = SpiceParser(source=source)
        bootstrap_circuit = parser.build_circuit()

        bootstrap_source = str(bootstrap_circuit)

        # ======== DEFINE THE SIMULATOR AND RUN THE ANALYSIS ========
        simulator = bootstrap_circuit.simulator(temperature=25, nominal_temperature=25)
        try:
            # Randomize the SPICE integration step with the same order of magnitude declared in the call
            t_step_exponent = int(math.floor(math.log10(abs(t_step))))
            t_step_mantissa = t_step / (10 ** t_step_exponent)
            t_step_rand = random.random()
            t_step_new = (t_step_mantissa + t_step_rand) * 10 ** t_step_exponent
            analysis = simulator.transient(step_time=t_step_new, end_time=t_end,use_initial_condition=True)
        except: # EXCEPT 1
            # Retry to randomize the SPICE integration step with the same order of magnitude declared in the call
            t_step_exponent = int(math.floor(math.log10(abs(t_step))))
            t_step_mantissa = t_step / (10 ** t_step_exponent)
            t_step_rand = random.random()
            t_step_new = (t_step_mantissa + t_step_rand) * 10 ** t_step_exponent
            analysis = simulator.transient(step_time=t_step_new, end_time=t_end,use_initial_condition=True)

        # ======== EXTRACT THE SYMULATION RESULTS ========
        Simtime=np.array(analysis.time)

        node_val=[]
        node_name=[]
        for node in analysis.nodes.values():
            data_label="%s" % str(node) # extract node name
            node_name.append(data_label)
            data_value=np.array(node) # save node value/array of values
            node_val.append(data_value)
            #node_val.append(data_value.tolist())
          
        branch_val=[]
        branch_name=[]
        for node in analysis.branches.values():
            data_label="%s" % str(node) # extract node name
            branch_name.append(data_label)
            data_value=np.array(node) # save node value/array of values
            branch_val.append(data_value)
            #branch_val.append(data_value.tolist())
        
        # ======== Convert to a dictionary ========
        variables_dict = {'Simtime' : Simtime.tolist(), 'node_val' : [arr.tolist() for arr in node_val], 'node_name' : node_name, 'branch_val' : [arr.tolist() for arr in branch_val], 'branch_name' : branch_name}
        # Save to a file   
        with open('SPICE_circuit/PySpice_Error_BackUp/PySpice_Error_BackUp.json', 'w') as file:
            json.dump(variables_dict, file)

        # ======== Resturn ========
        return Simtime, node_val, node_name, branch_val, branch_name
    
    except: # EXCEPT 2
        # ======== Log the error ========
        if os.path.exists('LOG_PYTHON_SPICE.txt'):
            with open('LOG_PYTHON_SPICE.txt', 'a') as file:
                file.write('\n!!! WARNING !!!: EXCEPTION --> Occurred during the PySpice resolution of the circuit')
        else:
            with open('LOG_PYTHON_SPICE.txt', 'w') as file:
                file.write('!!! WARNING !!!: EXCEPTION --> Occurred during the PySpice resolution of the circuit')
        
        # ======== Load the stored Data ========
        with open('SPICE_circuit/PySpice_Error_BackUp/PySpice_Error_BackUp.json', 'r') as json_file:
            loaded_data = json.load(json_file)

        # ======== Extract the neaded variables in list form ========
        Simtime_1 = loaded_data["Simtime"]  
        node_val_1 = loaded_data["node_val"]
        node_name_1 = loaded_data["node_name"]
        branch_val_1 = loaded_data["branch_val"]
        branch_name_1 = loaded_data["branch_name"]

        # ======== Convert the variables if neaded ========
        Simtime = np.array(Simtime_1)
        node_val = [np.array(lst) for lst in node_val_1]
        node_name = node_name_1
        branch_val = [np.array(lst) for lst in branch_val_1]
        branch_name = branch_name_1

        # ======== Resturn ========
        return Simtime, node_val, node_name, branch_val, branch_name
        #pass









