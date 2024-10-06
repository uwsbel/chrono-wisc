# ======== IMPORT THE LIBRARIES ========
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
import time
import os
import json
import traceback  # Import traceback for detailed exception info
import pandas as pd
import io
import logging 
import re

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('PySpice.CircuitAnalysis')

libraries_path = find_libraries()
spice_library = SpiceLibrary(libraries_path)

ngspice = NgSpiceShared.new_instance()
ngspice.stdout_output_enabled = True

def extract_data(element_names, command_prefix):

    data_frames = []

    for element in element_names:
        try:
            command = f"{command_prefix}({element})"
            output = ngspice.exec_command(f'print {command}')
            output = "\n".join(line for line in output.splitlines()[1:] 
                               if "Index" not in line and "---" not in line and "Transient" not in line)
            
            column_names = ['Index', 'time', f'{element}']
            df = pd.read_csv(io.StringIO(output), sep=r'\s+', names=column_names)
            df.drop(columns=['Index'], inplace=True)  
            data_frames.append(df)

        except Exception as e:
            logger.error(f"Error while extracting data for {element}: {str(e)}")
            logger.error("Traceback details:\n" + traceback.format_exc())
            continue
    
    return data_frames

def CircuitAnalysis(t_step, t_end):
    try:
        # print("Starting SPICE iteration")

        # ======== OPEN THE CIRCUIT ========
        with open('../data/Circuit/MotorControl/Circuit_Netlist.cir.run') as f:
            circuit = f.read()

        source = str(circuit)

        t_step_exponent = int(math.floor(math.log10(abs(t_step))))
        t_step_mantissa = t_step / (10 ** t_step_exponent)
        t_step_rand = random.random()
        t_step_new = (t_step_mantissa + t_step_rand) * 10 ** t_step_exponent
        # Append the transient analysis command

        
        trans_str = f'\n.tran {t_step_new} {t_end}\n.end\n'
        source += trans_str

        # Load and run the circuit with ngspice
        ngspice.load_circuit(source)
        ngspice.run()

        # Define node names and branch names
        node_name = [
            'n5', 'n1', 'n4', 'n3', 'n2', 'n6', 'n12', 
            'n10', 'x1.9', 'x1.7', 'x1.8', 'n7', 'n8', 
            'x1.4', 'x1.5', 'x1.10', 'x1.11', 'x1.6', 
            'n11', 'n9'
        ]

        branch_name = [
            'v.x1.vfi1', 'v.x1.vfi2', 'lac', 
            'e.x1.ev16', 'vmos1', 'vgenpwmvar', 
            'vsw1var', 'vgenvar', 'vprobe1', 
            'vbackemfcvar'
        ]

        # Use the helper function to extract node and branch data
        node_data_frames = extract_data(node_name, "V")
        branch_data_frames = extract_data(branch_name, "I")
        df_nodes = pd.concat(node_data_frames, axis=1)
        df_nodes = df_nodes.loc[:, ~df_nodes.columns.duplicated()]
        df_branches = pd.concat(branch_data_frames, axis=1)
        df_branches = df_branches.loc[:, ~df_branches.columns.duplicated()]


        sim_time = df_branches['time'].values
        branch_names = df_branches.columns[1:]  # All columns except 'time'
        branch_values = df_branches[branch_names].values.T  # Transpose for easier access
        node_names = df_nodes.columns[1:]  # All columns except 'time'
        node_values = df_nodes[node_names].values.T  # Transpose for easier access

        assert len(node_names) > 0
        assert len(node_values) > 0

        # print(df_branches.columns)  # Debugging: check branch columns
        # print(df_nodes.columns)     # Debugging: check node columns

        # print("...Finished SPICE iteration")

        # print(df_branches['vprobe1'])

        return sim_time, node_values.tolist(), node_names.tolist(), branch_values.tolist(), branch_names.tolist()

    except Exception as e:  # Catch any exception and log details
        error_message = f"An error occurred during circuit analysis: {str(e)}"
        logger.error(error_message)
        logger.error("Traceback details:\n" + traceback.format_exc())
        print("Error occurred")

# Call the function
# for i in range(0,10):
#     CircuitAnalysis(1e-6, 0.0002)
