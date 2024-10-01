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

// =============================
// ======== Constructor ========
// =============================
// ChElectronicsCosimulation::ChElectronicsCosimulation(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var)
//     : ChElectronicsManipulator(NETLIST_name_var, INPUT_name_var, OUTPUT_name_var, PWL_sources_name_var, Param_IC_name_var, Python_simulator_name_var) {}

// ==========================================
// ======== Methods: SPICE execution ========
// ==========================================


// ======== Method: allows to run a Spice Netlist simulation and solve the circuit ========
// sim_time: A vector of doubles storing simulation times extracted from the first element of the tuple returned by CircuitAnalysis.
// results

void ChElectronicsCosimulation::Run_Spice(std::string File_name, std::string Method_name, double t_step, double t_end)
{
    std::vector<std::vector<double>> node_val;                   // Contains the voltage values at every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::string> node_name;                         // Contains the names of every SPICE circuit nodes returned by the Python module, it is update at every call of the class
    std::vector<std::vector<double>> branch_val;                 // Contains the current values at every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<std::string> branch_name;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class
    std::vector<double> sim_time;                       // Contains the names of every SPICE circuit branches returned by the Python module, it is update at every call of the class

    // -------- Python call --------
    
    // Import the Python module    
    mymodule = py::module::import(File_name.c_str());            // c_str() allows to convert a string to a char string, the File_name does not need the extension .py 

    // Call the desired method from the Python module
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
}

// ======== Method: allows to initialize the PWL sources present in the circuit ========
// No class variables
void ChElectronicsCosimulation::PWL_sources_Initializer(std::vector<std::string>& PWL_sources_contents, std::vector<std::string>& NETLIST_contents) // To fix generator co-simulation dynamics problematics
{
    // Initialize the PWL sources: Define the name of the PWL source, if no PWL present in the circuit PWL_V_source = []
    // if both Voltage and Current source are presents define as follow PWL_V_source = ["V1"; "I1"];
    // Suffix: VAR(= variable source) = Voltage / Current source that depends by the other physical domains
    // PRE(= predefined source) = Voltage / Current source that doesn't depends by the other physical domains, thus can be predefined at the beginning of the simulation
    // -------- Set the PWL sources file --------
    for (const auto& string_line : PWL_sources_contents)
    {
        std::string str = string_line;                       // String to analyze 
        std::string pat = "VAR";                             // Pattern to find
        if (NetlistStrings::ContainsPattern(str, pat)) 
        {
            std::string Path;
            Path = "SPICE_circuit/PWL_sources/";
            Path += string_line;
            Path += ".txt";
            NetlistStrings::Write_File(Path, NetlistStrings::Num_To_LongEng_String(0)); 
        }
    }
    // -------- Set the voltage IC waveform through the PWL --------
    std::vector<std::string> PWL_lines;
    for (const auto& string_line : NETLIST_contents)
    {
        for (const auto& string_line1 : PWL_sources_contents)
        {
            std::string str = string_line;                   // String to analyze
            std::string pat = string_line1;                  // Pattern to find
            if (NetlistStrings::ContainsPattern(str, pat))
            {
                PWL_lines.push_back(string_line);
            }
        }
    }
    // Scope_String_Vector(PWL_lines);               // DEBUG: Scope what desired for debug purposes

    // -------- Extract only the lines with the PWL generators --------
    std::vector<std::string> PWL_generators;
    for (const auto& string_line : PWL_lines)
    {
        std::string str = string_line;                   // String to analyze
        std::string pat = "PWL";                         // Pattern to find
        if (NetlistStrings::ContainsPattern(str, pat))
        {
            PWL_generators.push_back(string_line);
        }
    }
    // Scope_String_Vector(PWL_generators);               // DEBUG: Scope what desired for debug purposes

    // -------- Find where the Netlist is equal to the generators lines --------
    std::vector<bool> TF2;
    bool flag = false;
    for (const auto& string_line : NETLIST_contents)
    {
        for (const auto& string_line1 : PWL_generators)  
        {
            std::string str = string_line;                           // String to analyze
            std::string pat = string_line1;                          // Pattern to find
            if (NetlistStrings::ContainsPattern(str, pat)) 
            {
                TF2.push_back(true);
                flag = true;
            }
        }
        if (flag == false)
        {
            TF2.push_back(false);
        }
        flag = false;
    }
    // cout << TF2.size();                                    // DEBUG: Scope some needed results
        
    // -------- Initialize the PWL sources --------
    std::vector<std::string> PWL_generators_new; 
    for (const auto& string_line : PWL_generators) 
    {
        std::string new_string_line = NetlistStrings::Extract_Before_Pattern(string_line, "PWL");  
        PWL_generators_new.push_back(new_string_line);  
    }
    // Scope_String_Vector(PWL_generators_new);               // DEBUG: Scope what desired for debug purposes

    std::vector<std::string> PWL_generators_new1;  
    for (const auto& string_line : PWL_generators_new)  
    {
        std::string new_string_line1 = string_line;  
        new_string_line1 += "PWL(0 0 1e-9 0)";  
        PWL_generators_new1.push_back(new_string_line1);  
    }
    // Scope_String_Vector(PWL_generators_new1);               // DEBUG: Scope what desired for debug purposes

    int ii = 0;
    int kk = 0;
    for (const auto& elem : TF2)
    {
        if (elem)
        {
            NETLIST_contents[ii] = PWL_generators_new1[kk];
            kk++;
        }
        ii++;
    }
    // cout << NETLIST_contents[1];                            // DEBUG: Scope some needed results
}

// ======== Method: allows to initialize the inductances present in the circuit ========
// No class variables used.
void ChElectronicsCosimulation::Inductances_Initializer(std::vector<std::string>& NETLIST_contents)   // To fix IC inductances current problematics during co-simulation
{
    // -------- Isolate the lines with Inductances --------
    std::vector<std::string> Inductances1;
    std::vector<std::string> str = NetlistStrings::First_Letter_Extractor(NETLIST_contents);
    std::vector<bool> TF;
    int ii = 0;  
    for (const auto& string_line : str) 
    {
        std::string str = string_line;                       // String to analyze
        std::string pat = "L";                               // Pattern to find
        if (NetlistStrings::ContainsPattern(str, pat)) 
        {
            Inductances1.push_back(NETLIST_contents[ii]);
            TF.push_back(true);
        }
        else
        {
            TF.push_back(false);
        }
        ii++; 
    }
    // Scope_String_Vector(Inductances1);               // DEBUG: Scope what desired for debug purposes

    // -------- Isolate the names of the Inductances --------
    std::vector<std::string> Inductances;
    for (const auto& string_line : Inductances1)
    {
        std::string new_string_line = NetlistStrings::Extract_Before_Pattern(string_line, " ");
        Inductances.push_back(new_string_line);
    }
    // Scope_String_Vector(Inductances);               // DEBUG: Scope what desired for debug purposes

    // -------- Store these obtained inductances list into a variable --------
    std::vector<std::string> Inductances1_original = Inductances;

    // -------- Sort into ascending order the inductances list --------
    auto result = NetlistStrings::Sort_With_Indices(Inductances);
    Inductances = result.first;
    std::vector<size_t> Idx_sort = result.second;
    // Scope_String_Vector(Inductances);                // DEBUG: Scope what desired for debug purposes
    // Scope_size_t_Vector(Idx_sort);                   // DEBUG: Scope what desired for debug purposes

    // -------- Check if are any duplicated inductances --------
    std::vector<std::string> InductancesTest;
    std::string str_new;
    std::unordered_set<std::string> seen; 
    int kk = 1;
    for (const auto& str : Inductances)
    {
        str_new = str; 
        if (seen.find(str) != seen.end())
        {
            str_new += "n";
            str_new += std::to_string(kk);

        }
        seen.insert(str);
        InductancesTest.push_back(str_new);
        kk++;
    }
    // Scope_String_Vector(InductancesTest);               // DEBUG: Scope what desired for debug purposes
        
    // -------- Reorder according to the netlist the inductances --------
    std::vector<std::string> Inductances_new;
    for (const auto& ii : Idx_sort)
    {
        Inductances_new.push_back(InductancesTest[ii]);
    }
    InductancesTest = Inductances_new;
    // Scope_String_Vector(Inductances_new);               // DEBUG: Scope what desired for debug purposes

    // -------- Erase the initial conditions from inductances lines --------
    std::vector<std::string> Inductances2;
    for (const auto& string_line : Inductances1)
    {
        std::string new_string_line = NetlistStrings::Extract_Before_Pattern(string_line, " ic{ic");
        Inductances2.push_back(new_string_line);
    }
    // Scope_String_Vector(Inductances2);               // DEBUG: Scope what desired for debug purposes

    // -------- Recreate the inductance lines with the enumerate inductances --------
    std::vector<std::string> strline = Inductances2;
    std::vector<std::string> oldline = Inductances1_original;
    std::vector<std::string> newline = InductancesTest;
    std::vector<std::string> Inductances_lines1;  
    ii = 0;
    for (const auto& string_line : strline)
    {
        std::string Inductances_lines1_temp = NetlistStrings::Replace_Substring(string_line, oldline[ii], newline[ii]);
        Inductances_lines1.push_back(Inductances_lines1_temp);
        ii++;
    }
    // Scope_String_Vector(Inductances_lines1);               // DEBUG: Scope what desired for debug purposes

    // -------- Assign the Initial Conditions --------
    std::vector<std::string> Inductances3;
    ii = 0;
    for (const auto& string_line : InductancesTest)
    {
        std::string pat = " ic{ic";
        pat += string_line;
        pat += "}";
        std::string Inductances_lines1_temp = Inductances_lines1[ii];
        Inductances3.push_back(Inductances_lines1_temp += pat);
        ii++;
    }
    // Scope_String_Vector(Inductances3);               // DEBUG: Scope what desired for debug purposes

    // -------- Modify the NETLIST_contents variable with the fixed inductances --------
    ii = 0;
    kk = 0;
    for (const auto& elem : TF)
    {
        if (elem)
        {
            NETLIST_contents[ii] = Inductances3[kk];
            kk++;
        }
        ii++;
    }
        // Here it's not necessary to return the NETLIST_contents variable because it's called inside the method with &

    // -------- Set the inductance current initial conditions IC: I(L)=0 --------
    Inductances = InductancesTest;
    Inductances1.clear();
    for (const auto& string_line : Inductances)
    {
        std::string Inductances1_temp = ".param ic";
        Inductances1_temp += string_line;
        Inductances1_temp += "=" + NetlistStrings::Num_To_LongEng_String(0); 
        Inductances1.push_back(Inductances1_temp);
    }
    // Scope_String_Vector(Inductances1);               // DEBUG: Scope what desired for debug purposes

        // (Per Fede) da sistemare nel caso in cui nella NETLIST non abbia neche una riga con ".param ic..."
    std::vector<size_t> indexes;
    ii = 0;
    bool flag = true; 
    for (const auto& string_line : NETLIST_contents) 
    {
        std::string str = string_line;                           // String to analyze
        std::string pat = ".param ic";                           // Pattern to find
        if (NetlistStrings::ContainsPattern(str, pat)) 
        {
            indexes.push_back(ii);
        }
        ii++;
    }
    // cout << indexes.front() << "\n";                 // DEBUG: Scope some needed results
    // cout << indexes.back() << "\n";                  // DEBUG: Scope some needed results
        
    // -------- Erase the elemnts from the Netlist vector --------
    NETLIST_contents.erase(NETLIST_contents.begin() + indexes.front(), NETLIST_contents.begin() + indexes.back()+1); // +1 is neaded because the index is different respect to the number of elemnts of an array
    // cout << NETLIST_contents[62] << "\n";            // DEBUG: Scope some needed results
    // cout << NETLIST_contents[63] << "\n";            // DEBUG: Scope some needed results
    size_t idx = indexes.front(); 
    for (const auto& string_line : Inductances1) 
    {
        auto position = NETLIST_contents.begin() + idx;  // Insert after the idx^th element
        NETLIST_contents.insert(position, string_line);
        idx++;
    }
    // cout << NETLIST_contents[62] << "\n";            // DEBUG: Scope some needed results
    // cout << NETLIST_contents[74] << "\n";            // DEBUG: Scope some needed results
}    

// ======== Method: allows to initialize the NETLIST file for co-simulation ========
// Class variables used:
// parser_input, parser_output
// PWL_sources_NETLIST_reordered
// PWL_sources_NETLIST_idx
// node_name
// sim_time_last
// Python_simulator_name
void ChElectronicsCosimulation::NETLIST_Initializer()
{

    auto node_name = this->results.node_name;

    // -------- Read the main circuit initialization files --------
    parser_output = parser.ParseCircuit(parser_input);

    if (parser_output.NETLIST_contents.empty()) // Alert the user if no circuit model is found inside the NETLIST
    {
        std::cerr << "WARNING!! -> No circuit model found inside the NETLIST" << std::endl;
        system("pause>0");
        return;
    }

    // -------- Initialize the PWL_sources and Inductances in the NETLIST -------- 
    PWL_sources_Initializer(parser_output.PWL_sources_contents, parser_output.NETLIST_contents);
    Inductances_Initializer(parser_output.NETLIST_contents);

    // -------- Update the NETLIST with the resulting PWL_sources and Inductances --------
    NetlistStrings::Write_File(parser_input.NETLIST_name, "");
    for (const auto& string_line : parser_output.NETLIST_contents)
    {
        NetlistStrings::Append_File(parser_input.NETLIST_name, string_line + "\n");
    }

    // -------- Run a preliminary simulation to initialize the parameters --------
    std::string File_name = Python_simulator_name;
    std::string Method_name = "CircuitAnalysis";
    double t_step = 1e-8; // Max allowed: t_step = 1e-8
    double t_end = 1e-6;

    Run_Spice(File_name, Method_name, t_step, t_end);

    // -------- Re-Read the NETLIST --------
    parser.RefreshNetlist(parser_input, parser_output);

    // -------- Set parameters to the initial conditions --------
    int index = 0;
    int param_idx = 0;
    for (auto& string_line : parser_output.NETLIST_contents)
    {
        std::string pat = ".param par";
        if (NetlistStrings::ContainsPattern(string_line, pat))
        {
            std::string Parameters1 = NetlistStrings::Extract_Before_Pattern(string_line, "=");
            std::string Parameters2 = Parameters1 + "=" + parser_output.Param_IC_contents[param_idx++];
            parser_output.NETLIST_contents[index] = Parameters2;
        }
        ++index;
    }

    // -------- Set the voltage initial conditions to 0 --------
    std::string V_IC_array = ".ic ";
    for (size_t i = 0; i < node_name.size(); ++i)
    {
        V_IC_array += "V(" + node_name[i] + ")=" + NetlistStrings::Num_To_LongEng_String(0);
        if (i < node_name.size() - 1) V_IC_array += " ";
    }

    index = 0;
    for (auto& string_line : parser_output.NETLIST_contents)
    {
        std::string pat = ".ic";
        if (NetlistStrings::ContainsPattern(string_line, pat))
        {
            parser_output.NETLIST_contents[index] = V_IC_array;
        }
        ++index;
    }

    // -------- Update the NETLIST with new initial conditions --------
    NetlistStrings::Write_File(parser_input.NETLIST_name, "");
    for (const auto& string_line : parser_output.NETLIST_contents)
    {
        NetlistStrings::Append_File(parser_input.NETLIST_name, string_line + "\n");
    }

    // -------- Identify and reorder PWL sources in the NETLIST --------
    for (const auto& string_line : parser_output.NETLIST_contents)
    {
        std::string pat = "PWL";
        if (NetlistStrings::ContainsPattern(string_line, pat))
        {
            for (const auto& pwl_source : parser_output.PWL_sources_contents)
            {
                if (NetlistStrings::ContainsPattern(string_line, pwl_source))
                {
                    PWL_sources_NETLIST_reordered.push_back(pwl_source);
                }
            }
            PWL_sources_NETLIST_idx.push_back(1);
        }
        else
        {
            PWL_sources_NETLIST_idx.push_back(0);
        }
    }

    // -------- Initialize the simulation time --------
    sim_time_last = 0;
}

// ======== Method: allows to initialize the NETLIST file and to extract the SPICE simulation results at every time step of the electronic call ========
// Class variables used:


// double t_clock;                       // Represents the current clock time for the simulation
// double t_step_electronic;             // Time step for the electronic simulation
// double T_sampling_electronic;         // Sampling time for the electronic simulation

// ParserOutput
// ParserInput

// CosimReult;   // Names of the nodes

// std::vector<std::string> PWL_sources_NETLIST_reordered;   // Reordered PWL sources in the NETLIST

// double sim_time_last;                 // Last simulation time step

// int sim_step;                         // Simulation step count
// std::vector<double> OUTPUT_value;     // Output values after parsing and comparison



void ChElectronicsCosimulation::NETLIST_Cosimulator(std::vector<std::vector<double>>& INPUT_values, double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var) 
{

    auto node_val = this->results.node_val;
    auto node_name = this->results.node_name;
    auto branch_val = this->results.branch_val;
    auto branch_name = this->results.branch_name;
    auto sim_time = this->results.sim_time;

    // -------- Read the main circuit initialization files --------
   auto NETLIST_contents = parser_output.NETLIST_contents;
    if (NETLIST_contents.empty())  // Alert the user if no circuit model is found inside the NETLIST
    {
        std::cerr << "WARNING!! -> No circuit model found inside the NETLIST" << std::endl;
        return;
    }

    // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var; 
    t_step_electronic = t_step_electronic_var; 
    T_sampling_electronic = T_sampling_electronic_var; 

    // -------- Set the PWL sources --------
    std::vector<std::vector<double>> t_interp_PWL_Source;
    std::vector<std::vector<double>> Source_Signal_PWL_Source;

    // Iterate over the PWLsources_reordered, it's important that we use the PWLsources_reordered so that the NETLIST will be updated correctly
    for (const auto& string_line : PWL_sources_NETLIST_reordered)
    {
        std::vector<double> t_interp; 
        std::vector<double> t_interp1;

        for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic) 
        {
            t_interp.push_back(t); 
            t_interp1.push_back(t - t_clock);
        }

        std::string pat = "VAR";
        if (NetlistStrings::ContainsPattern(string_line, pat))  // Processing a variable PWL source
        {
            std::vector<std::string> Source_Signal_1;
            int jj = 0; 
            for (const auto& string_line1 : parser_output.INPUT_contents)
            {
                if (NetlistStrings::ContainsPattern(string_line1, string_line))  // Variable PWL source found
                {
                    if (INPUT_values[jj].size() == 1) 
                    {
                        // Load previous time-step PWL sources value
                        std::string path = "SPICE_circuit/PWL_sources/" + string_line + ".txt"; 
                        std::vector<std::string> V_source = NetlistStrings::Read_File(path);

                        // Create a vector that stores all the PWL information
                        Source_Signal_1.push_back(V_source[0]); 
                        Source_Signal_1.push_back(NetlistStrings::Num_To_LongEng_String(INPUT_values[jj][0]));

                        std::vector<double> Source_Signal = NetlistStrings::Linspace(stod(Source_Signal_1[0]), stod(Source_Signal_1[1]), t_interp.size());
                        Source_Signal_PWL_Source.push_back(Source_Signal);

                        // Update the source value in the PWL_sources folder
                        std::string Path = "SPICE_circuit/PWL_sources/" + string_line + ".txt";
                        NetlistStrings::Write_File(Path, Source_Signal_1[1]);
                    }
                    else 
                    {
                        std::vector<double> Source_Signal = INPUT_values[jj];
                        Source_Signal_PWL_Source.push_back(Source_Signal);

                        // Update the source value in the PWL_sources folder
                        std::string Path = "SPICE_circuit/PWL_sources/" + string_line + ".txt";
                        NetlistStrings::Write_File(Path, std::to_string(Source_Signal.back()));
                    }
                }
                jj++;
            }
        }
        t_interp_PWL_Source.push_back(t_interp1);
    }


    if (!parser_output.PWL_sources_contents.empty())
    {
        std::vector<std::string> PWL_command_line;
        for (int jj = 0; jj < Source_Signal_PWL_Source.size(); ++jj)
        {
            std::vector<double> t_interp_PWL_Source_vector = t_interp_PWL_Source[jj];
            std::vector<double> Source_Signal_PWL_Source_vector = Source_Signal_PWL_Source[jj];
            std::string PWL_command_line_string = "(";

            for (int ii = 0; ii < t_interp_PWL_Source_vector.size(); ++ii) {
                PWL_command_line_string += NetlistStrings::Num_To_LongEng_String(t_interp_PWL_Source_vector[ii]) + " ";
                PWL_command_line_string += NetlistStrings::Num_To_LongEng_String(Source_Signal_PWL_Source_vector[ii]);
                if (ii < t_interp_PWL_Source_vector.size() - 1)
                    PWL_command_line_string += " ";
            }
            PWL_command_line_string += ")";
            PWL_command_line.push_back(PWL_command_line_string);
        }

        int ii = 0;
        int jj = 0;
        for (auto& string_line : NETLIST_contents)
        {
            if (NetlistStrings::ContainsPattern(string_line, "PWL("))
            {
                std::string PWL_sources_new = NetlistStrings::Extract_Before_Pattern(string_line, "PWL(") + "PWL" + PWL_command_line[ii];
                NETLIST_contents[jj] = PWL_sources_new;
                ii++; 
            }
            jj++;
        }
    }    

    // -------- Set the inductances in the NETLIST --------
    std::vector<std::string> Inductances1;
    std::vector<std::string> str = NetlistStrings::First_Letter_Extractor(NETLIST_contents);
    std::vector<bool> TF;
    int ii = 0; 

    for (const auto& string_line : str) 
    {
        if (NetlistStrings::ContainsPattern(string_line, "L")) 
        {
            Inductances1.push_back(NETLIST_contents[ii]); 
            TF.push_back(true);
        }
        else
        {
            TF.push_back(false);
        }
        ii++;
    }

    std::vector<std::string> Inductances;
    for (const auto& string_line : Inductances1)
    {
        std::string new_string_line = NetlistStrings::Extract_Before_Pattern(string_line, " ");
        Inductances.push_back(new_string_line);
    }

    std::vector<std::string> res_name;
    res_name = node_name;
    res_name.insert(res_name.end(), this->results.branch_name.begin(), this->results.branch_name.end()); 
    std::vector<std::vector<double>> res_val = NetlistStrings::Concatenate_2_Vectors(node_val, branch_val);

    // -------- Set the inductance current initial conditions IC: I(L) --------
    Inductances1.clear();
    std::vector<std::string> ToFind = NetlistStrings::To_Lower_Case(Inductances);
    int jj = 0;
    for (const auto& string_line : ToFind)
    {
        ii = 0;
        double IL_value_IC;
        for (const auto& string_line1 : this->results.branch_name) 
        {
            if (NetlistStrings::ContainsPattern(string_line1, string_line))
            {
                std::vector<double> IL_value_IC1 = this->results.branch_val[ii];
                IL_value_IC = IL_value_IC1.back(); 
            }
            ii++;
        }
        std::string new_string_line = ".param ic" + Inductances[jj] + "=" + NetlistStrings::Num_To_LongEng_String(IL_value_IC); 
        Inductances1.push_back(new_string_line); 
        jj++;
    }

    // -------- Reinitialize the ".param ic" rows in the NETLIST --------
    ii = 0;
    jj = 0; 
    for (auto& string_line : NETLIST_contents)
    {
        if (NetlistStrings::ContainsPattern(string_line, ".param ic"))
        {
            NETLIST_contents[ii] = Inductances1[jj];
            jj++;
        }
        ii++;
    }
    
        // -------- Set the voltage initial conditions IC: V --------
    std::string V_IC_array = ".ic ";
    ii = 1;
    jj = 0; 
    for (const auto& string_line : node_name) 
    {
        V_IC_array += "V(" + string_line + ")=";
        std::vector<double> node_val_IC = node_val[jj];
        V_IC_array += NetlistStrings::Num_To_LongEng_String(node_val_IC.back()); 
        if (ii < node_name.size())  
        {
            V_IC_array += " "; 
        }
        ii++;
        jj++;
    }

    // -------- Introduce the array in the netlist --------
    ii = 0;  
    for (auto& string_line : NETLIST_contents)
    {
        if (NetlistStrings::ContainsPattern(string_line, ".ic"))
        {
            NETLIST_contents[ii] = V_IC_array;
        }
        ii++;
    }

    // -------- Set the parameters --------
    std::vector<std::string> Parameters1;
    jj = 0; 
    for (const auto& string_line : parser_output.INPUT_contents) 
    {
        std::string Parameters1_line = ".param par" + string_line + "=" + NetlistStrings::Num_To_LongEng_String(INPUT_values[jj][0]);
        Parameters1.push_back(Parameters1_line);
        jj++; 
    }

    // -------- Introduce the array in the netlist --------
    ii = 0;
    jj = 0;
    for (auto& string_line : parser_output.NETLIST_contents)
    {
        if (NetlistStrings::ContainsPattern(string_line, ".param par"))
        {
            NETLIST_contents[ii] = Parameters1[jj]; 
            jj++; 
        }
        ii++; 
    }
    
    // -------- Update the NETLIST --------
    NetlistStrings::Write_File(parser_input.NETLIST_name, ""); 
    ii = 1;
    for (const auto& string_line : parser_output.NETLIST_contents)    // The NETLIST_contents variable was previously updated by the above two functions 
    {
        NetlistStrings::Append_File(parser_input.NETLIST_name, string_line); 
        if (ii < NETLIST_contents.size())
        {
            NetlistStrings::Append_File(parser_input.NETLIST_name, "\n");
        }
        ii++;
    }

    // -------- OUTPUT_extraction --------
    OUTPUT_value.clear();  
    for (const auto& string_line : NetlistStrings::To_Lower_Case(parser_output.OUTPUT_contents))  
    {
        ii = 0;
        for (const auto& string_line1 : res_name)  
        { 
            std::string str = string_line;                               // String to analyze    
            std::string pat = string_line1;                              // Pattern to find   
            if (NetlistStrings::StrCompare(str, pat))
            { 
                OUTPUT_value.push_back(res_val[ii].back()); 
                //std::cout << "str:" << str << "\n\n";                // DEBUG: Scope what desired for debug purposes 
                //std::cout << "pat:" << pat << "\n\n";                // DEBUG: Scope what desired for debug purposes 
            }
            ii++; 
        }   
    }

    // std::cout << OUTPUT_value.size() << "\n";                // DEBUG: Scope what desired for debug purposes 
    // Scope_String_Vector(OUTPUT_contents);                    // DEBUG: Scope what desired for debug purposes 

    // -------- Update the sim_step_last variable --------
    sim_time_last = sim_time.back(); 
    sim_step++;
}  
