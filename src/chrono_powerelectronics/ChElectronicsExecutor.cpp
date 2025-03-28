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
#include "ChElectronicsExecutor.h"

// =============================
// ======== Constructor ========
// =============================
ChElectronicsExecutor::ChElectronicsExecutor(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var)
    : ChElectronicsManipulator(NETLIST_name_var, INPUT_name_var, OUTPUT_name_var, PWL_sources_name_var, Param_IC_name_var, Python_simulator_name_var) {}

// ==========================================
// ======== Methods: SPICE execution ========
// ==========================================
    // ======== Method: allows to set the class t_clock global variable ========
void ChElectronicsExecutor::Set_t_clock_var(double t_clock_var) 
{
    t_clock = t_clock_var; 
}

    // ======== Method: allows to read a NETLIST circuit ========
std::vector<std::string> ChElectronicsExecutor::Read_NETLIST(std::string File_name)
{
    std::vector<std::string> File_contents = Read_File(File_name);
    return File_contents;
}

    // ======== Method: allows to read the INPUT list file ========
std::vector<std::string> ChElectronicsExecutor::Read_INPUT_Parameters(std::string File_name)
{
    std::vector<std::string> File_contents = Read_File(File_name);
    return File_contents;
}

    // ======== Method: allows to read the OUTPUT list file ========
std::vector<std::string> ChElectronicsExecutor::Read_OUTPUT_Parameters(std::string File_name)
{
    std::vector<std::string> File_contents = Read_File(File_name);
    return File_contents;
}

    // ======== Method: allows to read the Piece Wise Linear sowrces list file ========
std::vector<std::string> ChElectronicsExecutor::Read_PWL_sources(std::string File_name)
{
    std::vector<std::string> File_contents = Read_File(File_name);
    return File_contents;
}

    // ======== Method: allows to read the Parameters Initial Conditions values list file ========
std::vector<std::string> ChElectronicsExecutor::Read_Param_IC(std::string File_name)
{
    std::vector<std::string> File_contents = Read_File(File_name);
    return File_contents;
}

// ======== Method: allows to run a Spice Netlist simulation and solve the circuit ========
void ChElectronicsExecutor::Run_Spice(std::string File_name, std::string Method_name, double t_step, double t_end)
{
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
}

    // ======== Method: allows to initialize the PWL sources present in the circuit ========
void ChElectronicsExecutor::PWL_sources_Initializer(std::vector<std::string>& PWL_sources_contents, std::vector<std::string>& NETLIST_contents) // To fix generator co-simulation dynamics problematics
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
        if (ContainsPattern(str, pat)) 
        {
            std::string Path;
            Path = "SPICE_circuit/PWL_sources/";
            Path += string_line;
            Path += ".txt";
            Write_File(Path, Num_To_LongEng_String(0)); 
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
            if (ContainsPattern(str, pat))
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
        if (ContainsPattern(str, pat))
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
            if (ContainsPattern(str, pat)) 
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
        std::string new_string_line = Extract_Before_Pattern(string_line, "PWL");  
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
void ChElectronicsExecutor::Inductances_Initializer(std::vector<std::string>& NETLIST_contents)   // To fix IC inductances current problematics during co-simulation
{
    // -------- Isolate the lines with Inductances --------
    std::vector<std::string> Inductances1;
    std::vector<std::string> str = First_Letter_Extractor(NETLIST_contents);
    std::vector<bool> TF;
    int ii = 0;  
    for (const auto& string_line : str) 
    {
        std::string str = string_line;                       // String to analyze
        std::string pat = "L";                               // Pattern to find
        if (ContainsPattern(str, pat)) 
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
        std::string new_string_line = Extract_Before_Pattern(string_line, " ");
        Inductances.push_back(new_string_line);
    }
    // Scope_String_Vector(Inductances);               // DEBUG: Scope what desired for debug purposes

    // -------- Store these obtained inductances list into a variable --------
    std::vector<std::string> Inductances1_original = Inductances;

    // -------- Sort into ascending order the inductances list --------
    auto result = Sort_With_Indices(Inductances);
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
        std::string new_string_line = Extract_Before_Pattern(string_line, " ic{ic");
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
        std::string Inductances_lines1_temp = Replace_Substring(string_line, oldline[ii], newline[ii]);
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
        Inductances1_temp += "=" + Num_To_LongEng_String(0); 
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
        if (ContainsPattern(str, pat)) 
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
void ChElectronicsExecutor::NETLIST_Initializer()
{
    // -------- Read the main circuit initialization files --------
    std::vector<std::string> NETLIST_contents = Read_NETLIST(NETLIST_name);
    if (NETLIST_contents.size() == 0)                   // Allert the user that no circuit model is found inside the NETLIST
    {
        std::cerr << "WARNING!! -> No circuit model found inside the NETLIST" << std::endl;
        system("pause>0");
    }
    INPUT_contents = Read_INPUT_Parameters(INPUT_name); 
    OUTPUT_contents = Read_OUTPUT_Parameters(OUTPUT_name); 
    PWL_sources_contents = Read_PWL_sources(PWL_sources_name);  
    Param_IC_contents = Read_Param_IC(Param_IC_name); 
    
    // -------- Initialize the PWL_sources in the NETLIST -------- 
    PWL_sources_Initializer(PWL_sources_contents, NETLIST_contents);
    // Scope_String_Vector(NETLIST_contents);               // DEBUG: Scope what desired for debug purposes
        
    // -------- Initialize the Inductances in the NETLIST --------
    Inductances_Initializer(NETLIST_contents);
    // Scope_String_Vector(NETLIST_contents);               // DEBUG: Scope what desired for debug purposes
        
    // -------- Update the NETLIST with the resulting PWL_sources and Inductances --------
    Write_File(NETLIST_name, ""); 
    int ii = 1;
    for (const auto& string_line : NETLIST_contents)        // The NETLIST_contents variable was previously updated by the above two functions
    {
        Append_File(NETLIST_name, string_line); 
        if (ii < NETLIST_contents.size())
        {
            Append_File(NETLIST_name, "\n"); 
        }
        ii++;
    }
    // -------- Run a preliminary simulation to initialize the parameters --------
    std::string File_name = Python_simulator_name;
    std::string Method_name = "CircuitAnalysis";
    double t_step = 1e-8; // Max allowed : t_step = 1e-8;
    double t_end = 1e-6;
    Run_Spice(File_name, Method_name, t_step, t_end);
    
    // -------- Read the NETLIST --------
    NETLIST_contents.clear();
    NETLIST_contents = Read_NETLIST(NETLIST_name);

    // -------- Get the list of all the folders inside the Spice_circuit/Results folder --------
    std::string currentDirectory = fs::current_path().string();
    std::string WorkingDirectory = currentDirectory + "/" + "Spice_circuit" + "/" + "Results";
    std::vector<std::string> Folder_list = List_Folders_In_Directory(WorkingDirectory);
    // Scope_String_Vector(Folder_list);               // DEBUG: Scope what desired for debug purposes

    // -------- Create the folder needed to store the results at every time step --------
    if (Folder_list.size() == 0)
    {
        // Create the new results folder: node_val_folder
        try {
            fs::create_directory(WorkingDirectory + "/" + "node_val_folder");       // Create a new folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        // Create the new results folder: branch_val_folder
        try {
            fs::create_directory(WorkingDirectory + "/" + "branch_val_folder");     // Create a new folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        // Create the new results folder: sim_time_folder
        try {
            fs::create_directory(WorkingDirectory + "/" + "sim_time_folder");       // Create a new folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
    else
    {
        // node_val_folder
            // Find all the node_val_folder among the found folders
        std::vector<std::string> node_val_folder;
        for (const auto& string_line : Folder_list)
        {
            std::string str = string_line;                                // String to analyze
            std::string pat = "node_val_folder";                          // Pattern to find
            if (ContainsPattern(str, pat))
            {
                node_val_folder.push_back(string_line);
            }
        }
        // Scope_String_Vector(node_val_folder);               // DEBUG: Scope what desired for debug purposes

            // Find the major folder among the node_val_folder
        std::vector<int> node_val_folder_idx;
        for (const auto& string_line : node_val_folder)
        {
            std::string node_val_folder_idx1 = Replace_Substring(string_line, "node_val_folder", "");
            if (node_val_folder_idx1 == "")
            {
                node_val_folder_idx1 = "0";
            }
            int node_val_folder_idx2 = stoi(node_val_folder_idx1);
            node_val_folder_idx.push_back(node_val_folder_idx2);
        }
        // Scope_String_Vector(node_val_folder_idx);               // DEBUG: Scope what desired for debug purposes

        auto node_val_folder_max_ptr = max_element(node_val_folder_idx.begin(), node_val_folder_idx.end()); // FInd the folder with the maximum index
        int node_val_folder_max = *node_val_folder_max_ptr;
        std::string node_val_folder_new = "node_val_folder" + std::to_string(node_val_folder_max + 1);
        // cout << node_val_folder_new << "\n";                     // DEBUG: Scope some needed results
            
            // Rename the last results folder
        try {
            fs::rename(WorkingDirectory + "/" + "node_val_folder", WorkingDirectory + "/" + node_val_folder_new); // Rename the folder
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

            // Create the new results folder
        try {
            fs::create_directory(WorkingDirectory + "/" + "node_val_folder");       // Create a new folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        // branch_val_folder
            // Find all the branch_val_folder among the found folders
        std::vector<std::string> branch_val_folder;
        for (const auto& string_line : Folder_list)
        {
            std::string str = string_line;                                   // String to analyze
            std::string pat = "branch_val_folder";                           // Pattern to find
            if (ContainsPattern(str, pat))
            {
                branch_val_folder.push_back(string_line);
            }
        }
        // Scope_String_Vector(branch_val_folder);               // DEBUG: Scope what desired for debug purposes

            // Find the major folder among the branch_val_folder
        std::vector<int> branch_val_folder_idx;
        for (const auto& string_line : branch_val_folder)
        {
            std::string branch_val_folder_idx1 = Replace_Substring(string_line, "branch_val_folder", "");
            if (branch_val_folder_idx1 == "")
            {
                branch_val_folder_idx1 = "0";
            }
            int branch_val_folder_idx2 = stoi(branch_val_folder_idx1);
            branch_val_folder_idx.push_back(branch_val_folder_idx2);
        }
        // Scope_String_Vector(branch_val_folder_idx);               // DEBUG: Scope what desired for debug purposes
        
        auto branch_val_folder_max_ptr = max_element(branch_val_folder_idx.begin(), branch_val_folder_idx.end()); // FInd the folder with the maximum index
        int branch_val_folder_max = *branch_val_folder_max_ptr;
        std::string branch_val_folder_new = "branch_val_folder" + std::to_string(branch_val_folder_max + 1); 
        // cout << branch_val_folder_new << "\n";                   // DEBUG: Scope some needed results
             
            // Rename the last results folder
        try {
            fs::rename(WorkingDirectory + "/" + "branch_val_folder", WorkingDirectory + "/" + branch_val_folder_new);       // Rename the folder
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

            // Create the new results folder
        try {
            fs::create_directory(WorkingDirectory + "/" + "branch_val_folder");                                             // Create a new folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        // sim_time_folder
            // Find all the sim_time_folder among the found folders
        std::vector<std::string> sim_time_folder;
        for (const auto& string_line : Folder_list)
        {
            std::string str = string_line;                                // String to analyze
            std::string pat = "sim_time_folder";                          // Pattern to find
            if (ContainsPattern(str, pat))
            {
                sim_time_folder.push_back(string_line);
            }
        }
        // Scope_String_Vector(sim_time_folder);               // DEBUG: Scope what desired for debug purposes

            // Find the major folder among the sim_time_folder
        std::vector<int> sim_time_folder_idx;
        for (const auto& string_line : sim_time_folder)
        {
            std::string sim_time_folder_idx1 = Replace_Substring(string_line, "sim_time_folder", "");
            if (sim_time_folder_idx1 == "")
            {
                sim_time_folder_idx1 = "0";
            }
            int sim_time_folder_idx2 = stoi(sim_time_folder_idx1);
            sim_time_folder_idx.push_back(sim_time_folder_idx2);
        }
        // Scope_String_Vector(sim_time_folder_idx);               // DEBUG: Scope what desired for debug purposes

        auto sim_time_folder_max_ptr = max_element(sim_time_folder_idx.begin(), sim_time_folder_idx.end()); // FInd the folder with the maximum index
        int sim_time_folder_max = *sim_time_folder_max_ptr;
        std::string sim_time_folder_new = "sim_time_folder" + std::to_string(sim_time_folder_max + 1); 
        // cout << sim_time_folder_new << "\n";                     // DEBUG: Scope some needed results
        
            // Rename the last results folder
        try {
            fs::rename(WorkingDirectory + "/" + "sim_time_folder", WorkingDirectory + "/" + sim_time_folder_new); // Rename the folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }

            // Create the new results folder
        try {
            fs::create_directory(WorkingDirectory + "/" + "sim_time_folder"); // Create a new folder 
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    // -------- Sort the current and voltage results --------
        // Voltage nodes
    auto result = Sort_With_Indices(node_name);
    std::vector<std::string> node_name_Vec = result.first;
    std::vector<std::vector<double>> node_val_Vec;
    std::vector<std::vector<double>> node_val_Vec_new;

        // Current branches
    auto result1 = Sort_With_Indices(branch_name);
    std::vector<std::string> branch_name_Vec = result1.first;
    std::vector<std::vector<double>> branch_val_Vec;
    std::vector<std::vector<double>> branch_val_Vec_new;

        // Simulation time
    std::vector<double> sim_time_Vec;
    sim_time_Vec.push_back(0);
    std::vector<double> sim_time_Vec_new;
    sim_time_Vec_new.push_back(0);
    
    // -------- Save the results in the created folders --------
    Save_Vector_Vector_double_To_Txt(WorkingDirectory + "/" + "node_val_folder" + "/" + "node_val_Vec1.txt", node_val_Vec_new);
    Save_Vector_Vector_double_To_Txt(WorkingDirectory + "/" + "branch_val_folder" + "/" + "branch_val_Vec1.txt", branch_val_Vec_new);
    Save_Vector_double_To_Txt(WorkingDirectory + "/" + "sim_time_folder" + "/" + "sim_time_Vec1.txt", sim_time_Vec_new);

    // -------- Save the reordered file branch and node name variables --------
        // Update the node_name_Vec
    Write_File("SPICE_circuit/Results/node_val_folder/node_name_Vec.txt", "");
    ii = 1; 
    for (const auto& string_line : node_name_Vec) 
    {
        Append_File("SPICE_circuit/Results/node_val_folder/node_name_Vec.txt", string_line);
        if (ii < node_name_Vec.size())
        {
            Append_File("SPICE_circuit/Results/node_val_folder/node_name_Vec.txt", "\n");
        }
        ii++; 
    }
    
        // Update the branch_name_Vec
    Write_File("SPICE_circuit/Results/branch_val_folder/branch_name_Vec.txt", "");
    ii = 1;
    for (const auto& string_line : branch_name_Vec)
    {
        Append_File("SPICE_circuit/Results/branch_val_folder/branch_name_Vec.txt", string_line);
        if (ii < branch_name_Vec.size())
        {
            Append_File("SPICE_circuit/Results/branch_val_folder/branch_name_Vec.txt", "\n");
        }
        ii++;
    }    
    
    // -------- Set the parameters to the IC values --------
    ii = 0;
    int jj = 0;
    for (const auto& string_line : NETLIST_contents)
    {
        std::string str = string_line;                                               // String to analyze
        std::string pat = ".param par";                                              // Pattern to find
        if (ContainsPattern(str, pat))
        {
            std::string Parameters1 = Extract_Before_Pattern(string_line, "=");
            std::string Parameters2 = Parameters1 + "=" + Param_IC_contents[jj];     // Remember to implement the Par_IC_contents file with the order of the .param par in the NETLIST
            NETLIST_contents[ii] = Parameters2;
            jj++;
        }
        ii++;
    }
    // -------- Set the voltage initial conditions to 0 IC: V --------
    std::string V_IC_array = ".ic ";
    ii = 1; 
    for (const auto& string_line : node_name)
    {
        V_IC_array += "V(";
        V_IC_array += string_line;
        V_IC_array += ")=" + Num_To_LongEng_String(0); 
        if (ii < node_name.size())
        {
            V_IC_array += " ";
        }
        ii++;
        
    }
    ii = 0; 
    for (const auto& string_line : NETLIST_contents)
    {
        std::string str = string_line;                       // String to analyze
        std::string pat = ".ic";                             // Pattern to find
        if (ContainsPattern(str, pat))
        {
            NETLIST_contents[ii] = V_IC_array;
        }
        ii++;
    }

    // -------- Update the NETLIST --------
    Write_File(NETLIST_name, "");
    ii = 1;
    for (const auto& string_line : NETLIST_contents)    // The NETLIST_contents variable was previously updated by the above two functions
    {
        Append_File(NETLIST_name, string_line);
        if (ii < NETLIST_contents.size())
        {
            Append_File(NETLIST_name, "\n");
        }
        ii++;
    }

    // -------- Set some important variable of the Class to speed up the simulation --------
        // Find the PWL sources lines in the NETLIST
    for (const auto& string_line : NETLIST_contents)
    {
        std::string str = string_line;                       // String to analyze
        std::string pat = "PWL";                             // Pattern to find
        if (ContainsPattern(str, pat))
        {
            for (const auto& string_line1 : PWL_sources_contents)
            {
                std::string str = string_line;               // String to analyze
                std::string pat = string_line1;              // Pattern to find
                if (ContainsPattern(str, pat))
                {
                    PWL_sources_NETLIST_reordered.push_back(string_line1);
                }
            }
            PWL_sources_NETLIST_idx.push_back(1);
        }
        else
        {
            PWL_sources_NETLIST_idx.push_back(0);
        }
    }
    // -------- Update the sim_step_last variable --------
    sim_time_last = 0;  
}

    // ======== Method: allows to initialize the NETLIST file and to extract the SPICE simulation results at every time step of the electronic call ========
void ChElectronicsExecutor::NETLIST_Cosimulator(std::vector<std::vector<double>>& INPUT_values, double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var) 
{
    // -------- Read the main circuit initialization files --------
    std::vector<std::string> NETLIST_contents = Read_NETLIST(NETLIST_name);
    if (NETLIST_contents.size() == 0)           // Allert the user that no circuit model is found inside the NETLIST
    {
        std::cerr << "WARNING!! -> No circuit model found inside the NETLIST" << std::endl;
        system("pause>0");
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
            // Define the time interval for the electronics time step
        std::vector<double> t_interp;                            // Localized onto the global time line
        std::vector<double> t_interp1;                           // Localized onto the SPICE time line
        for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic) {
            t_interp.push_back(t);                          // Add the current value of t to the vector
            t_interp1.push_back(t - t_clock);
        }

            // -------- Check if it is a VARiable or a PREdefined source --------
        std::string str = string_line;                           // String to analyze
        std::string pat = "VAR";                                 // Pattern to find
        if (ContainsPattern(str, pat))                      // We are now processing a VARiable PWL source
        {
            
            std::vector<std::string> Source_Signal_1;
            int jj = 0;                                     // Iterator over the INPUT_contents elements 
            for (const auto& string_line1 : INPUT_contents)
            {
                std::string str = string_line1;                  // String to analyze
                std::string pat = string_line;                   // Pattern to find
                if (ContainsPattern(str, pat))              // We are now processing a VARiable PWL sources
                {
                    if (INPUT_values[jj].size() == 1) // Case in which we are passing only a value of the PWl source
                    {
                        // Load the previous time-step PWL_sorces value 
                        std::string path = "SPICE_circuit/PWL_sources/" + string_line + ".txt"; 
                        std::vector<std::string> V_source = Read_File(path); 

                        // Create a vector that stores all the PWL informations 
                        Source_Signal_1.push_back(V_source[0]); 
                        Source_Signal_1.push_back(Num_To_LongEng_String(INPUT_values[jj][0])); 

                        std::vector<double> Source_Signal = Linspace(stod(Source_Signal_1[0]), stod(Source_Signal_1[1]), t_interp.size());
                        Source_Signal_PWL_Source.push_back(Source_Signal);
                        // Update the source value in the PWL_sources folder
                        std::string Path;
                        Path = "SPICE_circuit/PWL_sources/";
                        Path += string_line;
                        Path += ".txt";
                        Write_File(Path, Source_Signal_1[1]);
                    }
                    else // Case in which we are passing a complete PWL array to the PWL source
                    {
                        std::vector<double> Source_Signal = INPUT_values[jj];
                        Source_Signal_PWL_Source.push_back(Source_Signal);
                        // Update the source value in the PWL_sources folder
                        std::string Path;
                        Path = "SPICE_circuit/PWL_sources/";
                        Path += string_line;
                        Path += ".txt";
                        Write_File(Path, std::to_string(Source_Signal.back()));
                    }
                        
                }
                jj++;
            }   
        }
        else      // We are now processing a PREdefined PWL sources --> OBSOLETE
        {
            //
            //
            // Qua andrebbe da righe 98 a 108 della versione Matlab
            //
            //
        }
            // Define the time interval for the electronics time step initialized for the SPICE time line
        t_interp_PWL_Source.push_back(t_interp1);
    }
    // vector <double> test = t_interp_PWL_Source[1];               // DEBUG: Set some needed results
    // cout << test[1] << "\n";                                     // DEBUG: Scope some needed results
    // std::vector <double> test = Source_Signal_PWL_Source[1];     // DEBUG: Set some needed results 
    // std::cout << "IL valore e'" << test[10] << "\n";             // DEBUG: Scope some needed results 
    
    if (PWL_sources_contents.size() > 0)
    {
            // Continue the PWL sources settings
        std::vector<std::string> PWL_command_line;
        for (int jj = 0; jj < Source_Signal_PWL_Source.size(); ++jj)                                // Iterate through both vectors alternately and add elements to the result vector
        {
            std::vector<double> t_interp_PWL_Source_vector = t_interp_PWL_Source[jj];
            std::vector<double> Source_Signal_PWL_Source_vector = Source_Signal_PWL_Source[jj];
            std::string PWL_command_line_string = "(";
                // Determine the size of the resulting vector (sum of sizes of vector1 and vector2)
            int size = t_interp_PWL_Source_vector.size() + Source_Signal_PWL_Source_vector.size();
                // Iterate through both vectors alternately and add elements to the result vector
            for (int ii = 0; ii < size; ++ii) {
                if (ii < t_interp_PWL_Source_vector.size()) {
                    PWL_command_line_string += Num_To_LongEng_String(t_interp_PWL_Source_vector[ii]);   
                    PWL_command_line_string += " ";
                }
                if (ii < Source_Signal_PWL_Source_vector.size()) {
                    PWL_command_line_string += Num_To_LongEng_String(Source_Signal_PWL_Source_vector[ii]); 
                    if (ii < Source_Signal_PWL_Source_vector.size()-1)
                    {
                        PWL_command_line_string += " ";
                    }
                }
            }
            PWL_command_line_string += ")";
            PWL_command_line.push_back(PWL_command_line_string);
        }
        // Scope_String_Vector(PWL_sources_contents);               // DEBUG: Scope what desired for debug purposes
        
            // Extract from the NETLIST the PWL sources lines
        int ii = 0;
        int jj = 0; 
        for (const auto& string_line : NETLIST_contents)
        {
            std::string str = string_line;                               // String to analyze
            std::string pat = "PWL(";                                    // Pattern to find
            if (ContainsPattern(str, pat))                          // We are now processing a VARiable PWL source
            {
                std::string PWL_sources_new = Extract_Before_Pattern(string_line, "PWL(");
                PWL_sources_new += "PWL";
                PWL_sources_new += PWL_command_line[ii]; 
                NETLIST_contents[jj] = PWL_sources_new;  
                ii++; 
            }
            else
            {
                // std::cerr << "!!! ERROR !!! -> The line:" << string_line << "doesn't contain PWL sources found, check the NETLIST if the PWL line is written correvtly" << std::endl; 
            }
            jj++;
        }
        //cout << Source_Signal_PWL_Source.size() << "\n";          // DEBUG: Scope some needed results
        //cout << PWL_command_line.size() << "\n";                  // DEBUG: Scope some needed results
        //cout << ii << "\n";                                       // DEBUG: Scope some needed results
        // std::cout << NETLIST_contents[3] << "\n";                // DEBUG: Scope some needed results

    }
    
    // -------- Set the inductances in the NETLIST --------
        // Isolate the lines with Inductances
    std::vector<std::string> Inductances1;
    std::vector<std::string> str = First_Letter_Extractor(NETLIST_contents);
    std::vector<bool> TF;
    int ii = 0; 
    for (const auto& string_line : str) 
    {
        std::string str = string_line;                   // String to analyze 
        std::string pat = "L";                           // Pattern to find 
        if (ContainsPattern(str, pat)) 
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
        std::string new_string_line = Extract_Before_Pattern(string_line, " ");
        Inductances.push_back(new_string_line);
    }
    // Scope_String_Vector(Inductances);               // DEBUG: Scope what desired for debug purposes

    // -------- Upload the results data --------
    std::vector<std::string> res_name;
    res_name = node_name;
    res_name.insert(res_name.end(), branch_name.begin(), branch_name.end()); 
    std::vector<std::vector<double>> res_val = Concatenate_2_Vectors(node_val, branch_val);
    
    // -------- Set the inductance current initial conditions IC: I(L) --------
    Inductances1.clear();
    std::vector<std::string> ToFind = To_Lower_Case(Inductances);
    // Scope_String_Vector(ToFind);                     // DEBUG: Scope what desired for debug purposes
    int jj = 0;
    for (const auto& string_line : ToFind)
    {
        ii = 0;
        double IL_value_IC;
        for (const auto& string_line1 : branch_name) 
        {
            std::string str = string_line1;                      // String to analyze
            std::string pat = string_line;                       // Pattern to find
            if (ContainsPattern(str, pat))
            {
                std::vector<double> IL_value_IC1 = branch_val[ii];
                IL_value_IC = IL_value_IC1.back(); 
            }
            ii++;
        }
        std::string new_string_line = ".param ic" + Inductances[jj] + "=" + Num_To_LongEng_String(IL_value_IC); 
        Inductances1.push_back(new_string_line); 
        jj++;
    }
    // Scope_String_Vector(Inductances1);               // DEBUG: Scope what desired for debug purposes 
    
        
    // -------- Reinitialize the ".param ic" rows in the NETLIST --------
    ii = 0;
    jj = 0; 
    for (const auto& string_line : NETLIST_contents)
    {
        std::string str = string_line;                       // String to analyze 
        std::string pat = ".param ic";                       // Pattern to find  
        if (ContainsPattern(str, pat))
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
        V_IC_array += "V("; 
        V_IC_array += string_line; 
        V_IC_array += ")=";
        std::vector<double> node_val_IC = node_val[jj];
        V_IC_array += Num_To_LongEng_String(node_val_IC.back()); 
        if (ii < node_name.size())  
        {
            V_IC_array += " "; 
        }
        ii++;
        jj++;
    }
    
    // -------- Introduce the array in the netlist --------
    ii = 0;  
    for (const auto& string_line : NETLIST_contents)
    {
        std::string str = string_line;                       // String to analyze  
        std::string pat = ".ic";                             // Pattern to find 
        if (ContainsPattern(str, pat))
        {
            NETLIST_contents[ii] = V_IC_array;
        }
        ii++;
    }
    
    // -------- Set the parameters --------
    std::vector<std::string> Parameters1;
    jj = 0;                                             // Iterator over the INPUT_contents elements 
    for (const auto& string_line : INPUT_contents) 
    {
        std::string Parameters1_line = ".param par" + string_line + "=" + Num_To_LongEng_String(INPUT_values[jj][0]);
        Parameters1.push_back(Parameters1_line);
        jj++; 
    }
    // Scope_String_Vector(Parameters1);               // DEBUG: Scope what desired for debug purposes 

    // -------- Introduce the array in the netlist --------
    ii = 0;
    jj = 0;
    for (const auto& string_line : NETLIST_contents)
    {
        std::string str = string_line;                       // String to analyze 
        std::string pat = ".param par";                      // Pattern to find 
        if (ContainsPattern(str, pat))
        {
            NETLIST_contents[ii] = Parameters1[jj]; 
            jj++; 
        }
        ii++; 
    }
    
    // -------- Update the NETLIST --------
    Write_File(NETLIST_name, ""); 
    ii = 1;
    for (const auto& string_line : NETLIST_contents)    // The NETLIST_contents variable was previously updated by the above two functions 
    {
        Append_File(NETLIST_name, string_line); 
        if (ii < NETLIST_contents.size())
        {
            Append_File(NETLIST_name, "\n");
        }
        ii++;
    }

    // -------- Update the global SPICE results storing them in the Storage disk to clean-up the RAM --------
        // Get the list of all the folders inside the Spice_circuit/Results folder
    std::string currentDirectory = fs::current_path().string();
    std::string WorkingDirectory = currentDirectory + "/" + "Spice_circuit" + "/" + "Results";
        
        // Save the results in the appropriate folders
        // Voltage nodes
    auto result1 = Sort_With_Indices(node_name);
    std::vector<size_t> Idx_sort_node_name = result1.second;
    std::vector<std::vector<double>> node_val_Vec_new; 
    for (const auto& size_value : Idx_sort_node_name)
    {
        node_val_Vec_new.push_back(node_val[size_value]); 
    }
    Save_Vector_Vector_double_To_Txt(WorkingDirectory + "/" + "node_val_folder" + "/" + "node_val_Vec" + std::to_string(sim_step) + ".txt", node_val_Vec_new);

        // Current branches
    auto result2 = Sort_With_Indices(branch_name);
    std::vector<size_t> Idx_sort_branch_name = result2.second;
    std::vector<std::vector<double>> branch_val_Vec_new;
    for (const auto& size_value : Idx_sort_branch_name) 
    {
        branch_val_Vec_new.push_back(branch_val[size_value]);
    }
    Save_Vector_Vector_double_To_Txt(WorkingDirectory + "/" + "branch_val_folder" + "/" + "branch_val_Vec" + std::to_string(sim_step) + ".txt", branch_val_Vec_new);

        // Simulation time
    std::vector<double> sim_time_Vec_new;
    for (size_t kk = 0; kk < sim_time.size(); ++kk)
    {
        sim_time_Vec_new.push_back(sim_time[kk] + sim_time_last); 
    }
    Save_Vector_double_To_Txt(WorkingDirectory + "/" + "sim_time_folder" + "/" + "sim_time_Vec" + std::to_string(sim_step) + ".txt", sim_time_Vec_new);

    // -------- OUTPUT_extraction --------
    OUTPUT_value.clear();  
    for (const auto& string_line : To_Lower_Case(OUTPUT_contents))  
    {
        ii = 0;
        for (const auto& string_line1 : res_name)  
        { 
            std::string str = string_line;                               // String to analyze    
            std::string pat = string_line1;                              // Pattern to find   
            if (StrCompare(str, pat))
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
    // cout << sim_time_last << "\n";                               // DEBUG: Scope some needed results

    // -------- Increment the simulation step --------
    sim_step++;
}  
