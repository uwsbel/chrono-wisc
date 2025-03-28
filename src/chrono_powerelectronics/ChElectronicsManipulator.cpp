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
#include "ChElectronicsManipulator.h"

// =============================
// ======== Constructor ========
// =============================
ChElectronicsManipulator::ChElectronicsManipulator(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var)
    : ChElectronics(NETLIST_name_var, INPUT_name_var, OUTPUT_name_var, PWL_sources_name_var, Param_IC_name_var, Python_simulator_name_var) {}  

// ==========================================================
// ======== Methods: files and variables manipulator ========
// ==========================================================
    // ======== Method: allows to read a text file ========
std::vector<std::string> ChElectronicsManipulator::Read_File(std::string File_name)
{
    std::vector<std::string> File_contents;
    std::ifstream newfile; 
    newfile.open(File_name, std::ios::in);                    // Open the text file
    if (newfile.is_open())                              // Check if the file is open
    {
        std::string file_line;
        while (std::getline(newfile, file_line))             // Loop over the text lines
        {
            File_contents.push_back(file_line);         // Attach the readed lines to the c++ string variable
        }
        newfile.close();                                // Close the text file
        //cout << File_contents.back() << "\n";           // Scope to test the results
    }
    return File_contents;
}

    // ======== Method: allows to write into a text file ========
void ChElectronicsManipulator::Write_File(std::string File_name, std::string File_contents)
{
    std::ofstream newfile; 
    newfile.open(File_name, std::ios::out);                  // Open the text file 
    if (newfile.is_open())                              // Check if the file is open 
    {
        newfile << File_contents;                       // Write into the file
        newfile.close();                                // Close the text file
        //cout << File_contents.back() << "\n";           // Scope to test the results
    }
}

    // ======== Method: allows to append into a text file ========
void ChElectronicsManipulator::Append_File(std::string File_name, std::string File_contents)
{
    std::ofstream newfile;
    newfile.open(File_name, std::ios::app);                  // Open the text file
    if (newfile.is_open())                              // Check if the file is open
    {
        newfile << File_contents;                       // Write into the file
        newfile.close();                                // Close the text file
        // cout << File_contents.back() << "\n";           // Scope to test the results
    }
}

    // ======== Method: allows to export vector<double> variable in .dat files ========
void ChElectronicsManipulator::Dat_File_Vector_double(std::string File_name, std::vector<double> File_contents)
{
    // Open a file named "data.dat" in binary mode for writing
    std::ofstream outFile(File_name, std::ios::out | std::ios::binary); 
    // Check if the file is successfully opened
    if (!outFile) { 
        std::cerr << "Error opening the file!" << std::endl;
        // return 1;
    }
    // Write the vector data to the file
    outFile.write(reinterpret_cast<const char*>(File_contents.data()), File_contents.size() * sizeof(double));  
    // Close the file
    outFile.close(); 
}

    // ======== Method: allows to export vector<vector<double>> variables in .dat files ========
void ChElectronicsManipulator::Dat_File_Vector_Vector_double(std::string File_name, std::vector<std::vector<double>> File_contents)
{
    // Open a file named "data.dat" in binary mode for writing
    std::ofstream outFile(File_name, std::ios::out | std::ios::binary); 
    // Check if the file is successfully opened
    if (!outFile) { 
        std::cerr << "Error opening the file!" << std::endl; 
        // return 1;
    }
    // Write the vector data to the file
    outFile.write(reinterpret_cast<const char*>(File_contents.data()), File_contents.size() * sizeof(double)); 
    // Close the file
    outFile.close();  
}

    // ======== Method: allows to export vector<double> variables into .txt files ========
void ChElectronicsManipulator::Save_Vector_double_To_Txt(const std::string& file_name, const std::vector<double>& data) {
    std::ofstream outFile(file_name); 
    // Check if the file is successfully opened
    if (!outFile) {
        std::cerr << "Error opening the file!" << std::endl; 
        return;
    }
    // Save data to the file
    for (const auto& value : data) {
        outFile << Num_To_LongEng_String(value) << " ";
    }
    // Close the file
    outFile.close(); 
}

    // ======== Method: allows to save into a vector<double> variable from .txt files ========
std::vector<double> ChElectronicsManipulator::Read_Txt_File_Into_Vector_double(const std::string& file_name) {
    std::vector<double> data;
    // Open the file in input mode
    std::ifstream inFile(file_name);
    // Check if the file is successfully opened
    if (!inFile) {
        std::cerr << "Error opening the file!" << std::endl;
        return data;
    }
    double value;
    while (inFile >> value) {
        data.push_back(value);
    }
    // Close the file
    inFile.close();
    return data;
}

    // ======== Method: allows to export vector<vector<double>> variables in .txt files ========
void ChElectronicsManipulator::Save_Vector_Vector_double_To_Txt(const std::string& file_name, const std::vector<std::vector<double>>& data) {
    std::ofstream outFile(file_name);
    // Check if the file is successfully opened
    if (!outFile) {
        std::cerr << "Error opening the file!" << std::endl;
        return;
    }
    // Save data to the file
    for (const auto& row : data) {
        for (const auto& value : row) {
            outFile << Num_To_LongEng_String(value) << " "; 
        }
        outFile << std::endl;
    }
    // Close the file
    outFile.close();
}

    // ======== Method: allows to save into a vector<vector<double>> variable from .txt files ========
std::vector<std::vector<double>> ChElectronicsManipulator::Read_Txt_File_Into_Vector_Vector_double(const std::string& file_name)
{
    std::vector<std::vector<double>> data;
    // Open the file in input mode
    std::ifstream inFile(file_name);
    // Check if the file is successfully opened
    if (!inFile) {
        std::cerr << "Error opening the file!" << std::endl;
        return data;
    }
    std::string line;
    while (getline(inFile, line)) {
        std::vector<double> row;
        std::istringstream iss(line); 
        double value;
        while (iss >> value) {
            row.push_back(value);
        }
        data.push_back(row);
    }
    // Close the file
    inFile.close();
    return data;
}

    // ======== Method: allows to know the folder inside a particular directory ========
std::vector<std::string> ChElectronicsManipulator::List_Folders_In_Directory(const std::string& directoryPath) {
    std::vector<std::string> Folder_list;
    try {
        for (const auto& entry : fs::directory_iterator(directoryPath))
        {
            if (fs::is_directory(entry.path()))
            {
                Folder_list.push_back(entry.path().filename().string());
            }
        }
    }
    catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
    return Folder_list;
}

    // ======== Method: allows to check if a string pattern is contained inside a string ========
bool ChElectronicsManipulator::ContainsPattern(const std::string& str, const std::string& pattern)
{
    if (str.find(pattern) != std::string::npos) 
    {
        return true;                                        // Pattern found in the current string
    }
    return false;                                           // Pattern not found in any string
}

    // ======== Method: allows to check if a string is enterelly equal to another string ========
bool ChElectronicsManipulator::StrCompare(const std::string& str, const std::string& pattern)
{
    size_t position = str.find(pattern);
    if (position != std::string::npos && position == 0 && str.length() == pattern.length())
    {
        return true; // Pattern found at the beginning of the string and the lengths match
    }
    return false; // Pattern not found or not at the beginning or lengths don't match
}

    // ======== Method: allows to extract the first letter from all the string lines ========
std::vector<std::string> ChElectronicsManipulator::First_Letter_Extractor(std::vector<std::string>& File_contents)
{
    std::vector<std::string> First_letters_contents;
    for (const auto& string_line : File_contents)
    {
        std::string str = string_line;
        std::string letter(1, str[0]);
        First_letters_contents.push_back(letter);
    }
    return First_letters_contents;
}

    // ======== Method: allows to extract a string before a certain pattern ========
std::string ChElectronicsManipulator::Extract_Before_Pattern(const std::string& input, const std::string& pattern) {
    size_t patternPos = input.find(pattern);
    if (patternPos != std::string::npos) {
        return input.substr(0, patternPos);
    }
    // Pattern not found, return the original string
    return input;
}

    // ======== Method: allows to replicate the Matlab function : y = linspace(x1,x2,n) ========
std::vector<double> ChElectronicsManipulator::Linspace(double x1, double x2, int n) {
    std::vector<double> result;
    if (n <= 1) {
        result.push_back(x1);
        return result;
    }

    double step = (x2 - x1) / (n - 1);
    for (int i = 0; i < n; ++i) {
        result.push_back(x1 + step * i);
    }
    return result;
}

    // ======== Method: allows to return both the sorted values and their corresponding indices ========
struct IndexedValue {
    size_t index;
    std::string value;
    // Constructor to initialize index and value
    IndexedValue(size_t i, const std::string& v) : index(i), value(v) {}
    // Comparator for sorting based on the value
    static bool compareByValue(const IndexedValue& a, const IndexedValue& b) {
        return a.value < b.value;
    }
};

std::pair<std::vector<std::string>, std::vector<size_t> > ChElectronicsManipulator::Sort_With_Indices(const std::vector<std::string>& input)
{
    // Create a vector of IndexedValue structs
    std::vector<IndexedValue> indexedStrings;  
    for (size_t i = 0; i < input.size(); ++i) {
        indexedStrings.emplace_back(i, input[i]);
    }
    // Sort the vector of IndexedValue based on string values
    sort(indexedStrings.begin(), indexedStrings.end(), IndexedValue::compareByValue); 
    // Extract sorted strings and indices
    std::vector<std::string> sortedStrings;
    std::vector<size_t> sortedIndices;
    for (const auto& indexedValue : indexedStrings) {
        sortedStrings.push_back(indexedValue.value);
        sortedIndices.push_back(indexedValue.index);
    }
    return make_pair(sortedStrings, sortedIndices);
}

    // ======== Method: allows to find a string and replace with another one ========
std::string ChElectronicsManipulator::Replace_Substring(const std::string& str, const std::string& oldStr, const std::string& newStr) {
    std::string result = str;
    size_t pos = 0;
    // Find the first occurrence of oldStr in the string
    while ((pos = result.find(oldStr, pos)) != std::string::npos) { 
        // Replace the old substring with newStr
        result.replace(pos, oldStr.length(), newStr);
        // Move the position after the replaced substring
        pos += newStr.length();
    }
    return result;
}

    // ======== Method: allows to lower case a vector<string> ========
std::vector<std::string> ChElectronicsManipulator::To_Lower_Case(const std::vector<std::string>& strings) {
    std::vector<std::string> result;
    for (const std::string& str : strings) {
        std::string lowerStr = str; 
        transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower); 
        result.push_back(lowerStr); 
    }
    return result;
}

    // ======== Method: allows to scope on the cmd a string vector for debug purposes ========
void ChElectronicsManipulator::Scope_String_Vector(std::vector<std::string>& File_contents)
{
    for (const auto& string_line : File_contents)
    {
        std::cout << string_line << "\n"; 
    }
}

    // ======== Method: allows to scope on the cmd a size_t vector for debug purposes ========
void ChElectronicsManipulator::Scope_size_t_Vector(std::vector<size_t>& File_contents)
{
    for (const auto& size_t_line : File_contents) 
    {
        std::cout << size_t_line << "\n";
    }
}

    // ======== Method: allows to concatenate two vector<vector<double>> ========
std::vector<std::vector<double>> ChElectronicsManipulator::Concatenate_2_Vectors(const std::vector<std::vector<double>>& vector1, const std::vector<std::vector<double>>& vector2)
{
    std::vector<std::vector<double>> result = vector1;
    // Concatenating vector2 into the result vector
    for (const auto& innerVector : vector2) {
        result.push_back(innerVector);
    }
    return result;
}

    // ======== Method: allows to convert any type of number (int, double, double) into a string in LongEng format ========
/*
std::string ChElectronicsManipulator::Num_To_LongEng_String(double number)
{ 
    std::ostringstream oss;
    oss << std::scientific << std::uppercase << std::setprecision(8) << number; 
    std::string result = oss.str();
    size_t posE = result.find('E');
    std::string mantissa = result.substr(0, posE);
    std::string exponent = result.substr(posE + 1);
    //std::cout <<"MANTISSONE" << mantissa + "e" + exponent << "\n";
    return mantissa + "e" + exponent;
}*/
 
std::string ChElectronicsManipulator::Num_To_LongEng_String(double number)
{
    std::ostringstream oss;
    oss << std::scientific << std::uppercase << std::setprecision(8) << number;
    std::string result = oss.str();

    size_t posE = result.find('E');
    std::string mantissa = result.substr(0, posE);

    // Convert mantissa to fixed-point format to remove unnecessary zeros
    std::istringstream iss(mantissa);
    double mantissaValue;
    iss >> mantissaValue;

    std::ostringstream mantissaStream;
    mantissaStream << std::fixed << mantissaValue;
    mantissa = mantissaStream.str();

    std::string exponent = result.substr(posE + 1);
    return mantissa + "e" + exponent;
}
