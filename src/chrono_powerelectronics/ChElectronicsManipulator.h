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

#ifndef CHELECTRONICSMANIPULATOR_H
#define CHELECTRONICSMANIPULATOR_H

// =========================
// ======== Headers ========
// =========================
#include "ChElectronics.h"

// =======================
// ======== Class ========
// =======================
class ChElectronicsManipulator : public ChElectronics {
public:
    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronicsManipulator(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var);

    // ==========================================================
    // ======== Methods: files and variables manipulator ========
    // ==========================================================
        // ======== Method: allows to read a text file ========
    std::vector<std::string> Read_File(std::string File_name);

        // ======== Method: allows to write into a text file ========
    void Write_File(std::string File_name, std::string File_contents);

        // ======== Method: allows to append into a text file ========
    void Append_File(std::string File_name, std::string File_contents);

        // ======== Method: allows to export vector<double> variable in .dat files ========
    void Dat_File_Vector_double(std::string File_name, std::vector<double> File_contents);

        // ======== Method: allows to export vector<vector<double>> variables in .dat files ========
    void Dat_File_Vector_Vector_double(std::string File_name, std::vector<std::vector<double>> File_contents);

        // ======== Method: allows to export vector<double> variables into .txt files ========
    void Save_Vector_double_To_Txt(const std::string& file_name, const std::vector<double>& data);

        // ======== Method: allows to save into a vector<double> variable from .txt files ========
    std::vector<double> Read_Txt_File_Into_Vector_double(const std::string& file_name);

        // ======== Method: allows to export vector<vector<double>> variables in .txt files ========
    void Save_Vector_Vector_double_To_Txt(const std::string& file_name, const std::vector<std::vector<double>>& data);

        // ======== Method: allows to save into a vector<vector<double>> variable from .txt files ========
    std::vector<std::vector<double>> Read_Txt_File_Into_Vector_Vector_double(const std::string& file_name);

        // ======== Method: allows to know the folder inside a particular directory ========
    std::vector<std::string> List_Folders_In_Directory(const std::string& directoryPath);

        // ======== Method: allows to check if a string pattern is contained inside a string ========
    bool ContainsPattern(const std::string& str, const std::string& pattern);
    
        // ======== Method: allows to check if a string is enterelly equal to another string ========
    bool StrCompare(const std::string& str, const std::string& pattern);

        // ======== Method: allows to extract the first letter from all the string lines ========
    std::vector<std::string> First_Letter_Extractor(std::vector<std::string>& File_contents);

        // ======== Method: allows to extract a string before a certain pattern ========
    std::string Extract_Before_Pattern(const std::string& input, const std::string& pattern);

        // ======== Method: allows to replicate the Matlab function : y = linspace(x1,x2,n) ========
    std::vector<double> Linspace(double x1, double x2, int n);

        // ======== Method: allows to return both the sorted values and their corresponding indices ========
    std::pair<std::vector<std::string>, std::vector<size_t> > Sort_With_Indices(const std::vector<std::string>& input);

        // ======== Method: allows to find a string and replace with another one ========
    std::string Replace_Substring(const std::string& str, const std::string& oldStr, const std::string& newStr);

        // ======== Method: allows to lower case a vector<string> ========
    std::vector<std::string> To_Lower_Case(const std::vector<std::string>& strings);

        // ======== Method: allows to scope on the cmd a string vector for debug purposes ========
    void Scope_String_Vector(std::vector<std::string>& File_contents);

        // ======== Method: allows to scope on the cmd a size_t vector for debug purposes ========
    void Scope_size_t_Vector(std::vector<size_t>& File_contents);

        // ======== Method: allows to concatenate two vector<vector<double>> ========
    std::vector<std::vector<double>> Concatenate_2_Vectors(const std::vector<std::vector<double>>& vector1, const std::vector<std::vector<double>>& vector2);

        // ======== Method: allows to convert any type of number (int, double, double) into a string in LongEng format ========
    std::string Num_To_LongEng_String(double number); 
};

#endif // CHELECTRONICSMANIPULATOR_H
