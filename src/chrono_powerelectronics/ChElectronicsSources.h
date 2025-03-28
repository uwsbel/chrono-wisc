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

#ifndef CHELECTRONICSSOURCES_H
#define CHELECTRONICSSOURCES_H

// =========================
// ======== Headers ========
// =========================
#include "ChElectronicsManipulator.h"

// =======================
// ======== Class ========
// =======================
class ChElectronicsSources : public ChElectronicsManipulator {
public:
    // ============================
    // ======== Attributes ========
    // ============================
        // ======== Global variables ========
    std::string Source_Type_Glob = "None";                      // Contains the source Type: SIn, Trapez, PWM, Linear 
    double t_Actuation_Glob = 0.0;//[s]                         // Contains the actuation time instant
    std::vector<double> dt_Switch_Glob;//[s]                    // Contains the time interval of the different actuation sections 
    std::vector<double> Source_Amplitude_Glob;//[A] or [V]      // Contains the signal amplitude of the different actuation sections
    std::vector<double> Source_Frequency_Glob;//[Hz]            // Contains the signal frequencies of the different actuation sections
    std::vector<double> Source_Duty_Glob;//[]                   // Contains the signal duties of the different actuation sections
    std::vector<double> Source_t_rise_Glob;//[s]                // Contains the signal rise interval of the different actuation sections for Trapez or PWM sources
    std::vector<double> Source_t_fall_Glob;//[s]                // Contains the signal fall interval of the different actuation sections for Trapez or PWM sources
    std::vector<double> Source_Phase_Glob;//[A] or [V]          // Contains the signal phases of the different actuation sections
    std::vector<double> Source_Bias_Glob;//[A] or [V]           // Contains the signal DC bias of the different actuation sections
    std::vector<double> Source_Rate_Glob;//[A] or [V]           // Contains the signal reates of the different actuation sections for Linear sources
    int source_call_counter = 0;                                // Contains how many times you called this source
    
public:
    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronicsSources(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var);
    
    // ====================================================================
    // ======== Methods: Sin Voltage & Current sources manipulator ========
    // ====================================================================
        // ======== Method: allows to set the Sin_Source_Signal global variables ========
    void Set_Sin_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Phase, const double& Source_Bias);
    
        // ======== Method: allows to return the relative Sin source signal PWL array respect to a time window of the simulation ========
    std::vector<double> Sin_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
        // ======== Method: allows to return the relative Sin source signal value respect to a time instant of the simulation ========
    double Sin_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
    // =======================================================================
    // ======== Methods: Trapez Voltage & Current sources manipulator ========
    // =======================================================================
        // ======== Method: allows to set the Trapez_Source_Signal global variables ========
    void Set_Trapez_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Bias, const double& Source_Duty, const double& Source_t_rise, const double& Source_t_fall);  
    
        // ======== Method: allows to build a single trapezoidal wavefront ========
    std::pair<std::vector<double>, std::vector<double> > Trapez_Wavefront(const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Duty, const double& Source_t_rise, const double& Source_t_fall, const double& Source_Bias); 
    
        // ======== Method: allows to return the relative Trapez source signal PWL array respect to a time window of the simulation ========
    std::pair<std::vector<double>, std::vector<double> > Trapez_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
        // ======== Method: allows to return the relative Trapez source signal value respect to a time instant of the simulation ========
    double Trapez_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
    // ====================================================================
    // ======== Methods: PWM Voltage & Current sources manipulator ========
    // ====================================================================
        // ======== Method: allows to set the PWM_Source_Signal global variables ========
    void Set_PWM_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Duty, const double& Source_t_rise, const double& Source_t_fall);
    
        // ======== Method: allows to return the relative PWM source signal PWL array respect to a time window of the simulation ========
    std::pair<std::vector<double>, std::vector<double> > PWM_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
        // ======== Method: allows to return the relative PWM source signal value respect to a time instant of the simulation ========
    double PWM_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
    // =======================================================================
    // ======== Methods: Linear Voltage & Current sources manipulator ========
    // =======================================================================
        // ======== Method: allows to set the Linear_Source_Signal global variables ========
    void Set_Linear_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Rate, const double& Source_Bias);
    
        // ======== Method: allows to return the relative Linear source signal PWL array respect to a time window of the simulation ========
    std::vector<double> Linear_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
    
        // ======== Method: allows to return the relative Linear source signal value respect to a time instant of the simulation ========
    double Linear_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var);
 };

#endif // CHELECTRONICSSOURCES_H