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
#include "ChElectronicsSources.h"

// =============================
// ======== Constructor ========
// =============================
ChElectronicsSources::ChElectronicsSources(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var)
    : ChElectronicsManipulator(NETLIST_name_var, INPUT_name_var, OUTPUT_name_var, PWL_sources_name_var, Param_IC_name_var, Python_simulator_name_var) {}

// ====================================================================
// ======== Methods: Sin Voltage & Current sources manipulator ========
// ====================================================================
    // ======== Method: allows to set the Sin_Source_Signal global variables ========
void ChElectronicsSources::Set_Sin_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Phase, const double& Source_Bias)
{
        // -------- Input --------
    // new_section -> Do you want to add a new signal section YES=true, NO=false
    // t_Actuation -> [s]
    // dt_Switch -> [s]
    // Source_Amplitude -> [V] or [A]
    // Source_Frequency -> [Hz]
    // Source_Phase -> [rad]
    // Source_Bias -> [V] or [A]
    
        // -------- Set the global variables: for a Sin source --------
    if (Source_Type_Glob == "None" || Source_Type_Glob == "Sin")
    {
        Source_Type_Glob = "Sin"; 
    }
    else 
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Sin_Source_Signal...Error... !!! -> you are tring to re-type an already set source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check the consistency of the t_Actuation time instant --------
    if (source_call_counter == 0)
    {
        t_Actuation_Glob = t_Actuation;
        source_call_counter++;
    }

    if (t_Actuation != t_Actuation_Glob)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Sin_Source_Signal...Error... !!! -> you are tring to pass a t_Actuation value different from the original source one... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");    
    }

        // ------- Initialize the global variables --------
    std::vector<double> dt_Switch_Loc = dt_Switch_Glob; 
    dt_Switch_Loc.push_back(dt_Switch); 
    double t_Actuation_interval = std::accumulate(dt_Switch_Loc.begin(), dt_Switch_Loc.end(), 0.0); 
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval) 
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Sin_Source_Signal...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
    else 
    {
        if (new_section == true) // Case in which the creation of a new section is desired
        {
            dt_Switch_Glob.push_back(dt_Switch); 
            Source_Amplitude_Glob.push_back(Source_Amplitude); 
            Source_Frequency_Glob.push_back(Source_Frequency); 
            Source_Phase_Glob.push_back(Source_Phase); 
            Source_Bias_Glob.push_back(Source_Bias); 
        }
    }
}

    // ======== Method: allows to return the relative Sin source signal PWL array respect to a time window of the simulation ========
std::vector<double> ChElectronicsSources::Sin_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var; 

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Sin")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Sin_Source_Signal_PWL...Error... !!! -> the source is a Sin source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0); 
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1) 
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Sin_Source_Signal_PWL...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Build the Source_Signal --------
    std::vector<double> Source_Signal_ultimate;
    double t_Actuation_programmed = t_Actuation_Glob;
    if ( t_clock + T_sampling_electronic <= t_Actuation_programmed)     // Case in which the acutuation instant is after the t_clock + T_sampling_electronic
    {
        std::vector<double> t_interp;                                   // Localized onto the global time line 
        for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic)
        { 
            t_interp.push_back(t);                                      // Add the current value of t to the vector 
        } 
        Source_Signal_ultimate = Linspace(0.0, 0.0, t_interp.size());
    }
    else
    {
        if (t_clock <= t_Actuation_programmed) // Case in which the acutuation instant is between t_clock and T_sampling_electronic
        {
                // -------- Window before t_Actuation_Glob --------
            std::vector<double> t_interp1;                              // Localized onto the global time line   
            for (double t = t_clock; t < t_Actuation_programmed; t += t_step_electronic)
            {
                t_interp1.push_back(t);                                 // Add the current value of t to the vector  
            }
            std::vector<double> Source_Signal_ultimate1 = Linspace(0.0, 0.0, t_interp1.size());

                // -------- Window after t_Actuation_Glob --------
            std::vector<double> t_interp2;                              // Localized onto the global time line 
            std::vector<double> Source_Signal_ultimate2;
            for (double t = t_Actuation_programmed; t <= t_clock + T_sampling_electronic; t += t_step_electronic)
            {
                t_interp2.push_back(t- t_Actuation_programmed);         // Add the current value of t to the vector 
                double t_linear = t - t_Actuation_programmed;
                Source_Signal_ultimate2.push_back(Source_Bias_Glob[0] + (Source_Amplitude_Glob[0] * sin(2.0 * M_PI * Source_Frequency_Glob[0] * t_linear + Source_Phase_Glob[0])));
            }
            
                // -------- Unify the Source_Signal_ultimate vector --------
            Source_Signal_ultimate = Source_Signal_ultimate1;
            for (double& elem : Source_Signal_ultimate2)
            {
                Source_Signal_ultimate.push_back(elem);
            }
        }
        else // Case in which the acutuation instant is before the t_clock + T_sampling_electronic
        {
                // -------- Find the minimum dt_Switch_Glob --------
            double t_Actuation_programmed1 = t_Actuation_programmed;
            std::vector<int> dt_Switch_Glob_idx;
            int ii = 0; 
            for (double& elem : dt_Switch_Glob)  
            {
                t_Actuation_programmed1 += elem;
                if (t_clock  <=  t_Actuation_programmed1)
                {
                    dt_Switch_Glob_idx.push_back(ii); 
                }
                ii++;
            }
            auto dt_Switch_Glob_idx_ptr = std::min_element(dt_Switch_Glob_idx.begin(), dt_Switch_Glob_idx.end()); 
            int dt_Switch_Glob_idx_min = *dt_Switch_Glob_idx_ptr;  

                // -------- Define the time interval for the electronics time step and the source wavefront --------
            double t_Actuation_interval = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.begin() + (dt_Switch_Glob_idx_min + 1), 0.0);        // Calculate the actuation interval
            std::vector<double> t_interp;                           // Localized onto the SPICE time line  
            for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic) { 
                t_interp.push_back(t);
                double t_interm_elem = t - (t_Actuation_Glob + t_Actuation_interval);
                Source_Signal_ultimate.push_back(Source_Bias_Glob[dt_Switch_Glob_idx_min] + (Source_Amplitude_Glob[dt_Switch_Glob_idx_min] * sin(2.0 * M_PI * Source_Frequency_Glob[dt_Switch_Glob_idx_min] * t_interm_elem + Source_Phase_Glob[dt_Switch_Glob_idx_min])));
            }
            double fictitioius_operation = Source_Signal_ultimate.size() * 1.0; // This variable is neaded only to stabilize the solution atherwise we obtain an undefined behaviour 
        }
    }
    return Source_Signal_ultimate;
}

    // ======== Method: allows to return the relative Sin source signal value respect to a time instant of the simulation ========
double ChElectronicsSources::Sin_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var; 
    t_step_electronic = t_step_electronic_var; 
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Sin")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Sin_Source_Signal_Val...Error... !!! -> the source is a Sin source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Sin_Source_Signal_Val...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!"); 
    }

        // -------- Build the Source_Signal --------
    double Source_Signal_ultimate = 0.0; 
    double t_Actuation_programmed = t_Actuation_Glob;
    
    if (t_clock <= t_Actuation_programmed) // Case in which the acutuation instant is after t_clock
    {
        Source_Signal_ultimate = 0.0;
    }
    else // Case in which the acutuation instant is before t_clock
    {
            // -------- Find the minimum dt_Switch_Glob --------
        double t_Actuation_programmed1 = t_Actuation_programmed;
        std::vector<int> dt_Switch_Glob_idx;
        int ii = 0;
        for (double& elem : dt_Switch_Glob)
        {
            t_Actuation_programmed1 += elem;
            if (t_clock <= t_Actuation_programmed1)
            {
                dt_Switch_Glob_idx.push_back(ii);
            }
            ii++;
        }
        auto dt_Switch_Glob_idx_ptr = std::min_element(dt_Switch_Glob_idx.begin(), dt_Switch_Glob_idx.end()); 
        int dt_Switch_Glob_idx_min = *dt_Switch_Glob_idx_ptr;
        double t_Actuation_interval = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.begin() + (dt_Switch_Glob_idx_min+1), 0.0);         // Calculate the actuation interval
        
        double t_actuation = t_clock - (t_Actuation_Glob + t_Actuation_interval);
        Source_Signal_ultimate = Source_Bias_Glob[dt_Switch_Glob_idx_min] + (Source_Amplitude_Glob[dt_Switch_Glob_idx_min] * sin(2.0 * M_PI * Source_Frequency_Glob[dt_Switch_Glob_idx_min] * t_actuation + Source_Phase_Glob[dt_Switch_Glob_idx_min]));
    }
    return Source_Signal_ultimate;
}

// =======================================================================
// ======== Methods: Trapez Voltage & Current sources manipulator ========
// =======================================================================
    // ======== Method: allows to set the Trapez_Source_Signal global variables ========
void ChElectronicsSources::Set_Trapez_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Bias, const double& Source_Duty, const double& Source_t_rise, const double& Source_t_fall)
{
        // -------- Input --------
    // new_section -> Do you want to add a new signal section YES=true, NO=false
    // Source_Amplitude -> [V] or [A]
    // Source_Frequency -> [Hz]
    // Source_Duty -> []
    // Source_t_rise -> [s]
    // Source_t_fall -> [s]
    // Source_Bias -> [V] or [A]

        // -------- Set the global variables: for a Sin source --------
    if (Source_Type_Glob == "None")
    {
        Source_Type_Glob = "Trapez";
    }
    else
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Trapez_Source_Signal...Error... !!! -> you are tring to re-type an already set source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
    
        // -------- Check the consistency of the t_Actuation time instant --------
    if (source_call_counter == 0)
    {
        t_Actuation_Glob = t_Actuation;
        source_call_counter++;
    }

    if (t_Actuation != t_Actuation_Glob)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Trapez_Source_Signal...Error... !!! -> you are tring to pass a t_Actuation value different from the original source one... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!"); 
    }

        // ------- Initialize the global variables --------
    std::vector<double> dt_Switch_Loc = dt_Switch_Glob;
    dt_Switch_Loc.push_back(dt_Switch);
    double t_Actuation_interval = std::accumulate(dt_Switch_Loc.begin(), dt_Switch_Loc.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Trapez_Source_Signal...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
    else
    {
        if (new_section == true) // Case in which the creation of a new section is desired
        {
            dt_Switch_Glob.push_back(dt_Switch);
            Source_Amplitude_Glob.push_back(Source_Amplitude);
            Source_Frequency_Glob.push_back(Source_Frequency);
            Source_Duty_Glob.push_back(Source_Duty);
            Source_t_rise_Glob.push_back(Source_t_rise);
            Source_t_fall_Glob.push_back(Source_t_fall);
            Source_Bias_Glob.push_back(Source_Bias);
        }
    }    
}

    // ======== Method: allows to build a single trapezoidal wavefront ========
std::pair<std::vector<double>, std::vector<double> > ChElectronicsSources::Trapez_Wavefront(const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Duty, const double& Source_t_rise, const double& Source_t_fall, const double& Source_Bias)
{
        // -------- Input --------
    // Source_Amplitude -> [V] or [A]
    // Source_Frequency -> [Hz]
    // Source_Duty -> []
    // Source_t_rise -> [s]
    // Source_t_fall -> [s]
    // Source_Bias -> [V] or [A]

        // -------- Define the shape of a single wave front --------
    double Source_Period = 1 / Source_Frequency;
    std::vector<double> Source_Signal;
    Source_Signal.push_back(0.0 + Source_Bias);
    Source_Signal.push_back(Source_Amplitude + Source_Bias);
    Source_Signal.push_back(Source_Amplitude + Source_Bias);
    Source_Signal.push_back(0.0 + Source_Bias);
    Source_Signal.push_back(0.0 + Source_Bias);
        // -------- Define the time window of a single wave front --------
    std::vector<double> t_window;
    t_window.push_back(0.0);
    t_window.push_back(0.0 + Source_t_rise);
    t_window.push_back(0.0 + Source_t_rise + Source_Period * Source_Duty);
    t_window.push_back(0.0 + Source_t_rise + Source_Period * Source_Duty + Source_t_fall);
    t_window.push_back(0.0 + Source_t_rise + Source_Period * Source_Duty + Source_t_fall + Source_Period - (0 + Source_t_rise + Source_Period * Source_Duty + Source_t_fall));
        // -------- Return the time and wavefront vector --------
    return make_pair(t_window, Source_Signal);
}

    // ======== Method: allows to return the relative Trapez source signal PWL array respect to a time window of the simulation ========
std::pair<std::vector<double>, std::vector<double> > ChElectronicsSources::Trapez_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Trapez")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Trapez_Source_Signal_PWL...Error... !!! -> the source is a Trapez source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);

    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Trapez_Source_Signal_PWL...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Build the Source_Signal --------
    std::vector<double> Source_Signal_ultimate;
    std::vector<double> t_interp;                                   // Localized onto the global time line 
    std::vector<double> t_interp1; 
    double t_Actuation_programmed = t_Actuation_Glob; 
    //std::cout << "t1:" << t_clock + T_sampling_electronic << "\n\n";
    //std::cout << "t2:" << t_Actuation_programmed << "\n\n";
    if (t_clock + T_sampling_electronic <= t_Actuation_programmed)      // Case in which the acutuation instant is after the t_clock + T_sampling_electronic
    {
        //std::vector<double> t_interp;                                   // Localized onto the global time line
        //std::vector<double> t_interp1;
        for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic)
        {
            t_interp.push_back(t);                          // Add the current value of t to the vector
            t_interp1.push_back(t - t_clock);
        }
        Source_Signal_ultimate = Linspace(0.0, 0.0, t_interp.size()); 
    }
    else
    {
        if (t_clock <= t_Actuation_programmed)                          // Case in which the acutuation instant is between t_clock and T_sampling_electronic
        {
                // -------- Window before t_Actuation_Glob --------
            //std::vector<double> t_interp;                              // Localized onto the global time line 
            //std::vector<double> t_interp1;
            for (double t = t_clock; t < t_Actuation_programmed; t += t_step_electronic) 
            {
                t_interp.push_back(t);                          // Add the current value of t to the vector
                t_interp1.push_back(t - t_clock);
            }
            std::vector<double> Source_Signal_ultimate1 = Linspace(0.0, 0.0, t_interp1.size()); 

                // -------- Window after t_Actuation_Glob --------
            double t_interp2_step =ceil(((t_clock + T_sampling_electronic)- t_Actuation_programmed)/ t_step_electronic);                                // Localized onto the global time line
            auto result = Trapez_Wavefront(Source_Amplitude_Glob[0], Source_Frequency_Glob[0], Source_Duty_Glob[0], Source_t_rise_Glob[0], Source_t_fall_Glob[0], Source_Bias_Glob[0]);
                // -------- Unify the Source_Signal_ultimate vector --------
            Source_Signal_ultimate = Source_Signal_ultimate1;
            std::vector<double> t_trapez_1 = result.first; 
            std::vector<double> Sig_trapez_1 = result.second;
                // -------- Populate a full length trapezoidal signal replicating the single one --------
            std::vector<double> t_trapez = t_trapez_1; 
            std::vector<double> Sig_trapez = Sig_trapez_1; 
            double Trapez_Sig_Replica = std::floor(((t_clock + T_sampling_electronic) - t_Actuation_programmed) / t_trapez.back());
            for (int kk = 1; kk <= int(Trapez_Sig_Replica); kk += 1)
            {
                t_trapez.insert(t_trapez.end(), t_trapez_1.begin() + 1, t_trapez_1.end());
                Sig_trapez.insert(Sig_trapez.end(), Sig_trapez_1.begin() + 1, Sig_trapez_1.end());
            }
                // -------- Manually conclude the signal up to the last time instant --------
            if (t_trapez.back() < ((t_clock + T_sampling_electronic) - t_Actuation_programmed))  
            {
                t_trapez.push_back((t_clock + T_sampling_electronic) - t_Actuation_programmed);
                Sig_trapez.push_back(Sig_trapez.back());
            }

            int ii = 1; 
            for (double& elem : t_trapez)
            {
                double t_interp_step = ceil((t_trapez[ii] - t_trapez[ii - 1]) / t_step_electronic); 
                std::vector<double> Source_Sig = Linspace(Sig_trapez[ii - 1], Sig_trapez[ii], t_interp_step);
                if (ii < t_trapez.size())
                {
                    if (Source_Sig.size() != 1) 
                    {
                        Source_Sig.pop_back(); 
                    }
                }
                Source_Signal_ultimate.insert(Source_Signal_ultimate.end(), Source_Sig.begin(), Source_Sig.end());
                ii++;
            }
        }
        else // Case in which the acutuation instant is before the t_clock + T_sampling_electronic
        {
                // -------- Find the minimum dt_Switch_Glob --------
            double t_Actuation_programmed1 = t_Actuation_programmed;
            std::vector<int> dt_Switch_Glob_idx;
            int ii = 0;
            for (double& elem : dt_Switch_Glob)
            {
                t_Actuation_programmed1 += elem;
                if (t_clock <= t_Actuation_programmed1)
                {
                    dt_Switch_Glob_idx.push_back(ii);
                }
                ii++;
            }
            auto dt_Switch_Glob_idx_ptr = std::min_element(dt_Switch_Glob_idx.begin(), dt_Switch_Glob_idx.end()); // Index of the current signal section 
            int dt_Switch_Glob_idx_min = *dt_Switch_Glob_idx_ptr;       
                // -------- Define the time interval for the electronics time step and the source wavefront --------
            double t_Actuation_interval = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.begin() + (dt_Switch_Glob_idx_min + 1), 0.0);        // Calculate the actuation interval
            auto result = Trapez_Wavefront(Source_Amplitude_Glob[dt_Switch_Glob_idx_min], Source_Frequency_Glob[dt_Switch_Glob_idx_min], Source_Duty_Glob[dt_Switch_Glob_idx_min], Source_t_rise_Glob[dt_Switch_Glob_idx_min], Source_t_fall_Glob[dt_Switch_Glob_idx_min], Source_Bias_Glob[dt_Switch_Glob_idx_min]);
            
                // -------- Unify the Source_Signal_ultimate vector --------
            std::vector<double> t_trapez_1 = result.first;
            std::vector<double> Sig_trapez_1 = result.second;

                // -------- Populate a full length trapezoidal signal replicating the single one --------
            std::vector<double> t_trapez = t_trapez_1;
            std::vector<double> Sig_trapez = Sig_trapez_1;
            double Trapez_Sig_Replica = std::floor((T_sampling_electronic) / t_trapez.back());
            
            for (int kk = 1; kk <= int(Trapez_Sig_Replica); kk += 1)
            {
                    // -------- Reallocate the new Trapez wavefront respect to the previous --------
                std::vector<double> t_trapez_2 = t_trapez_1;
                double constantToAdd = kk * t_trapez_1.back(); 
                std::transform(t_trapez_2.begin(), t_trapez_2.end(), t_trapez_2.begin(),
                    [constantToAdd](double element) { return element + constantToAdd; }); 

                    // -------- Build the complete signal --------
                t_trapez.insert(t_trapez.end(), t_trapez_2.begin() + 1, t_trapez_2.end());
                Sig_trapez.insert(Sig_trapez.end(), Sig_trapez_1.begin() + 1, Sig_trapez_1.end());
            }

                // -------- Manually conclude the signal up to the last time instant --------
            if (t_trapez.back() < T_sampling_electronic)  
            {
                t_trapez.push_back(T_sampling_electronic);   
                Sig_trapez.push_back(Sig_trapez.back());
            }
            //std::cout << "DEBUG: " << t_trapez.size() << std::endl; 
            //std::cout << "DEBUG: " << Sig_trapez.size() << std::endl; 

                // -------- Interpolate the point of the wavefront extreme discretized over t_step_electronic --------
            // Define the time interval for the electronics time step
            for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic)
            {
                t_interp1.push_back(t - t_clock);
            }

            if (t_interp1.size() >= t_trapez.size()) 
            {
                ii = 1;
                for (double& elem : t_interp1)
                {
                    double x1 = t_trapez[ii - 1];
                    double y1 = Sig_trapez[ii - 1];
                    double x2 = t_trapez[ii];
                    double y2 = Sig_trapez[ii];
                    double m = (y2 - y1) / (x2 - x1);
                    double q = y1 - m * x1;

                    if (elem <= t_trapez[ii])
                    {
                        Source_Signal_ultimate.push_back(m* elem + q);
                    }
                    else
                    {
                        ii++;
                        double x1 = t_trapez[ii - 1];
                        double y1 = Sig_trapez[ii - 1];
                        double x2 = t_trapez[ii];
                        double y2 = Sig_trapez[ii];
                        double m = (y2 - y1) / (x2 - x1);
                        double q = y1 - m * x1;
                        Source_Signal_ultimate.push_back(m* elem + q);
                    }
                }


                /*
                ii = 1;
                int kk = 0;
                for (double& elem : t_trapez)
                {
                    double x1 = t_trapez[ii - 1];
                    double y1 = Sig_trapez[ii - 1];
                    double x2 = t_trapez[ii];
                    double y2 = Sig_trapez[ii];
                    double m = (y2 - y1) / (x2 - x1);
                    double q = y1 - m * x1;
                    while (t_interp1[kk] <= t_trapez[ii])
                    {
                        Source_Signal_ultimate.push_back(m * t_interp1[kk] + q);
                        kk++;
                    }
                    ii++;
                }*/
            }
            else
            {
                // -------- Pause the cmd to see the result --------
                int wait_int;
                std::cout << "!!! WARNING: Trapez_Source_Signal_PWL...Error... !!! -> the PWL signal in not correctly discretized";
                std::cin >> wait_int;
                throw std::runtime_error("!!! WARNING !!!");
            }
            double fictitioius_operation = Source_Signal_ultimate.size() * 1.0; // This variable is neaded only to stabilize the solution otherwise we obtain an undefined behaviour
        }
    }
    //std::cout << "DEBUG: " << Source_Signal_ultimate.size() << std::endl;
    //std::cout << "DEBUG: " << t_interp1.size() << std::endl;
    //std::cout << "DEBUG: " << Source_Signal_ultimate.size() * t_step_electronic << std::endl;
    return make_pair(Source_Signal_ultimate, t_interp1);
} 

    // ======== Method: allows to return the relative Trapez source signal value respect to a time instant of the simulation ========
double ChElectronicsSources::Trapez_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Trapez")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Trapez_Source_Signal_Val...Error... !!! -> the source is a Trapez source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!"); 
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Trapez_Source_Signal_Val...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!"); 
    }

        // -------- Build the Source_Signal --------
    double Source_Signal_ultimate;
    double t_Actuation_programmed = t_Actuation_Glob;
    if (t_clock <= t_Actuation_programmed) // Case in which the acutuation instant is after t_clock
    {
        Source_Signal_ultimate = 0.0;
    }
    else // Case in which the acutuation instant is before t_clock
    {
            // -------- Find the minimum dt_Switch_Glob --------
        double t_Actuation_programmed1 = t_Actuation_programmed;
        std::vector<int> dt_Switch_Glob_idx;
        int ii = 0;
        for (double& elem : dt_Switch_Glob)
        {
            t_Actuation_programmed1 += elem;
            if (t_clock <= t_Actuation_programmed1)
            {
                dt_Switch_Glob_idx.push_back(ii);
            }
            ii++;
        }
        auto dt_Switch_Glob_idx_ptr = std::min_element(dt_Switch_Glob_idx.begin(), dt_Switch_Glob_idx.end()); // Index of the current signal section
        int dt_Switch_Glob_idx_min = *dt_Switch_Glob_idx_ptr;
        double t_Actuation_interval = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.begin() + (dt_Switch_Glob_idx_min + 1), 0.0);         // Calculate the actuation interval
        auto result = Trapez_Wavefront(Source_Amplitude_Glob[dt_Switch_Glob_idx_min], Source_Frequency_Glob[dt_Switch_Glob_idx_min], Source_Duty_Glob[dt_Switch_Glob_idx_min], Source_t_rise_Glob[dt_Switch_Glob_idx_min], Source_t_fall_Glob[dt_Switch_Glob_idx_min], Source_Bias_Glob[dt_Switch_Glob_idx_min]);
            // -------- Unify the Source_Signal_ultimate vector --------
        std::vector<double> t_window = result.first; 
        std::vector<double> Source_Signal = result.second;

            // -------- Reallocate the wavefront in the global time line --------
        double Source_Period = 1 / Source_Frequency_Glob[dt_Switch_Glob_idx_min];
        double t_section_begin = (t_Actuation_Glob + t_Actuation_interval) - dt_Switch_Glob[dt_Switch_Glob_idx_min];
        int t_section_duplication = ceil((t_clock - t_section_begin) / Source_Period);
        std::vector<double> Source_Signal_new = Source_Signal; 
        std::vector<double> t_window_new = t_window;
        
            // -------- Translate the time window --------
        double constantToAdd = t_section_begin + (t_section_duplication - 1) * Source_Period;
        std::transform(t_window_new.begin(), t_window_new.end(), t_window_new.begin(),
            [constantToAdd](double element) { return element + constantToAdd; });
        //std::cout << "DEBUG: " << t_window_new[0] << "\n";
        //std::cout << "DEBUG: " << t_window_new[1] << "\n";
        //std::cout << "DEBUG: " << t_window_new[2] << "\n";
        //std::cout << "DEBUG: " << t_window_new[3] << "\n";
        //std::cout << "DEBUG: " << t_window_new[4] << "\n";

            // -------- Find the minimum delta time respect to the clock that are major than 0 --------
        std::vector<double> min_delta_t_vec = t_window_new;
        double constantToAdd1 = -1 * t_clock;  
        std::transform(min_delta_t_vec.begin(), min_delta_t_vec.end(), min_delta_t_vec.begin(),
            [constantToAdd1](double element) { return element + constantToAdd1; }); 
        //std::cout << "DEBUG: " << min_delta_t_vec[0] << "\n";
        //std::cout << "DEBUG: " << min_delta_t_vec[1] << "\n";
        //std::cout << "DEBUG: " << min_delta_t_vec[2] << "\n";
        //std::cout << "DEBUG: " << min_delta_t_vec[3] << "\n";
        //std::cout << "DEBUG: " << min_delta_t_vec[4] << "\n";

        std::vector<int> min_delta_t_idx;
        ii = 0;
        for (const auto& elem : min_delta_t_vec)
        {
            if (elem >= 0)
            {
                min_delta_t_idx.push_back(ii);
            }
            ii++;
        }

        auto min_delta_t_idx_ptr = std::min_element(min_delta_t_idx.begin(), min_delta_t_idx.end()); // Use std::min_element to find the minimum element  
        int min_delta_t_index = *min_delta_t_idx_ptr;

            // -------- Extract the Trapez instant value @ t_clock time instant --------
        double t_window_ultimate; 
        if (min_delta_t_index == 0)
        {
            t_window_ultimate = t_window_new[min_delta_t_index];
            Source_Signal_ultimate = Source_Signal_new[min_delta_t_index];
        }
        else // Linear interpolation
        {
            double x1 = t_window_new[min_delta_t_index];
            double y1 = Source_Signal_new[min_delta_t_index];
            double x2 = t_window_new[min_delta_t_index - 1];
            double y2 = Source_Signal_new[min_delta_t_index - 1];
            double m = (y2 - y1) / (x2 - x1);
            double q = y1 - m * x1;
            t_window_ultimate = t_clock;
            Source_Signal_ultimate = m * t_window_ultimate + q;
            //std::cout << Num_To_LongEng_String(Source_Signal_ultimate) << "\n";          // DEBUG: Scope what desired for debug purposes
            //std::cout << Source_Signal_ultimate <<"\n";
        }
    }
    return Source_Signal_ultimate;
}

// ====================================================================
// ======== Methods: PWM Voltage & Current sources manipulator ========
// ====================================================================
    // ======== Method: allows to set the PWM_Source_Signal global variables ========
void ChElectronicsSources::Set_PWM_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Amplitude, const double& Source_Frequency, const double& Source_Duty, const double& Source_t_rise, const double& Source_t_fall)
{
        // -------- Input --------
    // new_section -> Do you want to add a new signal section YES=true, NO=false
    // Source_Amplitude -> [V] or [A]
    // Source_Frequency -> [Hz]
    // Source_Duty -> []
    // Source_t_rise -> [s]
    // Source_t_fall -> [s]
    // Source_Bias -> [V] or [A]

        // -------- Set the global variables: for a Sin source --------
    if (Source_Type_Glob == "None")
    {
        Source_Type_Glob = "Trapez"; // I need to save the type as a Trapez because PWM is defined over the Trapez wavefront
    }
    else
    {
            // -------- Pause the cmd to see the result --------
        int wait_int; 
        std::cout << "!!! WARNING: Set_PWM_Source_Signal...Error... !!! -> you are tring to re-type an already set source... (to continue press enter) ";
        std::cin >> wait_int; 
        throw std::runtime_error("!!! WARNING !!!"); 
    }

    
        // -------- Check the consistency of the t_Actuation time instant --------
    if (source_call_counter == 0)
    {
        t_Actuation_Glob = t_Actuation;
        source_call_counter++;
    }

    if (t_Actuation != t_Actuation_Glob)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_PWM_Source_Signal...Error... !!! -> you are tring to pass a t_Actuation value different from the original source one... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // ------- Initialize the global variables --------
    std::vector<double> dt_Switch_Loc = dt_Switch_Glob;
    dt_Switch_Loc.push_back(dt_Switch);
    double t_Actuation_interval = std::accumulate(dt_Switch_Loc.begin(), dt_Switch_Loc.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_PWM_Source_Signal...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
    else
    {
        if (new_section == true) // Case in which the creation of a new section is desired
        {
            dt_Switch_Glob.push_back(dt_Switch);
            Source_Amplitude_Glob.push_back(Source_Amplitude);
            Source_Frequency_Glob.push_back(Source_Frequency);
            Source_Duty_Glob.push_back(Source_Duty);
            Source_t_rise_Glob.push_back(Source_t_rise);
            Source_t_fall_Glob.push_back(Source_t_fall);
            Source_Bias_Glob.push_back(0.0);
        }
    }
}

    // ======== Method: allows to return the relative PWM source signal PWL array respect to a time window of the simulation ========
std::pair<std::vector<double>, std::vector<double> > ChElectronicsSources::PWM_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Trapez")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: PWM_Source_Signal_PWL...Error... !!! -> the source is a PWM source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: PWM_Source_Signal_PWL...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Build the Source_Signal --------
    auto result = Trapez_Source_Signal_PWL(t_clock_var, t_step_electronic_var, T_sampling_electronic_var); 
    std::vector<double> Source_Signal_ultimate = result.first;
    std::vector<double> t_interp1 = result.second;

    //std::cout << "Source_Signal_ultimate size PWM:"<< Source_Signal_ultimate.size() << "\n\n";
    //std::cout << "t_interp1 size PWM:" << t_interp1.size() << "\n\n";
    return make_pair(Source_Signal_ultimate, t_interp1);
}

    // ======== Method: allows to return the relative PWM source signal value respect to a time instant of the simulation ========
double ChElectronicsSources::PWM_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Trapez")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: PWM_Source_Signal_Val...Error... !!! -> the source is a PWM source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Sin_Source_Signal_Val...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
        // -------- Build the Source_Signal --------
    double Source_Signal_ultimate = Trapez_Source_Signal_Val(t_clock_var, t_step_electronic_var, T_sampling_electronic_var);
    return Source_Signal_ultimate; 
}

// =======================================================================
// ======== Methods: Linear Voltage & Current sources manipulator ========
// =======================================================================
    // ======== Method: allows to set the Linear_Source_Signal global variables ========
void ChElectronicsSources::Set_Linear_Source_Signal(const bool& new_section, const double& t_Actuation, const double& dt_Switch, const double& Source_Rate, const double& Source_Bias)
{
        // -------- Input --------
    // new_section -> Do you want to add a new signal section YES=true, NO=false
    // Source_Amplitude -> [V] or [A]
    // Source_Frequency -> [Hz]
    // Source_Duty -> []
    // Source_t_rise -> [s]
    // Source_t_fall -> [s]
    // Source_Bias -> [V] or [A]

        // -------- Set the global variables: for a Sin source --------
    if (Source_Type_Glob == "None")
    {
        Source_Type_Glob = "Linear";
    }
    else
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Linear_Source_Signal...Error... !!! -> you are tring to re-type an already set source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
    
        // -------- Check the consistency of the t_Actuation time instant --------
    if (source_call_counter == 0)
    {
        t_Actuation_Glob = t_Actuation;
        source_call_counter++;
    }

    if (t_Actuation != t_Actuation_Glob)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Linear_Source_Signal...Error... !!! -> you are tring to pass a t_Actuation value different from the original source one... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!"); 
    }

        // ------- Initialize the global variables --------
    std::vector<double> dt_Switch_Loc = dt_Switch_Glob;
    dt_Switch_Loc.push_back(dt_Switch);
    double t_Actuation_interval = std::accumulate(dt_Switch_Loc.begin(), dt_Switch_Loc.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Set_Linear_Source_Signal...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }
    else
    {
        if (new_section == true) // Case in which the creation of a new section is desired
        {
            dt_Switch_Glob.push_back(dt_Switch);
            Source_Rate_Glob.push_back(Source_Rate);
            Source_Bias_Glob.push_back(Source_Bias);
        }
    }  
}

    // ======== Method: allows to return the relative Linear source signal PWL array respect to a time window of the simulation ========
std::vector<double> ChElectronicsSources::Linear_Source_Signal_PWL(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Linear")
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Linear_Source_Signal_PWL...Error... !!! -> the source is a Linear source, you cannot change the type of a source... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
            // -------- Pause the cmd to see the result --------
        int wait_int;
        std::cout << "!!! WARNING: Linear_Source_Signal_PWL...Error... !!! -> the source signal is not set for this time instant: @t_clock... (to continue press enter) ";
        std::cin >> wait_int;
        throw std::runtime_error("!!! WARNING !!!");
    }

        // -------- Build the Source_Signal --------
    std::vector<double> Source_Signal_ultimate;
    double t_Actuation_programmed = t_Actuation_Glob;
    if (t_clock + T_sampling_electronic <= t_Actuation_programmed) // Case in which the acutuation instant is after the t_clock + T_sampling_electronic
    {
        std::vector<double> t_interp;                            // Localized onto the global time line 
        for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic)
        {
            t_interp.push_back(t);                          // Add the current value of t to the vector 
        }
        Source_Signal_ultimate = Linspace(0.0, 0.0, t_interp.size());
    }
    else
    {
        if (t_clock <= t_Actuation_programmed) // Case in which the acutuation instant is between t_clock and T_sampling_electronic
        {
                // -------- Window before t_Actuation_Glob --------
            std::vector<double> t_interp1;                            // Localized onto the global time line   
            for (double t = t_clock; t < t_Actuation_programmed; t += t_step_electronic)
            {
                t_interp1.push_back(t);                          // Add the current value of t to the vector  
            }
            std::vector<double> Source_Signal_ultimate1 = Linspace(0.0, 0.0, t_interp1.size());

                // -------- Window after t_Actuation_Glob --------
            std::vector<double> t_interp2;                             // Localized onto the global time line   
            std::vector<double> Source_Signal_ultimate2;                             // Localized onto the global time line  
            for (double t = t_Actuation_programmed; t <= t_clock + T_sampling_electronic; t += t_step_electronic) 
            {
                t_interp2.push_back(t - t_Actuation_programmed);                           // Add the current value of t to the vector
                double t_linear = t - t_Actuation_programmed;
                Source_Signal_ultimate2.push_back(Source_Rate_Glob[0] * t_linear + Source_Bias_Glob[0]);
            }
            Source_Signal_ultimate = Source_Signal_ultimate1; 
            Source_Signal_ultimate.insert(Source_Signal_ultimate.end(), Source_Signal_ultimate2.begin(), Source_Signal_ultimate2.end());
        }
        else // Case in which the acutuation instant is before the t_clock + T_sampling_electronic
        {
                // -------- Find the minimum dt_Switch_Glob --------
            double t_Actuation_programmed1 = t_Actuation_programmed;
            std::vector<int> dt_Switch_Glob_idx;
            int ii = 0;
            for (double& elem : dt_Switch_Glob)
            {
                t_Actuation_programmed1 += elem;
                if (t_clock <= t_Actuation_programmed1)
                {
                    dt_Switch_Glob_idx.push_back(ii);
                }
                ii++;
            }
            auto dt_Switch_Glob_idx_ptr = std::min_element(dt_Switch_Glob_idx.begin(), dt_Switch_Glob_idx.end()); // Index of the current signal section
            int dt_Switch_Glob_idx_min = *dt_Switch_Glob_idx_ptr;
                // -------- Define the time interval for the electronics time step and the source wavefront --------
            double t_Actuation_interval = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.begin() + dt_Switch_Glob_idx_min, 0.0);        // Calculate the actuation interval
            std::vector<double> t_interp;                           // Localized onto the SPICE time line  
            for (double t = t_clock; t <= t_clock + T_sampling_electronic; t += t_step_electronic) {
                t_interp.push_back(t);
                double t_interp_elem = t - (t_Actuation_Glob + t_Actuation_interval);
                Source_Signal_ultimate.push_back(Source_Rate_Glob[dt_Switch_Glob_idx_min] * t_interp_elem + Source_Bias_Glob[dt_Switch_Glob_idx_min]);
            }
        }
    }
    return Source_Signal_ultimate;
}

    // ======== Method: allows to return the relative Linear source signal value respect to a time instant of the simulation ========
double ChElectronicsSources::Linear_Source_Signal_Val(double t_clock_var, double t_step_electronic_var, double T_sampling_electronic_var)
{
        // -------- Set the SPICE simulation main variables --------
    t_clock = t_clock_var;
    t_step_electronic = t_step_electronic_var;
    T_sampling_electronic = T_sampling_electronic_var;

        // -------- Check if the Source is efectivelly the original source type --------
    if (Source_Type_Glob != "Linear")
    {
        std::cerr << "!!! WARNING !!! -> the source is a Sin source, you cannot change the type of a source" << std::endl;
        double break_out; // It's mandatory to create a break_out variable of the same type of the method
        return break_out; // as break  
    }

        // -------- Check if the source is set for this time instant: @t_clock --------
    double t_Actuation_interval1 = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.end(), 0.0);
    if (t_clock >= t_Actuation_Glob + t_Actuation_interval1)
    {
        std::cerr << "!!! WARNING !!! -> the source signal is not set for this time instant: @t_clock " << std::endl;
        double break_out; // It's mandatory to create a break_out variable of the same type of the method
        return break_out; // as break  
    }

        // -------- Build the Source_Signal --------
    double Source_Signal_ultimate;
    double t_Actuation_programmed = t_Actuation_Glob;
    if (t_clock <= t_Actuation_programmed) // Case in which the acutuation instant is after t_clock
    {
        Source_Signal_ultimate = 0;
    }
    else // Case in which the acutuation instant is before t_clock
    {
            // -------- Find the minimum dt_Switch_Glob --------
        double t_Actuation_programmed1 = t_Actuation_programmed;
        std::vector<int> dt_Switch_Glob_idx;
        int ii = 0;
        for (double& elem : dt_Switch_Glob)
        {
            t_Actuation_programmed1 += elem;
            if (t_clock <= t_Actuation_programmed1)
            {
                dt_Switch_Glob_idx.push_back(ii);
            }
            ii++;
        }
        auto dt_Switch_Glob_idx_ptr = std::min_element(dt_Switch_Glob_idx.begin(), dt_Switch_Glob_idx.end()); // Index of the current signal section
        int dt_Switch_Glob_idx_min = *dt_Switch_Glob_idx_ptr;
        double t_Actuation_interval = std::accumulate(dt_Switch_Glob.begin(), dt_Switch_Glob.begin() + dt_Switch_Glob_idx_min, 0.0);         // Calculate the actuation interval

        double t_actuation = t_clock - (t_Actuation_Glob + t_Actuation_interval);
        Source_Signal_ultimate = Source_Rate_Glob[dt_Switch_Glob_idx_min] * t_actuation + Source_Bias_Glob[dt_Switch_Glob_idx_min];
    }
    return Source_Signal_ultimate;
}