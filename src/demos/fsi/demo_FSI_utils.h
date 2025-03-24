#ifndef DEMO_FSI_UTILS_H
#define DEMO_FSI_UTILS_H

#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

/// Convert a floating point number to scientific notation string without decimal points.
/// This is particularly useful for creating filesystem-safe parameter strings.
/// For example:
///   0.0025    -> "25e-4"
///   0.015     -> "15e-3"
///   0.000125  -> "125e-6"
///   1.25      -> "125e-2"
///   3.00001   -> "3"      // Numbers very close to integers are simplified
///   2.99999   -> "3"      // Numbers very close to integers are rounded
inline std::string FloatToScientific(double val) {
    // Handle numbers very close to integers
    if (std::abs(val - std::round(val)) < 1e-5) {
        return std::to_string(static_cast<int>(std::round(val)));
    }

    // For other numbers, use scientific notation but with cleaner decimal handling
    std::stringstream ss;
    ss << std::scientific << std::setprecision(6) << val;
    std::string str = ss.str();
    
    // Remove trailing zeros after decimal point
    size_t e_pos = str.find('e');
    if (e_pos != std::string::npos) {
        size_t dot_pos = str.find('.');
        if (dot_pos != std::string::npos) {
            // Find last non-zero digit before 'e'
            size_t last_non_zero = e_pos - 1;
            while (last_non_zero > dot_pos && str[last_non_zero] == '0') {
                last_non_zero--;
            }
            // If we stopped at the decimal point, remove it
            if (last_non_zero == dot_pos) {
                str.erase(dot_pos, e_pos - dot_pos);
            } else {
                str.erase(last_non_zero + 1, e_pos - (last_non_zero + 1));
            }
        }
    }
    
    return str;
}

/// Create a unique directory name for FSI simulation outputs based on parameters.
/// This function creates a consistent naming scheme that avoids problematic characters
/// and is safe for filesystem operations.
inline std::string CreateFsiOutputName(const std::string& prefix,
                                     double initial_spacing,
                                     double d0,
                                     double step_size,
                                     double density,
                                     double v_max,
                                     const std::string& viscosity_type,                                     
                                     const std::string& run_tag="") {
    std::stringstream dir_name;
    dir_name << prefix
             << "_h_"    << FloatToScientific(initial_spacing)
             << "_d0_"   << FloatToScientific(d0)
             << "_dt_"   << FloatToScientific(step_size)
             << "_rho_"  << FloatToScientific(density)
             << "_vmax_" << FloatToScientific(v_max)
             << "_viscosityType_"   << viscosity_type;
    
    if (!run_tag.empty() && run_tag != "default") {
        dir_name << "_" << run_tag;
    }
    
    return dir_name.str();
}

#endif // DEMO_FSI_UTILS_H 