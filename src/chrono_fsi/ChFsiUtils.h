// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Your Name
// =============================================================================

#ifndef CH_FSI_UTILS_H
#define CH_FSI_UTILS_H

#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include "chrono/core/ChApiCE.h"
#include "chrono_fsi/ChApiFsi.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_utils
/// @{

/// Convert a floating point number to scientific notation string without decimal points.
/// This is particularly useful for creating filesystem-safe parameter strings.
/// For example:
///   0.0025    -> "25e-4"
///   0.015     -> "15e-3"
///   0.000125  -> "125e-6"
///   1.25      -> "125e-2"
inline std::string FloatToScientific(double val) {
    // Find the appropriate exponent
    int exp = 0;
    double base = std::abs(val);
    
    // Shift decimal left until base is < 10
    while (base >= 10.0) {
        base /= 10;
        exp++;
    }
    
    // Shift decimal right until base is >= 1
    while (base < 1.0 && base > 0.0) {
        base *= 10;
        exp--;
    }
    
    // Convert to string: multiply base by 10^|exp| to get an integer,
    // then append 'e' and the exponent
    std::stringstream ss;
    ss << static_cast<int>(base * std::pow(10, exp < 0 ? -exp : exp));
    ss << "e" << exp;
    return ss.str();
}

/// Create a unique directory name for FSI simulation outputs based on parameters.
/// This function creates a consistent naming scheme that avoids problematic characters
/// and is safe for filesystem operations.
inline std::string CreateFsiOutputName(const std::string& prefix,
                                     double step_size,
                                     double initial_spacing,
                                     double density,
                                     const std::string& boundary_type,
                                     const std::string& viscosity_type,
                                     int ps_freq,
                                     const std::string& run_tag = "") {
    std::stringstream dir_name;
    dir_name << prefix
             << "_s" << FloatToScientific(step_size)
             << "_h" << FloatToScientific(initial_spacing)
             << "_d" << std::fixed << std::setprecision(0) << density
             << "_" << boundary_type 
             << "_" << viscosity_type
             << "_ps" << ps_freq;
    
    if (!run_tag.empty() && run_tag != "default") {
        dir_name << "_" << run_tag;
    }
    
    return dir_name.str();
}

/// @} fsi_utils

}  // end namespace fsi
}  // end namespace chrono

#endif 