// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Huzaifa Unjhawala
// =============================================================================
//
// Python-friendly CUDA error handling for SPH simulations
//
// =============================================================================

#ifndef CH_SPH_ERROR_HANDLER_H
#define CH_SPH_ERROR_HANDLER_H

#include "chrono_fsi/ChApiFsi.h"
#include <string>

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_utils
/// @{

/// Check if a CUDA error has occurred in the SPH system.
/// @return true if an error occurred, false otherwise
CH_FSI_API bool HasCudaError();

/// Get the error message if a CUDA error occurred.
/// @return the error message, or empty string if no error
CH_FSI_API std::string GetCudaErrorMessage();

/// Clear the CUDA error state.
CH_FSI_API void ClearCudaError();

/// @} fsisph_utils

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
