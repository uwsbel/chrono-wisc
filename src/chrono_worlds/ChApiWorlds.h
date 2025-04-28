// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Nevindu M. Batagoda
// =============================================================================
//
// Macro defines for exporting DLL
// =============================================================================

#ifndef CHAPISENSOR_H_
#define CHAPISENSOR_H_

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_WORLDS
// (so that the symbols with 'CH_WORLDS_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_WORLDS)
    #define CH_WORLDS_API ChApiEXPORT
#else
    #define CH_WORLDS_API ChApiIMPORT
#endif

/**
    @defgroup sensor WORLDS module
    @brief Enabling large terrain simulations

    This module provides support for simulating planetary scale terrains

    For additional information, see:
    - the [installation guide](@ref module_worlds_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_worlds)

    - TODO: Add defgroup for worlds
*/

namespace chrono {

/// @addtogroup sensor
/// @{

/// Namespace for Chrono::Sensor
namespace worlds {}

/// @} worlds

}  // namespace chrono

#endif
