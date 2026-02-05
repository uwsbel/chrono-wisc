// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ganesh Arivoli
// =============================================================================

#ifndef CHAPI_CUDSS_H
#define CHAPI_CUDSS_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_CUDSS
// (so that the symbols with 'ChApiCuDSS' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_CUDSS)
    #define ChApiCuDSS ChApiEXPORT
#else
    #define ChApiCuDSS ChApiIMPORT
#endif

/**
    @defgroup cudss_module cuDSS module
    @brief Module for the NVIDIA cuDSS GPU sparse direct solver

    This module provides an interface to the NVIDIA cuDSS sparse direct solver for GPUs.

    For additional information, see:
    - the [installation guide](@ref module_cudss_installation)
*/

#endif
