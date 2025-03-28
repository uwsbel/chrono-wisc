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
#include "ChElectronics.h"

// =============================
// ======== Constructor ========
// =============================
ChElectronics::ChElectronics(std::string NETLIST_name_var, std::string INPUT_name_var, std::string OUTPUT_name_var, std::string PWL_sources_name_var, std::string Param_IC_name_var, std::string Python_simulator_name_var)
    : NETLIST_name(NETLIST_name_var), INPUT_name(INPUT_name_var), OUTPUT_name(OUTPUT_name_var), PWL_sources_name(PWL_sources_name_var), Param_IC_name(Param_IC_name_var), Python_simulator_name(Python_simulator_name_var) {}
