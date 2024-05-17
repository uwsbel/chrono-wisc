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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================
//
// Chrono wrappers to fmu_tools FMU export classes for FMI standard 2.0.
//
// =============================================================================

#ifndef CH_FMU_TOOLS_EXPORT_H
#define CH_FMU_TOOLS_EXPORT_H

#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>
#include <fstream>

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChFrameMoving.h"

// fmu_tools
// #include "rapidxml_ext.hpp"
#include "fmi2/FmuToolsExport.h"

namespace chrono {

/// Extension of FmuComponentBase class for Chrono FMUs.
class FmuChronoComponentBase : public FmuComponentBase {
  public:
    FmuChronoComponentBase(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
        : FmuComponentBase(
              instanceName,
              fmuType,
              fmuGUID,
              fmuResourceLocation,
              functions,
              visible,
              loggingOn,
              {{"logEvents", true},
               {"logSingularLinearSystems", true},
               {"logNonlinearSystems", true},
               {"logStatusWarning", true},
               {"logStatusError", true},
               {"logStatusPending", true},
               {"logDynamicStateSelection", true},
               {"logStatusDiscard", true},
               {"logStatusFatal", true},
               {"logAll", true}},
              {"logStatusWarning", "logStatusDiscard", "logStatusError", "logStatusFatal", "logStatusPending"}) {}

    virtual ~FmuChronoComponentBase() {}

    /// Add FMU variables corresponding to the specified ChVector3d.
    /// This function creates 3 FMU variables, one for each component of the ChVector3d, with names "name.x", "name.y",
    /// and "name.z", all of type FmuVariable::Type::Real.
    void AddFmuVecVariable(ChVector3d& v,
                           const std::string& name,
                           const std::string& unit_name,
                           const std::string& description,
                           FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                           FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous) {
        std::string comp[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            AddFmuVariable(&v.data()[i], name + "." + comp[i], FmuVariable::Type::Real, unit_name,
                           description + " (" + comp[i] + ")", causality, variability);
        }
    }

    /// Add FMU variables corresponding to the specified ChQuaternion.
    /// This function creates 4 FMU variables, one for each component of the ChVector3d, with names "name.e0",
    /// "name.e1", "name.e2", and "name.e3", all of type FmuVariable::Type::Real.
    void AddFmuQuatVariable(ChQuaternion<>& q,
                            const std::string& name,
                            const std::string& unit_name,
                            const std::string& description,
                            FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                            FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous) {
        std::string comp[4] = {"e0", "e1", "e2", "e3"};
        for (int i = 0; i < 4; i++) {
            AddFmuVariable(&q.data()[i], name + "." + comp[i], FmuVariable::Type::Real, unit_name,
                           description + " (" + comp[i] + ")", causality, variability);
        }
    }

    /// Add FMU variables corresponding to the specified ChCoordsys.
    /// This function creates 7 FMU variables, one for each component of the position ChVector3d and one for each
    /// component of the rotation quaternion, all of type FmuVariable::Type::Real.
    void AddFmuCsysVariable(ChCoordsysd& s,
                            const std::string& name,
                            const std::string& unit_name,
                            const std::string& description,
                            FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                            FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous) {
        AddFmuVecVariable(s.pos, name + ".pos", unit_name, description + " position", causality, variability);
        AddFmuQuatVariable(s.rot, name + ".rot", "1", description + " orientation", causality, variability);
    }

    /// Add FMU variables corresponding to the specified ChFrame.
    /// This function creates 7 FMU variables, one for each component of the position ChVector3d and one for each
    /// component of the rotation quaternion, all of type FmuVariable::Type::Real.
    void AddFmuFrameVariable(ChFrame<>& s,
                             const std::string& name,
                             const std::string& unit_name,
                             const std::string& description,
                             FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                             FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous) {
        AddFmuCsysVariable(s.m_csys, name, unit_name, description, causality, variability);
    }

    /// Add FMU variables corresponding to the specified ChFrameMoving.
    /// This function creates 7 FMU variables for the pose, one for each component of the position ChVector3d and one
    /// for each component of the rotation quaternion, all of type FmuVariable::Type::Real.  Additionally, 7 FMU
    /// variables are created to encode the position and orientation time derivatives.
    void AddFmuFrameMovingVariable(
        ChFrameMoving<>& s,
        const std::string& name,
        const std::string& unit_name,
        const std::string& unit_name_dt,
        const std::string& description,
        FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
        FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous) {
        AddFmuCsysVariable(s.m_csys, name, unit_name, description, causality, variability);
        AddFmuVecVariable(s.m_csys_dt.pos, name + ".pos_dt", unit_name_dt, description + " position derivative",
                          causality, variability);
        AddFmuQuatVariable(s.m_csys_dt.rot, name + ".rot_dt", "1", description + " orientation derivative", causality,
                           variability);
    }
};

// -----------------------------------------------------------------------------

#define ADD_BVAL_AS_FMU_GETSET(returnType, codeGet, codeSet)                                         \
    _fmucomp->AddFmuVariable(                                                                        \
        std::make_pair(std::function<fmi2##returnType(void)>([bVal]() -> fmi2##returnType codeGet),  \
                       std::function<void(fmi2##returnType)>([bVal](fmi2##returnType val) codeSet)), \
        getCurrentVarName(bVal.name()), FmuVariable::Type::returnType, "", "",                       \
        CausalityType_conv.at(bVal.GetCausality()), VariabilityType_conv.at(bVal.GetVariability()));

#define ADD_BVAL_AS_FMU_POINTER(returnType)                                                                          \
    _fmucomp->AddFmuVariable(&(bVal.value()), getCurrentVarName(bVal.name()), FmuVariable::Type::returnType, "", "", \
                             CausalityType_conv.at(bVal.GetCausality()),                                             \
                             VariabilityType_conv.at(bVal.GetVariability()));

const std::unordered_map<chrono::ChVariabilityType, FmuVariable::VariabilityType> VariabilityType_conv = {
    {chrono::ChVariabilityType::constant, FmuVariable::VariabilityType::constant},
    {chrono::ChVariabilityType::fixed, FmuVariable::VariabilityType::fixed},
    {chrono::ChVariabilityType::tunable, FmuVariable::VariabilityType::tunable},
    {chrono::ChVariabilityType::discrete, FmuVariable::VariabilityType::discrete},
    {chrono::ChVariabilityType::continuous, FmuVariable::VariabilityType::continuous}};

const std::unordered_map<chrono::ChCausalityType, FmuVariable::CausalityType> CausalityType_conv = {
    {chrono::ChCausalityType::parameter, FmuVariable::CausalityType::parameter},
    {chrono::ChCausalityType::calculatedParameter, FmuVariable::CausalityType::calculatedParameter},
    {chrono::ChCausalityType::input, FmuVariable::CausalityType::input},
    {chrono::ChCausalityType::output, FmuVariable::CausalityType::output},
    {chrono::ChCausalityType::local, FmuVariable::CausalityType::local},
    {chrono::ChCausalityType::independent, FmuVariable::CausalityType::independent}};

// TODO expand serialization to have description

/// Class for serializing to FmuComponentBase.
class ChOutputFMU : public ChArchiveOut {
  public:
    ChOutputFMU(FmuComponentBase& fmucomp) {
        _fmucomp = &fmucomp;

        tablevel = 0;
        nitems.push(0);
        is_array.push_back(false);
    };

    virtual ~ChOutputFMU() {
        nitems.pop();
        is_array.pop_back();
    };

    virtual void out(ChNameValue<bool> bVal) {
        ADD_BVAL_AS_FMU_GETSET(
            Boolean, { return static_cast<fmi2Boolean>(bVal.value()); }, { bVal.value() = val; })

        ++nitems.top();
    }
    virtual void out(ChNameValue<int> bVal) {
        ADD_BVAL_AS_FMU_POINTER(Integer)

        ++nitems.top();
    }
    virtual void out(ChNameValue<double> bVal) {
        ADD_BVAL_AS_FMU_POINTER(Real)

        ++nitems.top();
    }
    virtual void out(ChNameValue<float> bVal) {
        ADD_BVAL_AS_FMU_GETSET(
            Real, { return static_cast<fmi2Real>(bVal.value()); }, { bVal.value() = static_cast<float>(val); })

        ++nitems.top();
    }
    virtual void out(ChNameValue<char> bVal) {
        ADD_BVAL_AS_FMU_GETSET(
            Integer, { return static_cast<fmi2Integer>(bVal.value()); }, { bVal.value() = val; })

        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned int> bVal) {
        ADD_BVAL_AS_FMU_GETSET(
            Integer, { return static_cast<fmi2Integer>(bVal.value()); }, { bVal.value() = val; })

        ++nitems.top();
    }
    virtual void out(ChNameValue<const char*> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<std::string> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long long> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) { ++nitems.top(); }

    virtual void out_array_pre(ChValue& bVal, size_t msize) {
        pushLevelName(bVal.name());

        ++tablevel;
        nitems.push(0);
        // signaling that, from now on, serialized variables are part of an array
        is_array.push_back(true);
    }
    virtual void out_array_between(ChValue& bVal, size_t msize) {}

    virtual void out_array_end(ChValue& bVal, size_t msize) {
        --tablevel;
        nitems.pop();
        is_array.pop_back();
        ++nitems.top();
        popLevelName();
    }

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        pushLevelName(bVal.name());

        ++tablevel;
        nitems.push(0);
        is_array.push_back(false);

        bVal.CallArchiveOut(*this);

        --tablevel;
        nitems.pop();
        is_array.pop_back();

        ++nitems.top();
        popLevelName();
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        pushLevelName(bVal.name());

        ++tablevel;
        nitems.push(0);
        is_array.push_back(false);

        if (!already_inserted) {
            // New Object, we have to full serialize it
            bVal.CallArchiveOutConstructor(*this);
            bVal.CallArchiveOut(*this);
        }

        --tablevel;
        nitems.pop();

        is_array.pop_back();

        ++nitems.top();

        popLevelName();
    }

  protected:
    void pushLevelName(const std::string& newLevelName) { parent_names.push_back(newLevelName); }

    void popLevelName() { parent_names.pop_back(); }

    std::string getCurrentVarName(std::string bVal_name) {
        std::string current_fullname = "";
        if (parent_names.size() > 0)
            current_fullname = parent_names[0];

        for (size_t i = 1; i < parent_names.size(); ++i) {
            current_fullname += is_array.at(i) ? ("[" + parent_names[i] + "]") : "." + parent_names[i];
        }
        current_fullname += (is_array.back() ? ("[" + bVal_name + "]") : "." + bVal_name);

        return current_fullname;
    }

    int tablevel;
    FmuComponentBase* _fmucomp;
    std::stack<int> nitems;
    std::deque<bool> is_array;
    std::deque<std::string> parent_names;
};

}  // end namespace chrono

#endif
