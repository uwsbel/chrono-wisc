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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Three link solid axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/SolidThreeLinkAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
SolidThreeLinkAxle::SolidThreeLinkAxle(const std::string& filename)
    : ChSolidThreeLinkAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

SolidThreeLinkAxle::SolidThreeLinkAxle(const rapidjson::Document& d)
    : ChSolidThreeLinkAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

SolidThreeLinkAxle::~SolidThreeLinkAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a SolidThreeLinkAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void SolidThreeLinkAxle::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    if (d.HasMember("Camber Angle (deg)"))
        m_camber_angle = d["Camber Angle (deg)"].GetDouble() * CH_DEG_TO_RAD;
    else
        m_camber_angle = 0;

    if (d.HasMember("Toe Angle (deg)"))
        m_toe_angle = d["Toe Angle (deg)"].GetDouble() * CH_DEG_TO_RAD;
    else
        m_toe_angle = 0;

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read Axle Tube data
    assert(d.HasMember("Axle Tube"));
    assert(d["Axle Tube"].IsObject());

    m_axleTubeMass = d["Axle Tube"]["Mass"].GetDouble();
    m_axleTubeCOM = ReadVectorJSON(d["Axle Tube"]["COM"]);
    m_axleTubeInertia = ReadVectorJSON(d["Axle Tube"]["Inertia"]);
    m_axleTubeRadius = d["Axle Tube"]["Radius"].GetDouble();

    // Read triangular link data
    assert(d.HasMember("Triangular Link"));
    assert(d["Triangular Link"].IsObject());
    m_triangleMass = d["Triangular Link"]["Mass"].GetDouble();
    m_triangleInertia = ReadVectorJSON(d["Triangular Link"]["Inertia"]);
    m_points[TRIANGLE_C] = ReadVectorJSON(d["Triangular Link"]["Location Chassis"]);
    m_points[TRIANGLE_A] = ReadVectorJSON(d["Triangular Link"]["Location Axle"]);

    // Read longitudinal link data
    assert(d.HasMember("Longitudinal Link"));
    assert(d["Longitudinal Link"].IsObject());
    m_linkMass = d["Longitudinal Link"]["Mass"].GetDouble();
    m_linkInertia = ReadVectorJSON(d["Longitudinal Link"]["Inertia"]);
    m_points[LINK_C] = ReadVectorJSON(d["Longitudinal Link"]["Location Chassis"]);
    m_points[LINK_A] = ReadVectorJSON(d["Longitudinal Link"]["Location Axle"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Spring"], m_springRestLength);

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());
    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Axle"]);
    m_shockForceCB = ReadTSDAFunctorJSON(d["Shock"], m_shockRestLength);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
