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
// Authors: Radu Serban
// =============================================================================
//
// Utility classes implementing PID speed controllers.
//
// An object of this type can be used within a Chrono::Vehicle driver model to
// provide the throttle/braking outputs.
//
// =============================================================================

#include <cstdio>

#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the class ChSpeedController
// -----------------------------------------------------------------------------
ChSpeedController::ChSpeedController() : m_speed(0), m_err(0), m_errd(0), m_erri(0), m_csv(nullptr), m_collect(false) {
    // Default PID controller gains all zero (no control).
    SetGains(0, 0, 0);
}

ChSpeedController::ChSpeedController(const std::string& filename)
    : m_speed(0), m_err(0), m_errd(0), m_erri(0), m_csv(nullptr), m_collect(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    m_Kp = d["Gains"]["Kp"].GetDouble();
    m_Ki = d["Gains"]["Ki"].GetDouble();
    m_Kd = d["Gains"]["Kd"].GetDouble();

    std::cout << "Loaded JSON " << filename << std::endl;
}

ChSpeedController::~ChSpeedController() {
    delete m_csv;
}

void ChSpeedController::Reset(const ChFrameMoving<>& ref_frame) {
    m_speed = Vdot(ref_frame.GetPosDt(), ref_frame.GetRotMat().GetAxisX());
    m_err = 0;
    m_erri = 0;
    m_errd = 0;
}

double ChSpeedController::Advance(const ChFrameMoving<>& ref_frame, double target_speed, double time, double step) {
    // Current vehicle speed.
    m_speed = Vdot(ref_frame.GetPosDt(), ref_frame.GetRotMat().GetAxisX());

    // If data collection is enabled, append current target and sentinel locations.
    if (m_collect) {
        *m_csv << time << target_speed << m_speed << std::endl;
    }

    // Calculate current error.
    double err = target_speed - m_speed;

    // Estimate error derivative (backward FD approximation).
    m_errd = (err - m_err) / step;

    // Calculate current error integral (trapezoidal rule).
    m_erri += (err + m_err) * step / 2;

    // Cache new error
    m_err = err;

    // Return PID output (steering value)
    return m_Kp * m_err + m_Ki * m_erri + m_Kd * m_errd;
}

void ChSpeedController::StartDataCollection() {
    // Return now if currently collecting data.
    if (m_collect)
        return;
    // Create the ChWriterCSV object if needed (first call to this function).
    if (!m_csv) {
        m_csv = new utils::ChWriterCSV("\t");
        m_csv->Stream().setf(std::ios::scientific | std::ios::showpos);
        m_csv->Stream().precision(6);
    }
    // Enable data collection.
    m_collect = true;
}

void ChSpeedController::StopDataCollection() {
    // Suspend data collection.
    m_collect = false;
}

void ChSpeedController::WriteOutputFile(const std::string& filename) {
    // Do nothing if data collection was never enabled.
    if (m_csv)
        m_csv->WriteToFile(filename);
}

}  // end namespace vehicle
}  // end namespace chrono
