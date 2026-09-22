// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// ROS handler for a ChGPSSensor (publishes sensor_msgs/msg/NavSatFix).
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/utils/ChGPSUtils.h"

#include <iostream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

// sensor_msgs/msg/NavSatStatus covariance-type constant (the message defines
// COVARIANCE_TYPE_APPROXIMATED = 2); the schema carries no symbolic constants.
static constexpr uint64_t COVARIANCE_TYPE_APPROXIMATED = 2;

// sensor_msgs/msg/NavSatStatus service constant for GPS; the schema carries no symbolic constants.
static constexpr uint64_t SERVICE_GPS = 1;

/// Map a Chrono fix type onto the NavSatStatus status constants: STATUS_NO_FIX = -1,
/// STATUS_FIX = 0, STATUS_SBAS_FIX = 1. NavSatStatus has no RTK distinction, so both RTK modes
/// report as augmented.
static int NavSatStatusFromFix(GPSFix fix) {
    switch (fix) {
        case GPSFix::NONE:
            return -1;
        case GPSFix::SPS:
            return 0;
        case GPSFix::DGPS:
        case GPSFix::RTK_FLOAT:
        case GPSFix::RTK_FIXED:
            return 1;
    }
    return 0;
}

ChROSGPSHandler::ChROSGPSHandler(std::shared_ptr<ChGPSSensor> gps, const std::string& topic_name)
    : ChROSGPSHandler(gps->GetUpdateRate(), gps, topic_name) {}

ChROSGPSHandler::ChROSGPSHandler(double update_rate,
                                 std::shared_ptr<ChGPSSensor> gps,
                                 const std::string& topic_name)
    : ChROSHandler(update_rate), m_gps(gps), m_topic_name(topic_name), m_running_average({0, 0, 0}) {}

bool ChROSGPSHandler::Initialize(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGPSAccess, ChFilterGPSAccessName>(m_gps)) {
        return false;
    }
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/NavSatFix");
    return true;
}

void ChROSGPSHandler::Tick(double time) {
    auto buffer = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!buffer->Buffer) {
        std::cout << "GPS: waiting for first fix..." << std::endl;  // normal during warm-up
        return;
    }

    GPSData data = buffer->Buffer[0];
    auto covariance = CalculateCovariance(data);

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_gps->GetName());
    msg.SetTime("header.stamp", data.Time);  // sensor timestamp, not the sim step time
    msg.SetDouble("latitude", data.Latitude);
    msg.SetDouble("longitude", data.Longitude);
    msg.SetDouble("altitude", data.Altitude);
    msg.SetBlobCopy("position_covariance", covariance.data(), 9);
    msg.SetUInt("position_covariance_type", COVARIANCE_TYPE_APPROXIMATED);
    msg.SetInt("status.status", NavSatStatusFromFix(data.Fix));
    msg.SetUInt("status.service", SERVICE_GPS);
    m_publisher->Publish(msg);
}

std::array<double, 9> ChROSGPSHandler::CalculateCovariance(const GPSData& gps_data) {
    // The ChGPSSensor does not emit covariance; approximate it from a running mean
    // of the position in local ENU coordinates.
    // GPS2Cartesian takes (LONGITUDE, LATITUDE, ALTITUDE), not the NavSatFix field order.
    auto gps_coord = chrono::ChVector3d(gps_data.Longitude, gps_data.Latitude, gps_data.Altitude);
    chrono::sensor::GPS2Cartesian(gps_coord, m_gps->GetGPSReference());

    std::array<double, 3> enu = {gps_coord.x(), gps_coord.y(), gps_coord.z()};
    for (int i = 0; i < 3; i++)
        m_running_average[i] += (enu[i] - m_running_average[i]) / (GetTickCount() + 1);
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);
    return ChROSSensorHandlerUtilities::CalculateCovariance(enu, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono
