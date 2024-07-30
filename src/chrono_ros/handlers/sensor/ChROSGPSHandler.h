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
// Authors: Aaron Young
// =============================================================================
//
// ROS Handler for communicating gps information
//
// =============================================================================

#ifndef CH_ROS_GPS_HANDLER
#define CH_ROS_GPS_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChGPSSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <array>

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChGPSSensor to ROS. Will publish sensor_msgs::msg::NavSatFix.
class ChROSGPSHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to gps->GetUpdateRate().
    /// The update rate corresponds to the sensor's update rate.
    ChROSGPSHandler(std::shared_ptr<chrono::sensor::ChGPSSensor> gps, const std::string& topic_name);

    /// Full constructor. Takes a ChGPSSensor, update rate, and topic name.
    ChROSGPSHandler(double update_rate,
                    std::shared_ptr<chrono::sensor::ChGPSSensor> gps,
                    const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    /// Helper function to calculate the covariance of the accelerometer
    /// ChGPSSensor currently doesn't support covariance, so we'll use store
    /// the rolling averages and calculate the covariance here.
    std::array<double, 9> CalculateCovariance(const chrono::sensor::GPSData& gps_data);

  private:
    std::shared_ptr<chrono::sensor::ChGPSSensor> m_gps;  ///< handle to the gps sensor

    const std::string m_topic_name;                                         ///< name of the topic to publish to
    sensor_msgs::msg::NavSatFix m_gps_msg;                                  ///< message to publish
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_publisher;  ///< the publisher for the gps message

    std::array<double, 3> m_running_average; ///< rolling average of the gps data to calculate covariance
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
