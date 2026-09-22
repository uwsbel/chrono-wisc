// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Container class for an GPS sensor
//
// =============================================================================

#ifndef CHGPSSENSOR_H
#define CHGPSSENSOR_H

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/utils/ChGPSUtils.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// GPS class.
///
/// Maps the Cartesian simulation space onto the WGS-84 ellipsoid to produce the latitude, longitude
/// and altitude of the antenna. The reference location fixes the GPS coordinates of the simulation
/// origin. The mapping is performed such that the +Z-axis points up, +X-axis points East, and the
/// +Y-axis points North.
///
/// The reference is ordered (LONGITUDE, LATITUDE, ALTITUDE), which is the opposite of the ordering
/// used by NMEA, by most GPS libraries and by chrono::synchrono::GPScoord. A swapped pair is a valid
/// coordinate somewhere else on Earth, so nothing downstream can detect the mistake.
class CH_SENSOR_API ChGPSSensor : public ChDynamicSensor {
  public:
    /// Class constructor
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param gps_reference GPS coordinates of the simulation origin, as
    /// (LONGITUDE, LATITUDE, ALTITUDE) in degrees and metres. Note the ordering: longitude first.
    /// @param noise_model The noise model that should be used for augmenting the GPS data.
    ChGPSSensor(std::shared_ptr<ChBody> parent,
                float updateRate,
                ChFrame<double> offsetPose,
                ChVector3d gps_reference,
                std::shared_ptr<ChNoiseModel> noise_model);

    ~ChGPSSensor();

    virtual void PushKeyFrame() override;
    virtual ChKeyFrameStoreBase& KeyFrames() override { return m_keyframes; }

    /// Get the GPS reference location.
    /// @return The simulation origin as (longitude, latitude, altitude) in degrees and metres
    const ChVector3d GetGPSReference() const { return m_gps_reference; }

    /// Set the solution quality reported on every sample.
    ///
    /// A receiver reports these alongside the position, and consumers such as the ROS NavSatFix
    /// handler need them. They are constant unless the noise model is a ChNoiseGPS, which reports
    /// its own state and so overrides these while it models an outage.
    /// @param fix Quality of the position solution
    /// @param hdop Horizontal dilution of precision
    /// @param num_satellites Satellites used in the solution
    void SetNominalFix(ChGPSFixType fix, double hdop, unsigned int num_satellites);

    /// Get the configured solution quality.
    /// @return The fix type reported when the noise model does not override it
    ChGPSFixType GetNominalFixType() const { return m_fix; }

    /// Get the configured horizontal dilution of precision.
    /// @return The dilution of precision reported when the noise model does not override it
    double GetNominalHDOP() const { return m_hdop; }

    /// Get the configured satellite count.
    /// @return The satellite count reported when the noise model does not override it
    unsigned int GetNominalNumSatellites() const { return m_num_satellites; }

  private:
    /// One antenna sample: the time it was taken, and the antenna position and velocity in the
    /// simulation frame.
    struct KeyFrame {
        float time;
        ChVector3d position;
        ChVector3d velocity;
    };

    ChKeyFrameStore<KeyFrame> m_keyframes;  ///< sensor keyframes
    const ChVector3d m_gps_reference;  ///< reference location in GPS coordinates (longitude, latitude, altitude)

    ChGPSFixType m_fix = ChGPSFixType::SPS;  ///< configured solution quality
    double m_hdop = 1.0;                     ///< configured horizontal dilution of precision
    unsigned int m_num_satellites = 10;      ///< configured satellite count

    friend class ChFilterGPSUpdate;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
