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
// =============================================================================

#include <cmath>

#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChConstants.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"

namespace chrono {
namespace sensor {

ChFilterGPSUpdate::ChFilterGPSUpdate(ChVector3d gps_reference, std::shared_ptr<ChNoiseModel> noise_model)
    : ChFilter("GPS Updater"), m_noise_model(noise_model), m_ref(gps_reference) {}

CH_SENSOR_API void ChFilterGPSUpdate::Apply() {
    const auto& keyframes = m_GPSSensor->m_keyframes.Active();
    if (keyframes.empty())
        return;

    // A receiver reports one epoch, not an average over an interval, so the sample is the state at
    // the instant the collection window closed.
    const auto& keyframe = keyframes.back();
    const float sample_time = keyframe.time;

    // Noise is applied in metres, before the conversion to latitude and longitude, because that is
    // where a real receiver's position error enters.
    ChVector3d coords = keyframe.position;
    if (m_noise_model)
        m_noise_model->AddNoise(coords, m_last_sample_time, sample_time);
    m_last_sample_time = sample_time;

    Cartesian2GPS(coords, m_ref);

    m_bufferOut->Buffer[0].Latitude = coords.y();
    m_bufferOut->Buffer[0].Longitude = coords.x();
    m_bufferOut->Buffer[0].Altitude = coords.z();
    m_bufferOut->Buffer[0].Time = sample_time;

    // The simulation frame is local ENU, so the antenna velocity needs no rotation.
    m_bufferOut->Buffer[0].VelEast = keyframe.velocity.x();
    m_bufferOut->Buffer[0].VelNorth = keyframe.velocity.y();
    m_bufferOut->Buffer[0].VelUp = keyframe.velocity.z();

    m_bufferOut->Buffer[0].Speed = std::hypot(keyframe.velocity.x(), keyframe.velocity.y());
    // Course is clockwise from north, unlike the counter-clockwise-from-east convention of atan2.
    double course = std::atan2(keyframe.velocity.x(), keyframe.velocity.y()) * CH_RAD_TO_DEG;
    if (course < 0)
        course += 360.0;
    m_bufferOut->Buffer[0].Course = course;

    // A ChNoiseGPS tracks outages and so knows the receiver state; anything else leaves the state
    // fixed at what the sensor was configured with.
    if (m_gps_noise_model) {
        const auto& state = m_gps_noise_model->GetState();
        m_bufferOut->Buffer[0].Fix = (GPSFix)state.fix;
        m_bufferOut->Buffer[0].HDOP = state.hdop;
        m_bufferOut->Buffer[0].NumSatellites = state.num_satellites;
        m_bufferOut->Buffer[0].Valid = state.valid;
    } else {
        m_bufferOut->Buffer[0].Fix = (GPSFix)m_GPSSensor->GetNominalFixType();
        m_bufferOut->Buffer[0].HDOP = m_GPSSensor->GetNominalHDOP();
        m_bufferOut->Buffer[0].NumSatellites = m_GPSSensor->GetNominalNumSatellites();
        m_bufferOut->Buffer[0].Valid = (m_GPSSensor->GetNominalFixType() != ChGPSFixType::NONE);
    }

    m_bufferOut->LaunchedCount = m_GPSSensor->GetNumLaunches();
    // The time the fix describes, which with a nonzero lag precedes the time it became visible.
    m_bufferOut->TimeStamp = sample_time;
}

CH_SENSOR_API void ChFilterGPSUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                 std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("GPS update filter must be first in filter graph");
    }

    m_GPSSensor = std::dynamic_pointer_cast<ChGPSSensor>(pSensor);
    if (!m_GPSSensor) {
        throw std::runtime_error("GPS Update filter can only be used on a GPS sensor\n");
    }

    if (m_noise_model) {
        m_noise_model->Initialize(pSensor, RngUsage::GpsNoise, GetRngStreamIndex());
        m_gps_noise_model = std::dynamic_pointer_cast<ChNoiseGPS>(m_noise_model);
    }

    m_bufferOut = chrono_types::make_shared<SensorHostGPSBuffer>();
    m_bufferOut->Buffer = std::make_unique<GPSData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_GPSSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0;

    bufferInOut = m_bufferOut;
}

}  // namespace sensor
}  // namespace chrono
