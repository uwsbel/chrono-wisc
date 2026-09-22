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
// Authors: Patrick Chen
// =============================================================================
//
// Wave-domain radar sensor.
//
// =============================================================================

#ifndef CHPHYSRADARSENSOR_H
#define CHPHYSRADARSENSOR_H

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/sensors/radar/ChRadarConfig.h"
#include "chrono_sensor/sensors/radar/ChRadarMaterialRegistry.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Frequency modulated continuous wave radar modeled in the wave domain.
///
/// Rays carry delay, Doppler, amplitude and phase rather than range and reflectivity, and the
/// sensor forms a range-Doppler-angle map from their coherent sum before detecting on it. The
/// consequence is that its errors are radar errors: a target is missed because its return fell
/// under a detection threshold, a ghost appears because a guardrail reflected a real target into
/// a mirrored bearing, and a detection is late because a track needed several cycles to confirm.
/// None of that is injected; all of it follows from the signal.
///
/// The sensor ray traces at rays_azimuth by rays_elevation, which is what its width and height
/// report, and produces one radar frame per update.
class CH_SENSOR_API ChPhysRadarSensor : public ChOptixSensor {
  public:
    /// @param parent body the sensor is attached to
    /// @param updateRate rate at which the radar completes a cycle [Hz]
    /// @param offsetPose mounting pose relative to the parent
    /// @param config radar model; see MakeDefaultFrontRadarConfig and ChRadarModelConfig::ReadJSON
    /// @param build_default_chain append signal formation, detection and tracking. Pass false to
    ///        work with the raw path list, which no later stage can be removed to recover.
    ChPhysRadarSensor(std::shared_ptr<ChBody> parent,
                      float updateRate,
                      ChFrame<double> offsetPose,
                      const ChRadarModelConfig& config,
                      bool build_default_chain = true);

    ~ChPhysRadarSensor() override;

    /// Radar model this sensor was built with.
    const ChRadarModelConfig& GetConfig() const { return m_config; }

    /// Surface responses used by this radar. Assign entries before the first update.
    ChRadarMaterialRegistry& GetMaterialRegistry() { return m_materials; }
    const ChRadarMaterialRegistry& GetMaterialRegistry() const { return m_materials; }

    /// Translational velocity of the parent body, which sets the Doppler of everything static.
    ChVector3d GetTranslationalVelocity() const { return m_parent->GetPosDt(); }

    /// Angular velocity of the parent body.
    ChVector3d GetAngularVelocity() const { return m_parent->GetAngVelParent(); }

  private:
    ChRadarModelConfig m_config;
    ChRadarMaterialRegistry m_materials;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
