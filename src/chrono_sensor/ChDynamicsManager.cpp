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
// Class for managing the Optix rendering system
//
// =============================================================================

#include "chrono_sensor/ChDynamicsManager.h"

#include <iomanip>
#include <iostream>

namespace chrono {
namespace sensor {

ChDynamicsManager::ChDynamicsManager(ChSystem* chrono_system) {
    // save the chrono system handle
    m_system = chrono_system;
}

ChDynamicsManager::~ChDynamicsManager() {}

void ChDynamicsManager::UpdateSensors() {
    const double time = m_system->GetChTime();

    for (auto& sensor : m_sensor_list) {
        sensor->AdvanceTo(time);

        // Collect. Keyframes are sampled over [n / rate, n / rate + collection window]; the window
        // is paced by the number of windows closed, not by the number released, so a sensor lag
        // does not slow down sampling.
        const double window_start = sensor->GetNumWindows() / (double)sensor->GetUpdateRate();
        if (time > window_start - 1e-7) {
            sensor->PushKeyFrame();
            if (time > window_start + sensor->GetCollectionWindow() - 1e-7) {
                sensor->KeyFrames().Stash((float)time, (float)time + sensor->GetLag());
                sensor->IncrementNumWindows();
            }
        }

        // Release. Each closed window reaches the filter graph once its lag has elapsed, which is
        // what makes the data visible to the user that long after the window closed. A loop rather
        // than a single test, so a lag longer than the update period cannot let windows pile up.
        while (sensor->KeyFrames().DueBy((float)time)) {
            sensor->ReleaseKeyFrames();
            sensor->IncrementNumLaunches();
            for (auto& filter : sensor->GetFilterList())
                filter->Apply();
            sensor->ClearKeyFrames();
        }
    }
}

void ChDynamicsManager::AssignSensor(std::shared_ptr<ChSensor> sensor) {
    if (auto sen = std::dynamic_pointer_cast<ChDynamicSensor>(sensor)) {
        // check if sensor is already in sensor list
        if (std::find(m_sensor_list.begin(), m_sensor_list.end(), sen) != m_sensor_list.end()) {
            std::cerr << "WARNING: This sensor already exists in manager. Ignoring this addition\n";
            return;
        }
        m_sensor_list.push_back(sen);
        std::shared_ptr<SensorBuffer> buffer;
        for (auto f : sen->GetFilterList()) {
            f->Initialize(sen, buffer);
        }
        sen->LockFilterList();

    } else {
        std::cerr << "WARNING: unsupported sensor type found in the dynamic sensor manager. Ignoring...\n";
        std::cerr << "Sensor was: " << sensor->GetName() << std::endl;
    }
}

}  // namespace sensor
}  // namespace chrono
