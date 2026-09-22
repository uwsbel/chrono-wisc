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
// Base class for all sensors
//
// =============================================================================

#ifndef CHSENSOR_H
#define CHSENSOR_H

#include <deque>
#include <list>
#include <mutex>
#include <vector>

#include "chrono/physics/ChBody.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono_sensor/filters/ChFilter.h"
#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChOptixUtils.h"
#endif

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

// Global constants for use in template parameters
#ifdef CHRONO_HAS_OPTIX
const char ChFilterR8AccessName[] = "ChFilterR8Access";              /// single channel 8 bit array
const char ChFilterRGBA8AccessName[] = "ChFilterRGBA8Access";        /// 4 channel 8 bit array
const char ChFilterRGBA16AccessName[] = "ChFilterRGBA16Access";      /// 4 channels of u_int16_t (16 bit) arrays
const char ChFilterRGBDHalf4AccessName[] = "ChFilterRGBDHalf4Access";/// 4 channels of half (16 bit) arrays
const char ChFilterDIAccessName[] = "ChFilterDIAccess";              /// 2 channel float array (Depth+Intenisty)
const char ChFilterRadarAccessName[] = "ChFilterRadarAccess";        ///<
const char ChFilterRadarXYZAccessName[] = "ChFilterRadarXYZAccess";  ///<
const char ChFilterSemanticAccessName[] = "ChFilterSemanticAccess";  // 2 channels of unsigned short int (16 bit) arrays (class label + instance label)
const char ChFilterDepthAccessName[] = "ChFilterDepthAccess";        // single channel of a float array
const char ChFilterFloat4AccessName[] = "ChFilterFloat4Access";      // 4 channels of float arrays
const char ChFilterNormalAccessName[] = "ChFilterNormalAccess";      /// 3 channels of float (32 bit) arrays
#endif
#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)
const char ChFilterR8AccessName[] = "ChFilterR8Access";              ///< single channel 8 bit array
const char ChFilterRGBA8AccessName[] = "ChFilterRGBA8Access";        ///< 4 channel 8 bit array
const char ChFilterRGBA16AccessName[] = "ChFilterRGBA16Access";      ///< 4 channels of u_int16_t (16 bit) arrays
const char ChFilterRGBDHalf4AccessName[] = "ChFilterRGBDHalf4Access";///< 4 channels of half-compatible 16 bit arrays
const char ChFilterDIAccessName[] = "ChFilterDIAccess";              ///< 2 channel float array (Depth+Intensity)
const char ChFilterRadarAccessName[] = "ChFilterRadarAccess";        ///< Radar return array
const char ChFilterRadarXYZAccessName[] = "ChFilterRadarXYZAccess";  ///< Processed radar XYZ return array
const char ChFilterSemanticAccessName[] = "ChFilterSemanticAccess";  ///< class/instance labels
const char ChFilterDepthAccessName[] = "ChFilterDepthAccess";        ///< single channel depth array
const char ChFilterFloat4AccessName[] = "ChFilterFloat4Access";      ///< 4 channel float array
const char ChFilterNormalAccessName[] = "ChFilterNormalAccess";      ///< 3 channel normal array
#endif
const char ChFilterXYZIAccessName[] = "ChFilterXYZIAccess";      ///< 4 channel float array (XYZ positions+intensity)
const char ChFilterAccelAccessName[] = "ChFilterAccelAccess";    ///< Accelerometer data format (3 doubles total)
const char ChFilterGyroAccessName[] = "ChFilterGyroAccess";      ///< Gyroscope data format (3 doubles total)
const char ChFilterMagnetAccessName[] = "ChFilterMagnetAccess";  ///< Magnetometer data format (3 doubles total)
const char ChFilterGPSAccessName[] = "ChFilterGPSAccess";        ///< GPS data format (4 doubles total)
const char ChFilterTachometerAccessName[] = "ChFilterTachometerAccess";  ///< Tachometer data format
const char ChFilterEncoderAccessName[] = "ChFilterEncoderAccess";        ///< Encoder data format

/// Base class for a Chrono sensor.
class CH_SENSOR_API ChSensor {
  public:
    ///@brief Constructor for the base sensor class.
    ///@param parent Body to which the sensor is attached.
    ///@param updateRate Rate at which the sensor should update.
    ///@param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    //@param lag Lag time between end of data collection and when data becomes available to the user.
    //@param collection_window Collection time over which the sensor should collect data from the simulation.
    ChSensor(std::shared_ptr<ChBody> parent, float updateRate, ChFrame<double> offsetPose);

    virtual ~ChSensor();

    /// @brief Set the sensor's relative position and orientation.
    /// @param pose The relative position and orientation with respect to the parent body
    void SetOffsetPose(ChFrame<double> pose) { m_offsetPose = pose; }

    /// Get the sensor's relative position and orientation.
    /// @return The frame that specifies the offset pose
    ChFrame<double> GetOffsetPose() { return m_offsetPose; }

    /// Get the frame on the parent body against which the sensor offset pose is resolved.
    ///
    /// This is the body reference (REF) frame: the frame visual and collision shapes are placed in.
    /// It coincides with the centroidal frame for a plain ChBody and differs for a ChBodyAuxRef --
    /// which every Chrono::Vehicle chassis is -- where resolving against the centroidal frame would
    /// place a camera and an IMU given the same offset pose at different physical points.
    ///
    /// The returned frame carries valid first and second derivatives: ChBodyAuxRef::Update
    /// recomputes it through ChFrameMoving::TransformLocalToParent, which propagates both.
    /// @return The parent body reference frame expressed in absolute coordinates
    const ChFrameMoving<double>& GetMountingFrame() const { return m_parent->GetFrameRefToAbs(); }

    /// Get the object to which the sensor is attached.
    /// @return A shared pointer to the body on which the sensor is attached
    std::shared_ptr<ChBody> GetParent() const { return m_parent; }

    /// @brief Set the sensor's name.
    /// @param name Name of the sensor -> not used for any internal critical mechanisms
    void SetName(std::string name) { m_name = name; }

    /// Get the name of the sensor.
    /// @return The string name of the sensor
    std::string GetName() const { return m_name; }

    /// Get the sensor update rate (Hz).
    /// @returns The update rate in Hz
    float GetUpdateRate() const { return m_updateRate; }

    /// Set the lag parameter.
    /// @param t The lag time
    void SetLag(float t);

    /// Get the sensor lag (seconds).
    /// @return The lag of the sensor
    float GetLag() const { return m_lag; }

    /// Set the collection window.
    /// @param t The collection time of the sensor
    void SetCollectionWindow(float t);

    /// Get the sensor data collection window (seconds).
    /// @return The duration of simulation time over which the sensor generates its data
    float GetCollectionWindow() const { return m_collection_window; }

    ///@brief Set the sensor update rate (Hz).
    ///@param updateRate Desired update rate in Hz
    void SetUpdateRate(float updateRate) { m_updateRate = updateRate; }

    /// Get the number of times the sensor has been updated.
    /// @return The number of times an update for the sensor has been started
    unsigned int GetNumLaunches() {
        std::lock_guard<std::mutex> lck(m_dataAccess);
        return m_num_launches;
    }

    /// Increments the count of number of updates.
    void IncrementNumLaunches() {
        std::lock_guard<std::mutex> lck(m_dataAccess);
        m_num_launches++;
    }

    /// Get the sensor's list of filters.
    /// @return An immutable list of filters that are used to generate and augment the sensor's data
    std::list<std::shared_ptr<ChFilter>> GetFilterList() const { return m_filters; }

    /// Add a filter to the sensor.
    /// @param filter A filter that should be added to the filter list if the filter list is not yet locked. If the
    /// filter list has been locked (i.e. the sensor has started generating data) the new filter will be ignored.
    void PushFilter(std::shared_ptr<ChFilter> filter);

    /// Add a filter to the front of the list on a sensor.
    /// @param filter A filter that should be added to the filter list if the filter list is not yet locked. If the
    /// filter list has been locked (i.e. the sensor has started generating data) the new filter will be ignored.
    void PushFilterFront(std::shared_ptr<ChFilter> filter);

    /// Gives ability to lock the filter list to prevent race conditions.
    /// This is called automatically when the sensor is added to the ChSensor manager. This is needed to prevent
    /// changing of filters while a worker thread is processing the filters.
    /// WARNING: this operation cannot be undone.
    void LockFilterList() { m_filter_list_locked = true; }

    /// This sensor's registration ordinal within its manager, used to give it RNG streams distinct
    /// from every other sensor's. Assigned by ChSensorManager::AddSensor in registration order.
    ///
    /// Registration order is therefore part of the reproducibility contract: adding or reordering
    /// AddSensor calls changes which random numbers a sensor draws, even under the same fixed seed.
    /// @return The ordinal, or CH_SENSOR_UNASSIGNED_RNG_ID if this sensor was never registered.
    unsigned int GetRngSensorOrdinal() const { return m_rng_sensor_ordinal; }

    /// The id of the manager this sensor is registered with.
    ///
    /// Present because the fixed seed is process-global while ordinals restart at zero in each
    /// manager, so without this the first sensor of two coexisting managers would share a stream.
    /// @return The manager id, or CH_SENSOR_UNASSIGNED_RNG_ID if this sensor was never registered.
    unsigned int GetRngManagerId() const { return m_rng_manager_id; }

    /// Get the last filter in the list that matches the template type
    /// @return A shared pointer to a ChSensorBuffer of the templated type.
    template <class UserBufferType>
    UserBufferType GetMostRecentBuffer();  // explicit specializations exist for each buffer type available

  protected:
    float m_updateRate;  ///< sensor update rate
    float m_lag;         ///< sensor lag from the time all scene information is available (sensor processing time)
    float m_collection_window;  ///< time over which data is collected. (lag+shutter = time from when data collection
                                ///< is started to when it is available to the user)
    float m_timeLastUpdated;    ///< time since previous update
    std::shared_ptr<ChBody> m_parent;                ///< object to which the sensor is attached
    ChFrame<double> m_offsetPose;                    ///< position and orientation of the sensor relative to its parent
    std::string m_name;                              ///< name of the sensor
    unsigned int m_num_launches;                     ///< number of times the sensor has been updated
    std::list<std::shared_ptr<ChFilter>> m_filters;  ///< filter list for post-processing sensor data
    bool m_filter_list_locked = false;  ///< gives ability to lock the filter list to prevent race conditions

  private:
    template <class UserBufferType, class FilterType, const char* FilterName>
    UserBufferType GetMostRecentBufferHelper();  ///< explicit specializations exist for each buffer type available
    std::mutex m_dataAccess;                     ///< data access mutex to prevent data race in the sensor class

    /// Stamp an RNG stream index onto every filter that does not already have one.
    ///
    /// Needed because PushFilter is NOT the only way a filter reaches m_filters: m_filters is
    /// protected, and a derived sensor may fill it directly. ChPhysCameraSensor's constructor does
    /// exactly that for its seven-stage pipeline, one stage of which owns a cuRAND buffer, so relying
    /// on PushFilter alone left that stage with no identity and made it throw when it asked for a
    /// seed. Called by ChSensorManager::AddSensor, which every sensor must pass through, so this
    /// cannot be bypassed the way PushFilter can.
    ///
    /// Indices already assigned by PushFilter are left alone, so attach order is preserved where it
    /// exists; the remainder are numbered in list order, which is deterministic for a given sensor.
    void AssignPendingRngStreamIndices();

    /// RNG stream identity. Private with no public setter, and assigned only by ChSensorManager:
    /// a user able to set these could silently make two sensors draw identical noise, which is the
    /// defect this identity exists to prevent.
    unsigned int m_rng_sensor_ordinal = CH_SENSOR_UNASSIGNED_RNG_ID;
    unsigned int m_rng_manager_id = CH_SENSOR_UNASSIGNED_RNG_ID;

    /// Monotonic counter handed out to filters as they attach, never reused, never renumbered.
    unsigned int m_next_rng_stream_index = 0;

    friend class ChSensorManager;  ///< assigns m_rng_sensor_ordinal and m_rng_manager_id in AddSensor

};  // class ChSensor

/// Keyframe lifecycle of a dynamic sensor, exposed to ChDynamicsManager independently of the
/// keyframe type. See ChKeyFrameStore for the semantics of each operation.
class CH_SENSOR_API ChKeyFrameStoreBase {
  public:
    virtual ~ChKeyFrameStoreBase() {}

    /// Close the window being collected and hold it until release_time.
    virtual void Stash(float window_end, float release_time) = 0;

    /// Whether the oldest held window is due for release at simulation time t.
    virtual bool DueBy(float t) const = 0;

    /// Make the oldest held window the active one and return the time at which it closed.
    virtual float Release() = 0;

    /// Discard the active window.
    virtual void ClearActive() = 0;
};

/// Keyframes of one dynamic sensor: the window currently being collected, plus the closed windows
/// waiting out the sensor lag.
///
/// Holding closed windows rather than filtered buffers is what lets the filter graph run unchanged
/// at release time: the samples it reads are the ones taken during the window, and only the instant
/// at which they reach the user is deferred.
template <typename KeyFrame>
class ChKeyFrameStore : public ChKeyFrameStoreBase {
  public:
    /// The window being collected, or the released window while the filter graph runs over it.
    std::vector<KeyFrame>& Active() { return m_active; }
    const std::vector<KeyFrame>& Active() const { return m_active; }

    virtual void Stash(float window_end, float release_time) override {
        m_pending.push_back({window_end, release_time, std::move(m_active)});
        m_active.clear();
    }

    virtual bool DueBy(float t) const override {
        return !m_pending.empty() && m_pending.front().release_time <= t;
    }

    virtual float Release() override {
        m_active = std::move(m_pending.front().keyframes);
        const float window_end = m_pending.front().window_end;
        m_pending.pop_front();
        return window_end;
    }

    virtual void ClearActive() override { m_active.clear(); }

  private:
    struct Window {
        float window_end;    ///< simulation time at which collection stopped
        float release_time;  ///< window_end plus the sensor lag
        std::vector<KeyFrame> keyframes;
    };

    std::vector<KeyFrame> m_active;
    std::deque<Window> m_pending;
};

/// Base class for sensors whose readings come from the state of the parent body rather than from a
/// rendered image: the accelerometer, gyroscope, magnetometer, GPS, tachometer and encoder.
///
/// One sample is produced per update period. Keyframes are collected over the window
/// [n / update rate, n / update rate + collection window], then held for the sensor lag before the
/// filter graph runs on them, so data becomes visible to the user that long after the window closed.
class CH_SENSOR_API ChDynamicSensor : public ChSensor {
  public:
    virtual ~ChDynamicSensor() {}

    /// Sample the parent body state and append one keyframe to the window being collected.
    virtual void PushKeyFrame() = 0;

    /// The sensor's keyframe store.
    virtual ChKeyFrameStoreBase& KeyFrames() = 0;

    /// Discard the active window.
    void ClearKeyFrames() { KeyFrames().ClearActive(); }

    /// Make the oldest held window active and record the time at which it closed.
    void ReleaseKeyFrames() { m_sample_time = KeyFrames().Release(); }

    /// Get the simulation time at which the window now in the filter graph closed.
    ///
    /// This is the instant the sample describes. With a nonzero lag it precedes the time at which
    /// the data became visible, and it is what the update filters stamp on the output buffer.
    /// @return The sample time in seconds
    float GetSampleTime() const { return m_sample_time; }

    /// Get the number of collection windows closed so far.
    ///
    /// This paces collection and is distinct from GetNumLaunches(), which counts the windows
    /// released to the filter graph and so trails this by the number still waiting out the lag.
    /// @return The number of closed windows
    unsigned int GetNumWindows() const { return m_num_windows; }

    /// Increment the count of closed collection windows.
    void IncrementNumWindows() { m_num_windows++; }

    /// Advance any continuous-time model this sensor holds to the given simulation time.
    ///
    /// Called on every simulation step, whether or not the sensor is collecting, because a
    /// bandwidth model is a continuous-time filter and a signal gapped outside the collection
    /// window is not the one it is meant to see. Sensors without such a model do nothing here.
    /// @param time The current simulation time in seconds
    void AdvanceTo(double time) {
        // The first call only establishes the time origin: there is no interval to advance over
        // yet, and measuring one from a default-constructed zero would hand the model the whole
        // elapsed simulation in a single step.
        if (!m_continuous_started) {
            m_continuous_started = true;
            m_last_continuous_time = time;
            return;
        }
        const double dt = time - m_last_continuous_time;
        m_last_continuous_time = time;
        if (dt > 0)
            AdvanceContinuousModel(dt);
    }

  protected:
    ChDynamicSensor(std::shared_ptr<ChBody> parent, float updateRate, ChFrame<double> offsetPose)
        : ChSensor(parent, updateRate, offsetPose), m_sample_time(0), m_num_windows(m_num_launches) {}

    /// Advance this sensor's continuous-time model, if it has one, by dt seconds.
    /// @param dt Elapsed simulation time in seconds, always positive
    virtual void AdvanceContinuousModel(double dt) {}

    float m_sample_time;         ///< time at which the window in the filter graph closed
    unsigned int m_num_windows;  ///< number of collection windows closed so far

  private:
    double m_last_continuous_time = 0;  ///< simulation time of the last AdvanceTo call
    bool m_continuous_started = false;  ///< whether m_last_continuous_time holds a real time yet
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
