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
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// Manager class for multiple tire test rigs.
// - Manages multiple ChTireTestRig instances in a single simulation
// - Provides batch configuration and individual rig control
// - Allows power users to access individual rigs and their components
//
// =============================================================================

#ifndef CH_MANY_TIRE_TEST_RIGS_H
#define CH_MANY_TIRE_TEST_RIGS_H

#include <memory>
#include <vector>
#include <functional>

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Manager for multiple tire test rigs operating in a single simulation.
/// For now it only supports rigid terrain.
class CH_VEHICLE_API ChManyTireTestRigs {
  public:
    /// Configuration structure for batch rig setup.
    /// This structure holds common parameters that can be applied to multiple rigs.
    struct RigConfiguration {
        double normal_load = 1000.0;                                                 ///< Normal load on tire (N)
        double camber_angle = 0.0;                                                   ///< Camber angle (rad)
        double time_delay = 0.0;                                                     ///< Time delay before motion (s)
        double tire_step = 1e-3;                                                     ///< Tire step size (s)
        std::shared_ptr<ChFunction> long_speed_function;                             ///< Longitudinal speed function
        std::shared_ptr<ChFunction> ang_speed_function;                              ///< Angular speed function
        std::shared_ptr<ChFunction> slip_angle_function;                             ///< Slip angle function
        ChTire::CollisionType collision_type = ChTire::CollisionType::SINGLE_POINT;  ///< Tire collision type
        VisualizationType tire_visualization = VisualizationType::PRIMITIVES;        ///< Tire visualization type

        ChTireTestRig::TerrainParamsRigid terrain_params_rigid;

        /// Apply this configuration to a rig
        void ApplyTo(ChTireTestRig& rig) const;
    };

    /// Construct a manager for multiple tire test rigs.
    /// @param system Pointer to the Chrono system that will contain all rigs
    ChManyTireTestRigs(ChSystem* system);

    ~ChManyTireTestRigs() = default;

    // ========================================================
    // Rig Management
    // ========================================================

    /// Add a single rig with specified wheel and tire.
    /// @param wheel Wheel subsystem for this rig
    /// @param tire Tire subsystem for this rig
    /// @return Reference to the newly created rig for direct configuration
    ChTireTestRig& AddRig(std::string wheel_json, std::string tire_json, bool use_airless_tire = false);

    /// Add multiple identical rigs at once.
    /// All rigs will use a copy of the wheel and tire objects passed here.
    /// @param wheel Wheel subsystem template
    /// @param tire Tire subsystem template
    /// @param num_rigs Number of identical rigs to create
    void AddManyRigs(std::string wheel_json, std::string tire_json, bool use_airless_tire = false, size_t num_rigs = 1);

    /// Add multiple rigs with a common configuration applied.
    /// @param wheel Wheel subsystem template
    /// @param tire Tire subsystem template
    /// @param num_rigs Number of rigs to create
    /// @param config Configuration to apply to all new rigs
    void AddManyRigs(std::string wheel_json,
                     std::string tire_json,
                     bool use_airless_tire,
                     size_t num_rigs,
                     const RigConfiguration& config);

    /// Get the number of rigs currently managed.
    size_t GetNumRigs() const { return m_rigs.size(); }

    /// Get direct access to a specific rig for advanced configuration.
    /// @param index Index of the rig (0-based)
    /// @return Reference to the ChTireTestRig at the specified index
    ChTireTestRig& GetRig(size_t index);

    /// Get const access to a specific rig.
    const ChTireTestRig& GetRig(size_t index) const;

    /// Get pointer to the underlying Chrono system.
    ChSystem* GetSystem() const { return m_system; }

    // ========================================================
    // Batch Configuration Methods
    // ========================================================

    /// Apply a configuration to all rigs.
    void ApplyConfigurationToAll(const RigConfiguration& config);

    /// Apply a configuration to a range of rigs.
    void ApplyConfigurationToRange(const RigConfiguration& config, size_t start_index, size_t end_index);

    /// Apply a custom function to each rig
    /// @param func Function or lambda that takes a ChTireTestRig reference
    template <typename Func>
    void ForEachRig(Func&& func) {
        for (auto& rig : m_rigs) {
            func(*rig);
        }
    }

    /// Apply a custom function to each rig with index (for custom logic based on rig number).
    template <typename Func>
    void ForEachRigIndexed(Func&& func) {
        for (size_t i = 0; i < m_rigs.size(); ++i) {
            func(*m_rigs[i], i);
        }
    }

    // ========================================================
    // Common Batch Setters (Convenience Methods)
    // ========================================================

    /// Set gravitational acceleration for all rigs.
    void SetGravitationalAcceleration(const ChVector3d& grav);

    /// Set gravitational acceleration for all rigs (scalar Z value).
    void SetGravitationalAcceleration(double grav);

    /// Set normal load for all rigs.
    void SetNormalLoadAll(double load);

    /// Set camber angle for all rigs.
    void SetCamberAngleAll(double camber);

    /// Set longitudinal speed function for all rigs.
    void SetLongSpeedFunctionAll(std::shared_ptr<ChFunction> funct);

    /// Set angular speed function for all rigs.
    void SetAngSpeedFunctionAll(std::shared_ptr<ChFunction> funct);

    /// Set slip angle function for all rigs.
    void SetSlipAngleFunctionAll(std::shared_ptr<ChFunction> funct);

    /// Set tire collision type for all rigs.
    void SetTireCollisionTypeAll(ChTire::CollisionType coll_type);

    /// Set tire step size for all rigs.
    void SetTireStepsizeAll(double step);

    /// Set tire visualization type for all rigs.
    void SetTireVisualizationTypeAll(VisualizationType vis);

    /// Set rigid terrain parameters for all rigs.
    void SetTerrainRigidAll(const ChTireTestRig::TerrainParamsRigid& params);

    /// Set time delay for all rigs.
    void SetTimeDelayAll(double delay);

    // ========================================================
    // Smart Getters (Power User Access)
    // ========================================================

    /// Get terrain from a specific rig.
    std::shared_ptr<ChTerrain> GetTerrain(size_t index) const;

    /// Get carrier body position from a specific rig.
    ChVector3d GetCarrierPos(size_t index) const;

    /// Get total mass of a specific rig.
    double GetMass(size_t index) const;

    /// Collect a parameter from all rigs into a vector (e.g., all masses, all positions).
    /// Example: auto all_masses = CollectFromAll<double>([](const ChTireTestRig& r) { return r.GetMass(); });
    template <typename T, typename Getter>
    std::vector<T> CollectFromAll(Getter&& getter) const {
        std::vector<T> results;
        results.reserve(m_rigs.size());
        for (const auto& rig : m_rigs) {
            results.push_back(getter(*rig));
        }
        return results;
    }

    // ========================================================
    // Simulation Control
    // ========================================================

    /// Initialize all rigs with the specified mode.
    void InitializeAll(ChTireTestRig::Mode mode);

    /// Advance all rigs by the specified time step.
    void AdvanceAll(double step);

  private:
    ChSystem* m_system;                                  ///< Pointer to the Chrono system
    std::vector<std::unique_ptr<ChTireTestRig>> m_rigs;  ///< Vector of managed test rigs
    ChVector3d m_grav = ChVector3d(0, 0, -9.81);         ///< Gravitational acceleration
    ChVector3d m_last_rig_offset = ChVector3d(0, 0, 0);  ///< Offset from the origin for the last added rig
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
