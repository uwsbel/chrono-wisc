// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// Utilities for performance benchmarking of Chrono FSI simulations using the Google
// benchmark framework.
//
// =============================================================================

#ifndef CH_BENCHMARK_H
#define CH_BENCHMARK_H
#include <iomanip>
#include "chrono_thirdparty/googlebenchmark/include/benchmark/benchmark.h"
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsTimingOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace fsi {

/// @addtogroup chrono_fsi
/// @{

/// Base class for a Chrono FSI benchmark test.
/// A derived class should set up a complete Chrono FSI model in its constructor and implement
/// GetSystem (to return a pointer to the underlying Chrono system) and ExecuteStep (to perform
/// all operations required to advance the system state by one time step).
/// Timing information for various phases of the simulation is collected for a sequence of steps.
class ChBenchmarkTest {
  public:
    ChBenchmarkTest();
    virtual ~ChBenchmarkTest() {}

    virtual void ExecuteStep() = 0;
    virtual ChSystemFsi* GetSystem() = 0;
    virtual ChSystem* GetMultibodySystem() = 0;
    virtual double GetStepSize() = 0;
    virtual std::string GetViscosityTypeString() = 0;
    virtual std::string GetBoundaryTypeString() = 0;
    virtual int GetNumProximitySearchSteps() = 0;
    virtual double GetD0Multiplier() = 0;
    virtual double GetTEnd() = 0;
    virtual std::string GetTestName() = 0;
    void Simulate(int num_steps);
    void ResetTimers();

    double m_timer_step;                  ///< time for both CFD and MBS
    double m_timer_CFD;                   ///< time for CFD
    double m_timer_MBS;                   ///< time for MBS
    double m_timer_FSI;                   ///< time for FSI data exchange
    double m_timer_integrateSPH;          ///< time for SPH integration
    double m_timer_rigidForces;           ///< time for rigid body forces
    double m_timer_flex1DForces;          ///< time for 1D flexible forces
    double m_timer_flex2DForces;          ///< time for 2D flexible forces
    double m_timer_copySortedToOriginal;  ///< time for copying sorted to original
    double m_timer_sortParticles;         ///< time for sorting particles
    double m_timer_force;                 ///< time for force calculation
    double m_timer_updateFluid;           ///< time for fluid update
    double m_timer_periodicBoundary;      ///< time for periodic boundary
    double m_timer_boundaryCondition;     ///< time for boundary condition
    double m_timer_accelerationCalc;      ///< time for acceleration calculation

    // MBS specific timers
    double m_timer_collision;         ///< time for collision detection
    double m_timer_collision_broad;   ///< time for broad-phase collision
    double m_timer_collision_narrow;  ///< time for narrow-phase collision
    double m_timer_setup;             ///< time for system update
    double m_timer_update;            ///< time for system update
};

inline ChBenchmarkTest::ChBenchmarkTest()
    : m_timer_step(0),
      m_timer_CFD(0),
      m_timer_MBS(0),
      m_timer_FSI(0),
      m_timer_integrateSPH(0),
      m_timer_rigidForces(0),
      m_timer_flex1DForces(0),
      m_timer_flex2DForces(0),
      m_timer_copySortedToOriginal(0),
      m_timer_sortParticles(0),
      m_timer_force(0),
      m_timer_updateFluid(0),
      m_timer_periodicBoundary(0),
      m_timer_boundaryCondition(0),
      m_timer_accelerationCalc(0),
      m_timer_collision(0),
      m_timer_collision_broad(0),
      m_timer_collision_narrow(0),
      m_timer_setup(0),
      m_timer_update(0) {}

inline void ChBenchmarkTest::Simulate(int num_steps) {
    ////std::cout << "  simulate from t=" << GetSystem()->GetChTime() << " for steps=" << num_steps << std::endl;
    ResetTimers();
    for (int i = 0; i < num_steps; i++) {
        ExecuteStep();
        m_timer_step += GetSystem()->GetTimerStep();
        m_timer_CFD += GetSystem()->GetTimerCFD();
        m_timer_MBS += GetSystem()->GetTimerMBS();
        m_timer_FSI += GetSystem()->GetTimerFSI();
        m_timer_integrateSPH += GetSystem()->GetTimerCFD();
        // These come out as cummulative times
        m_timer_rigidForces = GetSystem()->GetTimeRigidForces();
        m_timer_flex1DForces = GetSystem()->GetTimeFlex1DForces();
        m_timer_flex2DForces = GetSystem()->GetTimeFlex2DForces();
        m_timer_copySortedToOriginal = GetSystem()->GetTimeCopySortedToOriginal();
        m_timer_sortParticles = GetSystem()->GetTimeSortParticles();
        m_timer_force = GetSystem()->GetTimeForce();
        m_timer_updateFluid = GetSystem()->GetTimeUpdateFluid();
        m_timer_periodicBoundary = GetSystem()->GetTimePeriodicBoundary();
        m_timer_boundaryCondition = GetSystem()->GetTimeBoundaryCondition();
        m_timer_accelerationCalc = GetSystem()->GetTimeAccelerationCalc();

        m_timer_collision += GetMultibodySystem()->GetTimerCollision();
        m_timer_collision_broad += GetMultibodySystem()->GetTimerCollisionBroad();
        m_timer_collision_narrow += GetMultibodySystem()->GetTimerCollisionNarrow();
        m_timer_setup += GetMultibodySystem()->GetTimerSetup();
        m_timer_update += GetMultibodySystem()->GetTimerUpdate();
    }
}

inline void ChBenchmarkTest::ResetTimers() {
    m_timer_step = 0;
    m_timer_CFD = 0;
    m_timer_MBS = 0;
    m_timer_FSI = 0;
    m_timer_integrateSPH = 0;
    m_timer_rigidForces = 0;
    m_timer_flex1DForces = 0;
    m_timer_flex2DForces = 0;
    m_timer_copySortedToOriginal = 0;
    m_timer_sortParticles = 0;
    m_timer_force = 0;
    m_timer_updateFluid = 0;
    m_timer_periodicBoundary = 0;
    m_timer_boundaryCondition = 0;
    m_timer_accelerationCalc = 0;
    m_timer_collision = 0;
    m_timer_collision_broad = 0;
    m_timer_collision_narrow = 0;
    m_timer_setup = 0;
    m_timer_update = 0;
}

// =============================================================================

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// This method benchmarks consecutive (in time) simulation batches and is therefore
/// appropriate for cases where the cost per step is expected to be relatively uniform.
/// An initial SKIP_STEPS integration steps are performed for hot start, after which
/// measurements are conducted for batches of SIM_STEPS integration steps.
/// The test is repeated REPETITIONS number of times, to collect statistics.
/// Note that each reported benchmark result may require simulations of several batches
/// (controlled by the benchmark library in order to stabilize timing results).
#define CH_BM_SIMULATION_LOOP(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::fsi::ChBenchmarkFixture<TEST, SKIP_STEPS>;           \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateLoop)(benchmark::State & st) {           \
        while (st.KeepRunning()) {                                                 \
            m_test->Simulate(SIM_STEPS);                                           \
        }                                                                          \
        Report(st);                                                                \
    }                                                                              \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateLoop)->Unit(benchmark::kMillisecond)->Repetitions(REPETITIONS);

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// This method benchmarks a single simulation interval and is appropriate for cases
/// where the cost of simulating a given length time interval can vary significantly
/// from interval to interval.
/// For each measurement, the underlying model is recreated from scratch. An initial
/// SKIP_STEPS integration steps are performed for hot start, after which a single
/// batch of SIM_STEPS is timed and recorded.
/// The test is repeated REPETITIONS number of times, to collect statistics.
#define CH_BM_SIMULATION_ONCE(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::fsi::ChBenchmarkFixture<TEST, 0>;                    \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateOnce)(benchmark::State & st) {           \
        Reset(SKIP_STEPS);                                                         \
        while (st.KeepRunning()) {                                                 \
            m_test->Simulate(SIM_STEPS);                                           \
        }                                                                          \
        st.SetLabel(#TEST_NAME); /* Add this line to store the test name */        \
        Report(st);                                                                \
    }                                                                              \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateOnce)->Unit(benchmark::kSecond)->Iterations(1)->Repetitions(REPETITIONS);

// =============================================================================

/// Generic benchmark fixture for Chrono tests.
/// The first template parameter is a ChBenchmarkTest.
/// The second template parameter is the initial number of simulation steps (hot start).
template <typename TEST, int SKIP>
class ChBenchmarkFixture : public ::benchmark::Fixture {
  public:
    ChBenchmarkFixture() : m_test(nullptr) {
        ////std::cout << "CREATE TEST" << std::endl;
        if (SKIP != 0) {
            m_test = new TEST();
            m_test->Simulate(SKIP);
        }
    }

    ~ChBenchmarkFixture() { delete m_test; }

    void Report(benchmark::State& st) {
        st.counters["StepTime"] = m_test->m_timer_step;
        st.counters["CFDTime"] = m_test->m_timer_CFD;
        st.counters["MBSTime"] = m_test->m_timer_MBS;
        st.counters["FSIExchangeTime"] = m_test->m_timer_FSI;
        st.counters["integrateSPHTime"] = m_test->m_timer_integrateSPH;
        st.counters["rigidForcesTime"] = m_test->m_timer_rigidForces;
        st.counters["flex1DForcesTime"] = m_test->m_timer_flex1DForces;
        st.counters["flex2DForcesTime"] = m_test->m_timer_flex2DForces;
        st.counters["copySortedToOriginalTime"] = m_test->m_timer_copySortedToOriginal;
        st.counters["sortParticlesTime"] = m_test->m_timer_sortParticles;
        st.counters["forceTime"] = m_test->m_timer_force;
        st.counters["updateFluidTime"] = m_test->m_timer_updateFluid;
        st.counters["periodicBoundaryTime"] = m_test->m_timer_periodicBoundary;
        st.counters["boundaryConditionTime"] = m_test->m_timer_boundaryCondition;
        st.counters["accelerationCalcTime"] = m_test->m_timer_accelerationCalc;

        SetChronoOutputPath("BENCHMARK_BASELINE_RTF/");
        // Output directories
        std::string out_dir;
        out_dir = GetChronoOutputPath() + m_test->GetTestName();
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return;
        }

        out_dir = out_dir + "/CRM_WCSPH/";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return;
        }
        // Create JSON output using existing utilities
        rapidjson::Document doc;
        // Format d0_multiplier
        std::ostringstream d0_str;
        d0_str << std::fixed << std::setprecision(1) << m_test->GetD0Multiplier();
        std::string d0_formatted = d0_str.str();
        d0_formatted.erase(d0_formatted.find_last_not_of('0') + 1, std::string::npos);
        if (d0_formatted.back() == '.')
            d0_formatted.pop_back();

        // Format scale
        std::string test_name = st.name();  // This will give you "FSI_RigidBceScaling_1/SimulateOnce"

        // Extract the number from the test name
        size_t underscore_pos = test_name.find_last_of('_');
        size_t slash_pos = test_name.find('/');
        std::string num_str = test_name.substr(underscore_pos + 1, slash_pos - underscore_pos - 1);

        std::string json_file_path =
            out_dir + "/rtf_default_default_ps1_d0" + d0_formatted + "_scale" + num_str + ".json";

        // First output parameters
        OutputParameterJSON(json_file_path,
                            m_test->GetSystem(),        // ChFsiSystem*
                            m_test->GetTEnd(),          // t_end (number of iterations)
                            m_test->GetStepSize(),      // step_size
                            "default",                  // viscosity_type
                            "default",                  // boundary_type
                            1,                          // ps_freq
                            m_test->GetD0Multiplier(),  // d0_multiplier
                            doc);

        // Then add timing information
        OutputTimingJSON(json_file_path,
                         m_test->m_timer_step,  // timer_step
                         m_test->m_timer_CFD,   // timer_CFD
                         m_test->m_timer_MBS,   // timer_MBS
                         m_test->m_timer_FSI,   // timer_FSI
                         m_test->GetSystem(),   // ChFsiSystem*
                         doc);
    }

    void Reset(int num_init_steps) {
        ////std::cout << "RESET" << std::endl;
        delete m_test;
        m_test = new TEST();
        m_test->Simulate(num_init_steps);
    }

    TEST* m_test;
};

/// @} chrono_utils

}  // end namespace fsi
}  // end namespace chrono
#endif
