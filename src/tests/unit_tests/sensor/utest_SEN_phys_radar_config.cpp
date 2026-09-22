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
// Wave-domain radar configuration: derived quantities against the closed-form waveform
// relations, validation of inconsistent configurations, and the shipped archetype files.
//
// =============================================================================

#include <cmath>
#include <fstream>
#include <stdexcept>

#include "gtest/gtest.h"

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/sensors/radar/ChRadarConfig.h"
#include "chrono_sensor/sensors/radar/ChRadarMaterialRegistry.h"

using namespace chrono;
using namespace chrono::sensor;

namespace {

constexpr double c_light = 299792458.0;

std::string RadarDataFile(const std::string& name) {
    return GetChronoDataFile("sensor/radar/" + name);
}

}  // namespace

// The waveform relations of section 7.3 of the radar design notes: range resolution from the
// bandwidth actually swept while the converter is running, velocity resolution from the length of
// the coherent processing interval, and the folding velocity from the chirp repetition interval.
TEST(PhysRadarConfig, DerivedQuantitiesMatchWaveformRelations) {
    ChRadarModelConfig cfg = MakeDefaultFrontRadarConfig();
    ASSERT_NO_THROW(cfg.Validate());

    const double wavelength = c_light / cfg.waveform.carrier_frequency;
    EXPECT_NEAR(cfg.GetWavelength(), wavelength, 1e-12);

    const double slope = cfg.waveform.bandwidth / cfg.waveform.chirp_duration;
    const double effective_bandwidth = slope * cfg.waveform.num_samples / cfg.waveform.sample_rate;
    EXPECT_NEAR(cfg.GetEffectiveBandwidth(), effective_bandwidth, 1e-3);
    EXPECT_NEAR(cfg.GetRangeResolution(), c_light / (2.0 * effective_bandwidth), 1e-9);

    const double pri = cfg.GetEffectivePri();
    EXPECT_NEAR(cfg.GetVelocityResolution(), wavelength / (2.0 * cfg.GetNumDopplerBins() * pri), 1e-12);
    EXPECT_NEAR(cfg.GetMaxUnambiguousVelocity(), wavelength / (4.0 * pri), 1e-12);

    // The Doppler axis spans exactly the unambiguous interval.
    EXPECT_NEAR(cfg.GetNumDopplerBins() * cfg.GetVelocityResolution(), 2.0 * cfg.GetMaxUnambiguousVelocity(), 1e-9);

    // Only the bins inside the configured span are formed, and they cover it.
    EXPECT_LE(cfg.GetNumRangeBins(), cfg.waveform.num_samples);
    EXPECT_GE(cfg.GetNumRangeBins() * cfg.GetRangeResolution(), cfg.max_range);
}

// Time division multiplexing hands each transmitter a fraction of the chirps, which multiplies the
// aperture and divides the velocity the radar can measure without folding.
TEST(PhysRadarConfig, TimeDivisionMultiplexingTradesVelocityForAperture) {
    ChRadarModelConfig single = MakeDefaultFrontRadarConfig();
    ChRadarModelConfig mimo = single;

    mimo.antenna.transmitters.resize(3, single.antenna.transmitters.front());
    mimo.antenna.transmitters[0].position = ChVector3d(0, -0.0078377, 0);
    mimo.antenna.transmitters[1].position = ChVector3d(0, 0.0, 0);
    mimo.antenna.transmitters[2].position = ChVector3d(0, 0.0078377, 0);
    mimo.antenna.multiplex = ChRadarMultiplex::TDM;
    mimo.waveform.num_chirps = 384;
    ASSERT_NO_THROW(mimo.Validate());

    EXPECT_EQ(mimo.GetNumVirtualChannels(), 3 * single.GetNumVirtualChannels());
    EXPECT_EQ(mimo.GetNumDopplerBins(), 128u);
    EXPECT_NEAR(mimo.GetEffectivePri(), 3.0 * mimo.waveform.chirp_repetition_interval, 1e-12);
    EXPECT_NEAR(mimo.GetMaxUnambiguousVelocity(), single.GetMaxUnambiguousVelocity() / 3.0, 1e-9);
}

TEST(PhysRadarConfig, ValidationRejectsInconsistentWaveforms) {
    ChRadarModelConfig cfg = MakeDefaultFrontRadarConfig();

    // Digitizing for longer than the sweep lasts.
    ChRadarModelConfig too_many_samples = cfg;
    too_many_samples.waveform.num_samples = 4096;
    EXPECT_THROW(too_many_samples.Validate(), std::invalid_argument);

    // Asking for a range the waveform cannot resolve without aliasing.
    ChRadarModelConfig too_far = cfg;
    too_far.max_range = 10.0 * cfg.GetMaxUnambiguousRange();
    EXPECT_THROW(too_far.Validate(), std::invalid_argument);

    // Several transmitters with no multiplexing scheme to separate them.
    ChRadarModelConfig unseparated = cfg;
    unseparated.antenna.transmitters.resize(2, cfg.antenna.transmitters.front());
    EXPECT_THROW(unseparated.Validate(), std::invalid_argument);

    // Chirps that do not divide evenly among the transmitters.
    ChRadarModelConfig uneven = cfg;
    uneven.antenna.transmitters.resize(3, cfg.antenna.transmitters.front());
    uneven.antenna.multiplex = ChRadarMultiplex::TDM;
    uneven.waveform.num_chirps = 256;
    EXPECT_THROW(uneven.Validate(), std::invalid_argument);
}

TEST(PhysRadarConfig, JsonRoundTripPreservesTheModel) {
    const ChRadarModelConfig original = MakeDefaultFrontRadarConfig();
    const std::string path = "utest_SEN_phys_radar_config.json";
    ASSERT_NO_THROW(original.WriteJSON(path));

    ChRadarModelConfig reloaded;
    ASSERT_NO_THROW(reloaded = ChRadarModelConfig::ReadJSON(path));

    EXPECT_EQ(reloaded.name, original.name);
    EXPECT_NEAR(reloaded.field_of_view_azimuth, original.field_of_view_azimuth, 1e-9);
    EXPECT_NEAR(reloaded.max_range, original.max_range, 1e-9);
    EXPECT_EQ(reloaded.waveform.num_chirps, original.waveform.num_chirps);
    EXPECT_NEAR(reloaded.waveform.bandwidth, original.waveform.bandwidth, 1e-3);
    EXPECT_EQ(reloaded.antenna.receivers.size(), original.antenna.receivers.size());
    EXPECT_NEAR(reloaded.antenna.receivers[2].position.y(), original.antenna.receivers[2].position.y(), 1e-12);
    EXPECT_EQ(reloaded.dsp.num_azimuth_bins, original.dsp.num_azimuth_bins);
    EXPECT_NEAR(reloaded.GetRangeResolution(), original.GetRangeResolution(), 1e-9);

    std::remove(path.c_str());
}

TEST(PhysRadarConfig, ShippedArchetypesAreValid) {
    for (const char* name : {"ars408_like_far.json", "ars408_like_near.json", "mimo_tdm_like.json"}) {
        const std::string path = RadarDataFile(name);
        std::ifstream probe(path);
        ASSERT_TRUE(probe.good()) << "missing shipped radar configuration " << path;
        probe.close();

        ChRadarModelConfig cfg;
        ASSERT_NO_THROW(cfg = ChRadarModelConfig::ReadJSON(path)) << name;
        EXPECT_GT(cfg.GetNumRangeBins(), 0u) << name;
        EXPECT_GT(cfg.GetNumDopplerBins(), 0u) << name;
        EXPECT_GT(cfg.GetNoisePowerPerCell(), 0.0) << name;
        EXPECT_GT(cfg.GetRaySolidAngle(), 0.0) << name;
    }

    // The long range archetype resolves range coarsely and reaches far; the near one is the other
    // way round. A file that swapped them would still validate, so the distinction is asserted.
    const ChRadarModelConfig far_config = ChRadarModelConfig::ReadJSON(RadarDataFile("ars408_like_far.json"));
    const ChRadarModelConfig near_config = ChRadarModelConfig::ReadJSON(RadarDataFile("ars408_like_near.json"));
    EXPECT_GT(far_config.max_range, near_config.max_range);
    EXPECT_GT(far_config.GetRangeResolution(), near_config.GetRangeResolution());
    EXPECT_LT(far_config.field_of_view_azimuth, near_config.field_of_view_azimuth);

    const ChRadarModelConfig mimo = ChRadarModelConfig::ReadJSON(RadarDataFile("mimo_tdm_like.json"));
    EXPECT_EQ(mimo.antenna.multiplex, ChRadarMultiplex::TDM);
    EXPECT_EQ(mimo.GetNumVirtualChannels(), 12u);
}

TEST(PhysRadarMaterials, RegistryTableIsDenseAndFlagsAssignment) {
    ChRadarMaterialRegistry registry;

    // With nothing assigned the shader is told to derive a response from the visual material.
    EXPECT_EQ(registry.GetFallback().assigned, 0);
    EXPECT_EQ(registry.BuildTable().size(), 1u);

    registry.Assign(4, "metal");
    registry.Assign(1, "asphalt");

    const std::vector<RadarMaterial> table = registry.BuildTable();
    ASSERT_EQ(table.size(), 5u);
    EXPECT_EQ(table[0].assigned, 0);
    EXPECT_EQ(table[2].assigned, 0);
    EXPECT_EQ(table[3].assigned, 0);
    EXPECT_EQ(table[1].assigned, 1);
    EXPECT_EQ(table[4].assigned, 1);

    // Metal keeps almost all of its power in the coherent lobe; asphalt scatters it.
    EXPECT_GT(table[4].specular_reflectivity, table[1].specular_reflectivity);
    EXPECT_GT(table[1].diffuse_reflectivity, table[4].diffuse_reflectivity);
    EXPECT_GT(table[1].lobe_width, table[4].lobe_width);

    EXPECT_THROW(ChRadarMaterialRegistry::GetSample("unobtainium"), std::invalid_argument);
    EXPECT_FALSE(ChRadarMaterialRegistry::GetSampleNames().empty());
}

// The pattern exponent is defined by its half-power point, so the gain there must be half.
TEST(PhysRadarConfig, AntennaPatternExponentMatchesBeamwidth) {
    for (double beamwidth_deg : {10.0, 30.0, 60.0, 90.0}) {
        const double beamwidth = beamwidth_deg * CH_DEG_TO_RAD;
        const double exponent = ChRadarPatternExponent(beamwidth);
        EXPECT_GT(exponent, 0.0);
        EXPECT_NEAR(std::pow(std::cos(0.5 * beamwidth), exponent), 0.5, 1e-9) << beamwidth_deg;
    }
    EXPECT_EQ(ChRadarPatternExponent(0.0), 0.0);
}
