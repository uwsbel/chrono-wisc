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
// Configuration of a wave-domain radar: waveform, antenna array, radio-frequency
// impairments, signal processing and ray budget. One engine, many radar models.
//
// =============================================================================

#ifndef CHRADARCONFIG_H
#define CHRADARCONFIG_H

#include <string>
#include <vector>

#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Waveform family modeled by the radar.
enum class ChRadarWaveformType {
    FMCW_FAST_CHIRP  ///< linear frequency modulated fast-chirp sequence, 2D range-Doppler formed
};

/// How several transmit elements share one coherent processing interval.
enum class ChRadarMultiplex {
    NONE,  ///< a single transmit element, or several radiating together as one
    TDM    ///< time division: transmitters take consecutive chirps in turn
};

/// Taper applied before the range and Doppler transforms.
enum class ChRadarWindowType { RECTANGULAR, HANN, HAMMING };

/// Constant false alarm rate detector variant.
enum class ChRadarCfarType {
    CA,  ///< cell averaging: mean of the training cells
    OS   ///< ordered statistic: a rank of the sorted training cells, robust to interferers
};

/// A transmit or receive antenna element.
struct CH_SENSOR_API ChRadarElementConfig {
    ChVector3d position = {0, 0, 0};  ///< phase centre in the sensor frame [m]
    double gain_dbi = 12.0;           ///< peak gain [dBi]
    double azimuth_beamwidth = 0.0;   ///< 3 dB azimuth beamwidth [rad]; 0 for isotropic
    double elevation_beamwidth = 0.0; ///< 3 dB elevation beamwidth [rad]; 0 for isotropic
};

/// Transmitted waveform.
///
/// The digitized part of the sweep is num_samples / sample_rate seconds long, which need not be
/// the whole chirp; range resolution follows from the bandwidth actually swept in that window.
struct CH_SENSOR_API ChRadarWaveformConfig {
    ChRadarWaveformType type = ChRadarWaveformType::FMCW_FAST_CHIRP;
    double carrier_frequency = 76.5e9;          ///< [Hz]
    double bandwidth = 8.4e7;                   ///< swept bandwidth [Hz]
    double chirp_duration = 6.0e-5;             ///< time to sweep the bandwidth [s]
    double chirp_repetition_interval = 6.5e-5;  ///< chirp start to chirp start [s]
    unsigned int num_chirps = 256;              ///< chirps per coherent processing interval
    unsigned int num_samples = 512;             ///< samples digitized per chirp
    double sample_rate = 1.0e7;                 ///< [Hz]
    double transmit_power_dbm = 12.0;           ///< per transmit element [dBm]
};

/// Antenna array and transmit multiplexing.
struct CH_SENSOR_API ChRadarAntennaConfig {
    std::vector<ChRadarElementConfig> transmitters;
    std::vector<ChRadarElementConfig> receivers;
    ChRadarMultiplex multiplex = ChRadarMultiplex::NONE;
};

/// Receiver noise and radio-frequency impairments.
///
/// Every entry here is a mechanism, not an output-stage error model: the phase noise skirt
/// masks weak neighbours of a strong return, the leakage line desensitizes near range, and the
/// converter resolution sets the floor a large signal lifts the rest of the spectrum to.
struct CH_SENSOR_API ChRadarImpairmentConfig {
    double noise_figure_db = 13.0;            ///< receiver noise figure [dB]
    double system_temperature = 290.0;        ///< reference temperature [K]
    double phase_noise_dbc_hz = -90.0;        ///< single sideband phase noise at the reference offset
    double phase_noise_offset = 1.0e6;        ///< offset the figure above is quoted at [Hz]
    double phase_noise_decade_slope = -20.0;  ///< skirt roll-off [dB per decade of offset]
    double transmit_leakage_db = -45.0;       ///< transmitter to receiver coupling [dB]
    unsigned int adc_bits = 14;               ///< converter resolution, sets the quantization floor
    bool enable_thermal_noise = true;
    bool enable_phase_noise = true;
    bool enable_transmit_leakage = true;
    bool enable_quantization = true;
};

/// Detection chain: beamforming, thresholding and peak extraction.
struct CH_SENSOR_API ChRadarDspConfig {
    ChRadarWindowType range_window = ChRadarWindowType::HANN;
    ChRadarWindowType doppler_window = ChRadarWindowType::HANN;
    unsigned int num_azimuth_bins = 64;  ///< beams formed across the virtual array

    ChRadarCfarType cfar_type = ChRadarCfarType::CA;
    unsigned int cfar_guard_range = 2;
    unsigned int cfar_guard_doppler = 2;
    unsigned int cfar_train_range = 8;
    unsigned int cfar_train_doppler = 4;
    double false_alarm_rate = 1e-4;
    double os_rank_fraction = 0.75;  ///< rank used by the ordered statistic detector, 0..1

    bool suppress_stationary = false;   ///< drop returns whose radial velocity cancels ego motion
    double stationary_tolerance = 0.75; ///< half-width of the rejected clutter ridge [m/s]
    unsigned int max_detections = 4096;
};

/// Track formation from detections.
struct CH_SENSOR_API ChRadarTrackerConfig {
    unsigned int confirm_hits = 3;    ///< hits needed inside the confirmation window
    unsigned int confirm_window = 5;  ///< cycles the confirmation is counted over
    unsigned int coast_cycles = 3;    ///< cycles a track survives without an update
    double gate_distance = 4.0;       ///< association gate radius [m]
    double process_noise = 6.0;       ///< acceleration process noise [m/s^2]
    double measurement_noise = 0.3;   ///< position measurement noise [m]
};

/// Ray budget of the propagation stage.
struct CH_SENSOR_API ChRadarRayTracingConfig {
    unsigned int rays_azimuth = 512;      ///< launched rays across the azimuth field of view
    unsigned int rays_elevation = 96;     ///< launched rays across the elevation field of view
    unsigned int max_bounces = 3;         ///< surface interactions before a walk is terminated
    unsigned int max_paths = 1u << 21;   ///< path buffer capacity
    double amplitude_cutoff_db = -60.0;  ///< paths this far below the per-sample thermal noise
                                         ///< voltage are discarded before they reach the cube
    unsigned int seed = 20260101;         ///< per-sensor random seed; fixed seed, fixed output
};

/// Complete description of one radar model.
///
/// Defaults describe a 77 GHz fast-chirp front radar. Archetypes ship as JSON under
/// data/sensor/radar and differ from each other only in this structure.
class CH_SENSOR_API ChRadarModelConfig {
  public:
    ChRadarModelConfig() = default;

    /// Read a configuration from a JSON file. Throws std::runtime_error on a malformed file and
    /// std::invalid_argument on a well-formed but inconsistent one.
    static ChRadarModelConfig ReadJSON(const std::string& filename);

    /// Write this configuration to a JSON file.
    void WriteJSON(const std::string& filename) const;

    /// Check internal consistency. Throws std::invalid_argument naming the offending field.
    void Validate() const;

    /// Human readable summary of the waveform, array and resulting resolutions.
    std::string GetDescription() const;

    /// Carrier wavelength [m].
    double GetWavelength() const;
    /// Bandwidth actually swept during the digitized window [Hz].
    double GetEffectiveBandwidth() const;
    /// Range bin size [m].
    double GetRangeResolution() const;
    /// Range of the last unaliased bin [m].
    double GetMaxUnambiguousRange() const;
    /// Number of range bins formed.
    unsigned int GetNumRangeBins() const;
    /// Number of transmitters sharing the coherent processing interval.
    unsigned int GetNumTransmitters() const;
    /// Number of virtual channels, one per transmit/receive pair.
    unsigned int GetNumVirtualChannels() const;
    /// Chirps available to each transmitter, which is the Doppler transform length.
    unsigned int GetNumDopplerBins() const;
    /// Chirp repetition interval seen by one transmitter [s].
    double GetEffectivePri() const;
    /// Velocity bin size [m/s].
    double GetVelocityResolution() const;
    /// Radial velocity beyond which Doppler folds [m/s].
    double GetMaxUnambiguousVelocity() const;
    /// Duration of one coherent processing interval [s].
    double GetCoherentProcessingInterval() const;
    /// Thermal noise power in one cube cell [W].
    double GetNoisePowerPerCell() const;
    /// Product of the two windows' power sums, the factor by which any white noise entering the
    /// receiver is scaled on its way into one cube cell.
    double GetWindowPowerProduct() const;
    /// Solid angle each launched ray represents [sr].
    double GetRaySolidAngle() const;
    /// Peak of the range window's transform, the coherent gain a target collects on that axis.
    double GetRangeWindowPeak() const;
    /// Peak of the Doppler window's transform.
    double GetDopplerWindowPeak() const;
    /// Training cells the constant false alarm rate detector averages over.
    unsigned int GetCfarTrainingCellCount() const;

    std::string name = "phys_radar";
    double field_of_view_azimuth = 1.0;    ///< full azimuth extent illuminated and traced [rad]
    double field_of_view_elevation = 0.25; ///< full elevation extent illuminated and traced [rad]
    double min_range = 0.3;                ///< near limit of the traced and processed span [m]
    double max_range = 250.0;              ///< far limit of the traced and processed span [m]

    ChRadarWaveformConfig waveform;
    ChRadarAntennaConfig antenna;
    ChRadarImpairmentConfig impairments;
    ChRadarDspConfig dsp;
    ChRadarTrackerConfig tracker;
    ChRadarRayTracingConfig ray_tracing;
};

/// Exponent of the separable cos^n antenna pattern that reproduces a 3 dB beamwidth. Returns zero,
/// meaning an isotropic factor, for a beamwidth outside (0, pi).
CH_SENSOR_API double ChRadarPatternExponent(double beamwidth);

/// Front radar with a single transmitter and a four element half-wavelength receive array,
/// in the class of a 77 GHz long range automotive sensor.
CH_SENSOR_API ChRadarModelConfig MakeDefaultFrontRadarConfig();

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
