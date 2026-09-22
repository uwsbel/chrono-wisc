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

#include "chrono_sensor/sensors/radar/ChRadarConfig.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "chrono/utils/ChConstants.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

namespace chrono {
namespace sensor {

namespace {

constexpr double c_light = 299792458.0;
constexpr double k_boltzmann = 1.380649e-23;

double DbToLinear(double db) {
    return std::pow(10.0, db / 10.0);
}

/// Peak of a window's transform, the coherent gain a target on that bin collects.
double WindowPeak(ChRadarWindowType type, unsigned int n) {
    switch (type) {
        case ChRadarWindowType::HANN:
            return 0.5 * n;
        case ChRadarWindowType::HAMMING:
            return 0.54 * n;
        case ChRadarWindowType::RECTANGULAR:
        default:
            return 1.0 * n;
    }
}

/// Sum of the squared window samples, which sets how much thermal noise power a cell collects.
double WindowPowerSum(ChRadarWindowType type, unsigned int n) {
    switch (type) {
        case ChRadarWindowType::HANN:
            return 0.375 * n;
        case ChRadarWindowType::HAMMING:
            return 0.3974 * n;
        case ChRadarWindowType::RECTANGULAR:
        default:
            return 1.0 * n;
    }
}

void Require(bool condition, const std::string& field, const std::string& why) {
    if (!condition)
        throw std::invalid_argument("ChRadarModelConfig: '" + field + "' " + why);
}

// --- JSON readers -----------------------------------------------------------------------------
// Every field is optional: a configuration file overrides the defaults it names and inherits the
// rest, so an archetype stays readable and a new field does not invalidate existing files.

using rapidjson::Value;

double ReadDouble(const Value& v, const char* key, double fallback) {
    if (!v.HasMember(key))
        return fallback;
    if (!v[key].IsNumber())
        throw std::runtime_error(std::string("ChRadarModelConfig: '") + key + "' must be a number");
    return v[key].GetDouble();
}

unsigned int ReadUint(const Value& v, const char* key, unsigned int fallback) {
    if (!v.HasMember(key))
        return fallback;
    if (!v[key].IsUint())
        throw std::runtime_error(std::string("ChRadarModelConfig: '") + key + "' must be a non-negative integer");
    return v[key].GetUint();
}

bool ReadBool(const Value& v, const char* key, bool fallback) {
    if (!v.HasMember(key))
        return fallback;
    if (!v[key].IsBool())
        throw std::runtime_error(std::string("ChRadarModelConfig: '") + key + "' must be a boolean");
    return v[key].GetBool();
}

std::string ReadString(const Value& v, const char* key, const std::string& fallback) {
    if (!v.HasMember(key))
        return fallback;
    if (!v[key].IsString())
        throw std::runtime_error(std::string("ChRadarModelConfig: '") + key + "' must be a string");
    return v[key].GetString();
}

double ReadAngleDeg(const Value& v, const char* key, double fallback_rad) {
    if (!v.HasMember(key))
        return fallback_rad;
    return ReadDouble(v, key, 0.0) * CH_DEG_TO_RAD;
}

template <typename T>
T ReadEnum(const Value& v,
           const char* key,
           T fallback,
           std::initializer_list<std::pair<const char*, T>> names) {
    if (!v.HasMember(key))
        return fallback;
    const std::string s = ReadString(v, key, "");
    for (const auto& n : names) {
        if (s == n.first)
            return n.second;
    }
    std::ostringstream msg;
    msg << "ChRadarModelConfig: unknown value '" << s << "' for '" << key << "'; expected one of";
    for (const auto& n : names)
        msg << " '" << n.first << "'";
    throw std::runtime_error(msg.str());
}

std::vector<ChRadarElementConfig> ReadElements(const Value& parent, const char* key) {
    std::vector<ChRadarElementConfig> elements;
    if (!parent.HasMember(key))
        return elements;
    if (!parent[key].IsArray())
        throw std::runtime_error(std::string("ChRadarModelConfig: '") + key + "' must be an array");
    for (const Value& e : parent[key].GetArray()) {
        ChRadarElementConfig element;
        if (e.HasMember("position")) {
            const Value& p = e["position"];
            if (!p.IsArray() || p.Size() != 3)
                throw std::runtime_error("ChRadarModelConfig: element 'position' must hold three numbers");
            element.position = ChVector3d(p[0].GetDouble(), p[1].GetDouble(), p[2].GetDouble());
        }
        element.gain_dbi = ReadDouble(e, "gain_dbi", element.gain_dbi);
        element.azimuth_beamwidth = ReadAngleDeg(e, "azimuth_beamwidth_deg", element.azimuth_beamwidth);
        element.elevation_beamwidth = ReadAngleDeg(e, "elevation_beamwidth_deg", element.elevation_beamwidth);
        elements.push_back(element);
    }
    return elements;
}

const char* WindowName(ChRadarWindowType t) {
    switch (t) {
        case ChRadarWindowType::RECTANGULAR:
            return "rectangular";
        case ChRadarWindowType::HAMMING:
            return "hamming";
        case ChRadarWindowType::HANN:
        default:
            return "hann";
    }
}

}  // namespace

// --- Derived quantities -----------------------------------------------------------------------

double ChRadarModelConfig::GetWavelength() const {
    return c_light / waveform.carrier_frequency;
}

double ChRadarModelConfig::GetEffectiveBandwidth() const {
    const double slope = waveform.bandwidth / waveform.chirp_duration;
    return slope * waveform.num_samples / waveform.sample_rate;
}

double ChRadarModelConfig::GetRangeResolution() const {
    return c_light / (2.0 * GetEffectiveBandwidth());
}

double ChRadarModelConfig::GetMaxUnambiguousRange() const {
    return waveform.num_samples * GetRangeResolution();
}

unsigned int ChRadarModelConfig::GetNumRangeBins() const {
    // Only the bins that fall inside the configured span are formed. Beyond max_range the cube
    // would hold nothing but noise, and the processing cost is linear in its size.
    const unsigned int spanned = static_cast<unsigned int>(std::ceil(max_range / GetRangeResolution())) + 1;
    return std::min(waveform.num_samples, spanned);
}

unsigned int ChRadarModelConfig::GetNumTransmitters() const {
    return antenna.transmitters.empty() ? 1u : static_cast<unsigned int>(antenna.transmitters.size());
}

unsigned int ChRadarModelConfig::GetNumVirtualChannels() const {
    const unsigned int rx = antenna.receivers.empty() ? 1u : static_cast<unsigned int>(antenna.receivers.size());
    return GetNumTransmitters() * rx;
}

unsigned int ChRadarModelConfig::GetNumDopplerBins() const {
    if (antenna.multiplex == ChRadarMultiplex::TDM)
        return waveform.num_chirps / GetNumTransmitters();
    return waveform.num_chirps;
}

double ChRadarModelConfig::GetEffectivePri() const {
    if (antenna.multiplex == ChRadarMultiplex::TDM)
        return waveform.chirp_repetition_interval * GetNumTransmitters();
    return waveform.chirp_repetition_interval;
}

double ChRadarModelConfig::GetVelocityResolution() const {
    return GetWavelength() / (2.0 * GetNumDopplerBins() * GetEffectivePri());
}

double ChRadarModelConfig::GetMaxUnambiguousVelocity() const {
    return GetWavelength() / (4.0 * GetEffectivePri());
}

double ChRadarModelConfig::GetCoherentProcessingInterval() const {
    return waveform.num_chirps * waveform.chirp_repetition_interval;
}

double ChRadarModelConfig::GetWindowPowerProduct() const {
    return WindowPowerSum(dsp.range_window, waveform.num_samples) *
           WindowPowerSum(dsp.doppler_window, GetNumDopplerBins());
}

double ChRadarModelConfig::GetNoisePowerPerCell() const {
    const double per_sample =
        k_boltzmann * impairments.system_temperature * DbToLinear(impairments.noise_figure_db) * waveform.sample_rate;
    return per_sample * GetWindowPowerProduct();
}

double ChRadarModelConfig::GetRaySolidAngle() const {
    // Rays tile the field of view uniformly in azimuth and elevation, so the solid angle a ray
    // represents varies with its own elevation. The mean value over the span is used: the
    // elevation extents of a road radar are small enough that the spread is under a percent.
    const double total = 2.0 * field_of_view_azimuth * std::sin(0.5 * field_of_view_elevation);
    return total / (ray_tracing.rays_azimuth * ray_tracing.rays_elevation);
}

double ChRadarModelConfig::GetRangeWindowPeak() const {
    return WindowPeak(dsp.range_window, waveform.num_samples);
}

double ChRadarModelConfig::GetDopplerWindowPeak() const {
    return WindowPeak(dsp.doppler_window, GetNumDopplerBins());
}

unsigned int ChRadarModelConfig::GetCfarTrainingCellCount() const {
    const unsigned int span_range = 2 * (dsp.cfar_train_range + dsp.cfar_guard_range) + 1;
    const unsigned int span_doppler = 2 * (dsp.cfar_train_doppler + dsp.cfar_guard_doppler) + 1;
    const unsigned int guard = (2 * dsp.cfar_guard_range + 1) * (2 * dsp.cfar_guard_doppler + 1);
    return span_range * span_doppler - guard;
}

double ChRadarPatternExponent(double beamwidth) {
    if (beamwidth <= 0.0 || beamwidth >= CH_PI)
        return 0.0;
    const double half_power = std::cos(0.5 * beamwidth);
    if (half_power <= 0.0 || half_power >= 1.0)
        return 0.0;
    return std::log(0.5) / std::log(half_power);
}

// --- Validation -------------------------------------------------------------------------------

void ChRadarModelConfig::Validate() const {
    Require(waveform.carrier_frequency > 0, "waveform.carrier_frequency", "must be positive");
    Require(waveform.bandwidth > 0, "waveform.bandwidth", "must be positive");
    Require(waveform.chirp_duration > 0, "waveform.chirp_duration", "must be positive");
    Require(waveform.sample_rate > 0, "waveform.sample_rate", "must be positive");
    Require(waveform.num_samples >= 8, "waveform.num_samples", "must be at least 8");
    Require(waveform.num_chirps >= 4, "waveform.num_chirps", "must be at least 4");

    const double digitized = waveform.num_samples / waveform.sample_rate;
    Require(digitized <= waveform.chirp_duration * (1.0 + 1e-9), "waveform.num_samples",
            "digitizes " + std::to_string(digitized) + " s, longer than the " +
                std::to_string(waveform.chirp_duration) + " s sweep");
    Require(waveform.chirp_repetition_interval >= waveform.chirp_duration,
            "waveform.chirp_repetition_interval", "must not be shorter than the sweep it repeats");

    Require(field_of_view_azimuth > 0 && field_of_view_azimuth <= CH_2PI, "field_of_view_azimuth",
            "must lie in (0, 2*pi]");
    Require(field_of_view_elevation > 0 && field_of_view_elevation <= CH_PI, "field_of_view_elevation",
            "must lie in (0, pi]");
    Require(max_range > min_range && min_range >= 0, "max_range", "must exceed min_range");
    Require(max_range <= GetMaxUnambiguousRange() * (1.0 + 1e-9), "max_range",
            "exceeds the unambiguous range " + std::to_string(GetMaxUnambiguousRange()) +
                " m implied by the waveform");

    Require(!antenna.receivers.empty(), "antenna.receivers", "must hold at least one element");
    Require(!antenna.transmitters.empty(), "antenna.transmitters", "must hold at least one element");
    if (antenna.multiplex == ChRadarMultiplex::TDM) {
        Require(waveform.num_chirps % GetNumTransmitters() == 0, "waveform.num_chirps",
                "must divide evenly among the transmitters when time division multiplexing");
        Require(GetNumDopplerBins() >= 4, "waveform.num_chirps",
                "leaves fewer than 4 chirps per transmitter");
    } else {
        Require(GetNumTransmitters() == 1, "antenna.multiplex",
                "must be 'tdm' when more than one transmitter is declared");
    }

    Require(dsp.num_azimuth_bins >= 1, "dsp.num_azimuth_bins", "must be at least 1");
    Require(dsp.false_alarm_rate > 0 && dsp.false_alarm_rate < 1, "dsp.false_alarm_rate",
            "must lie in (0, 1)");
    Require(dsp.os_rank_fraction > 0 && dsp.os_rank_fraction <= 1, "dsp.os_rank_fraction",
            "must lie in (0, 1]");
    Require(dsp.cfar_train_range > 0 || dsp.cfar_train_doppler > 0, "dsp.cfar_train_range",
            "and 'dsp.cfar_train_doppler' cannot both be zero");
    Require(dsp.max_detections > 0, "dsp.max_detections", "must be positive");

    Require(ray_tracing.rays_azimuth > 0 && ray_tracing.rays_elevation > 0, "ray_tracing.rays_azimuth",
            "and 'ray_tracing.rays_elevation' must be positive");
    Require(ray_tracing.max_bounces >= 1 && ray_tracing.max_bounces <= 8, "ray_tracing.max_bounces",
            "must lie in [1, 8]");
    Require(ray_tracing.max_paths >= 1024, "ray_tracing.max_paths", "must be at least 1024");

    Require(tracker.confirm_hits <= tracker.confirm_window, "tracker.confirm_hits",
            "cannot exceed 'tracker.confirm_window'");
    Require(tracker.gate_distance > 0, "tracker.gate_distance", "must be positive");
}

std::string ChRadarModelConfig::GetDescription() const {
    std::ostringstream s;
    s << "radar '" << name << "'\n";
    s << "  carrier          " << waveform.carrier_frequency / 1e9 << " GHz (lambda "
      << GetWavelength() * 1e3 << " mm)\n";
    s << "  swept bandwidth  " << waveform.bandwidth / 1e6 << " MHz over " << waveform.chirp_duration * 1e6
      << " us, " << GetEffectiveBandwidth() / 1e6 << " MHz digitized\n";
    s << "  range            " << min_range << " - " << max_range << " m, " << GetRangeResolution()
      << " m per bin, " << GetNumRangeBins() << " bins\n";
    s << "  velocity         +/-" << GetMaxUnambiguousVelocity() << " m/s, " << GetVelocityResolution()
      << " m/s per bin, " << GetNumDopplerBins() << " bins\n";
    s << "  array            " << antenna.transmitters.size() << " tx x " << antenna.receivers.size()
      << " rx = " << GetNumVirtualChannels() << " virtual channels, "
      << (antenna.multiplex == ChRadarMultiplex::TDM ? "time division multiplexed" : "single transmit")
      << "\n";
    s << "  field of view    " << field_of_view_azimuth * CH_RAD_TO_DEG << " deg azimuth x "
      << field_of_view_elevation * CH_RAD_TO_DEG << " deg elevation\n";
    s << "  cycle            " << GetCoherentProcessingInterval() * 1e3 << " ms coherent interval\n";
    s << "  rays             " << ray_tracing.rays_azimuth << " x " << ray_tracing.rays_elevation
      << ", up to " << ray_tracing.max_bounces << " bounces\n";
    s << "  noise floor      " << 10.0 * std::log10(GetNoisePowerPerCell() * 1e3) << " dBm per cell\n";
    return s.str();
}

// --- JSON -------------------------------------------------------------------------------------

ChRadarModelConfig ChRadarModelConfig::ReadJSON(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("ChRadarModelConfig: cannot open '" + filename + "'");
    std::stringstream contents;
    contents << file.rdbuf();
    const std::string text = contents.str();

    rapidjson::Document d;
    d.Parse(text.c_str());
    if (d.HasParseError() || !d.IsObject())
        throw std::runtime_error("ChRadarModelConfig: '" + filename + "' is not a valid JSON object");

    ChRadarModelConfig cfg = MakeDefaultFrontRadarConfig();
    cfg.name = ReadString(d, "name", cfg.name);
    cfg.field_of_view_azimuth = ReadAngleDeg(d, "field_of_view_azimuth_deg", cfg.field_of_view_azimuth);
    cfg.field_of_view_elevation = ReadAngleDeg(d, "field_of_view_elevation_deg", cfg.field_of_view_elevation);
    cfg.min_range = ReadDouble(d, "min_range_m", cfg.min_range);
    cfg.max_range = ReadDouble(d, "max_range_m", cfg.max_range);

    if (d.HasMember("waveform")) {
        const Value& w = d["waveform"];
        auto& t = cfg.waveform;
        t.type = ReadEnum(w, "type", t.type, {{"fmcw_fast_chirp", ChRadarWaveformType::FMCW_FAST_CHIRP}});
        t.carrier_frequency = ReadDouble(w, "carrier_frequency_hz", t.carrier_frequency);
        t.bandwidth = ReadDouble(w, "bandwidth_hz", t.bandwidth);
        t.chirp_duration = ReadDouble(w, "chirp_duration_s", t.chirp_duration);
        t.chirp_repetition_interval = ReadDouble(w, "chirp_repetition_interval_s", t.chirp_repetition_interval);
        t.num_chirps = ReadUint(w, "num_chirps", t.num_chirps);
        t.num_samples = ReadUint(w, "num_samples", t.num_samples);
        t.sample_rate = ReadDouble(w, "sample_rate_hz", t.sample_rate);
        t.transmit_power_dbm = ReadDouble(w, "transmit_power_dbm", t.transmit_power_dbm);
    }

    if (d.HasMember("antenna")) {
        const Value& a = d["antenna"];
        auto tx = ReadElements(a, "transmitters");
        auto rx = ReadElements(a, "receivers");
        if (!tx.empty())
            cfg.antenna.transmitters = std::move(tx);
        if (!rx.empty())
            cfg.antenna.receivers = std::move(rx);
        cfg.antenna.multiplex = ReadEnum(a, "multiplex", cfg.antenna.multiplex,
                                         {{"none", ChRadarMultiplex::NONE}, {"tdm", ChRadarMultiplex::TDM}});
    }

    if (d.HasMember("impairments")) {
        const Value& i = d["impairments"];
        auto& t = cfg.impairments;
        t.noise_figure_db = ReadDouble(i, "noise_figure_db", t.noise_figure_db);
        t.system_temperature = ReadDouble(i, "system_temperature_k", t.system_temperature);
        t.phase_noise_dbc_hz = ReadDouble(i, "phase_noise_dbc_hz", t.phase_noise_dbc_hz);
        t.phase_noise_offset = ReadDouble(i, "phase_noise_offset_hz", t.phase_noise_offset);
        t.phase_noise_decade_slope = ReadDouble(i, "phase_noise_decade_slope_db", t.phase_noise_decade_slope);
        t.transmit_leakage_db = ReadDouble(i, "transmit_leakage_db", t.transmit_leakage_db);
        t.adc_bits = ReadUint(i, "adc_bits", t.adc_bits);
        t.enable_thermal_noise = ReadBool(i, "enable_thermal_noise", t.enable_thermal_noise);
        t.enable_phase_noise = ReadBool(i, "enable_phase_noise", t.enable_phase_noise);
        t.enable_transmit_leakage = ReadBool(i, "enable_transmit_leakage", t.enable_transmit_leakage);
        t.enable_quantization = ReadBool(i, "enable_quantization", t.enable_quantization);
    }

    if (d.HasMember("dsp")) {
        const Value& p = d["dsp"];
        auto& t = cfg.dsp;
        const std::initializer_list<std::pair<const char*, ChRadarWindowType>> windows = {
            {"rectangular", ChRadarWindowType::RECTANGULAR},
            {"hann", ChRadarWindowType::HANN},
            {"hamming", ChRadarWindowType::HAMMING}};
        t.range_window = ReadEnum(p, "range_window", t.range_window, windows);
        t.doppler_window = ReadEnum(p, "doppler_window", t.doppler_window, windows);
        t.num_azimuth_bins = ReadUint(p, "num_azimuth_bins", t.num_azimuth_bins);
        t.cfar_type =
            ReadEnum(p, "cfar_type", t.cfar_type, {{"ca", ChRadarCfarType::CA}, {"os", ChRadarCfarType::OS}});
        t.cfar_guard_range = ReadUint(p, "cfar_guard_range", t.cfar_guard_range);
        t.cfar_guard_doppler = ReadUint(p, "cfar_guard_doppler", t.cfar_guard_doppler);
        t.cfar_train_range = ReadUint(p, "cfar_train_range", t.cfar_train_range);
        t.cfar_train_doppler = ReadUint(p, "cfar_train_doppler", t.cfar_train_doppler);
        t.false_alarm_rate = ReadDouble(p, "false_alarm_rate", t.false_alarm_rate);
        t.os_rank_fraction = ReadDouble(p, "os_rank_fraction", t.os_rank_fraction);
        t.suppress_stationary = ReadBool(p, "suppress_stationary", t.suppress_stationary);
        t.stationary_tolerance = ReadDouble(p, "stationary_tolerance_mps", t.stationary_tolerance);
        t.max_detections = ReadUint(p, "max_detections", t.max_detections);
    }

    if (d.HasMember("tracker")) {
        const Value& k = d["tracker"];
        auto& t = cfg.tracker;
        t.confirm_hits = ReadUint(k, "confirm_hits", t.confirm_hits);
        t.confirm_window = ReadUint(k, "confirm_window", t.confirm_window);
        t.coast_cycles = ReadUint(k, "coast_cycles", t.coast_cycles);
        t.gate_distance = ReadDouble(k, "gate_distance_m", t.gate_distance);
        t.process_noise = ReadDouble(k, "process_noise", t.process_noise);
        t.measurement_noise = ReadDouble(k, "measurement_noise", t.measurement_noise);
    }

    if (d.HasMember("ray_tracing")) {
        const Value& r = d["ray_tracing"];
        auto& t = cfg.ray_tracing;
        t.rays_azimuth = ReadUint(r, "rays_azimuth", t.rays_azimuth);
        t.rays_elevation = ReadUint(r, "rays_elevation", t.rays_elevation);
        t.max_bounces = ReadUint(r, "max_bounces", t.max_bounces);
        t.max_paths = ReadUint(r, "max_paths", t.max_paths);
        t.amplitude_cutoff_db = ReadDouble(r, "amplitude_cutoff_db", t.amplitude_cutoff_db);
        t.seed = ReadUint(r, "seed", t.seed);
    }

    cfg.Validate();
    return cfg;
}

void ChRadarModelConfig::WriteJSON(const std::string& filename) const {
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> w(buffer);

    auto element_array = [&w](const std::vector<ChRadarElementConfig>& elements) {
        w.StartArray();
        for (const auto& e : elements) {
            w.StartObject();
            w.Key("position");
            w.StartArray();
            w.Double(e.position.x());
            w.Double(e.position.y());
            w.Double(e.position.z());
            w.EndArray();
            w.Key("gain_dbi");
            w.Double(e.gain_dbi);
            w.Key("azimuth_beamwidth_deg");
            w.Double(e.azimuth_beamwidth * CH_RAD_TO_DEG);
            w.Key("elevation_beamwidth_deg");
            w.Double(e.elevation_beamwidth * CH_RAD_TO_DEG);
            w.EndObject();
        }
        w.EndArray();
    };

    w.StartObject();
    w.Key("name");
    w.String(name.c_str());
    w.Key("field_of_view_azimuth_deg");
    w.Double(field_of_view_azimuth * CH_RAD_TO_DEG);
    w.Key("field_of_view_elevation_deg");
    w.Double(field_of_view_elevation * CH_RAD_TO_DEG);
    w.Key("min_range_m");
    w.Double(min_range);
    w.Key("max_range_m");
    w.Double(max_range);

    w.Key("waveform");
    w.StartObject();
    w.Key("type");
    w.String("fmcw_fast_chirp");
    w.Key("carrier_frequency_hz");
    w.Double(waveform.carrier_frequency);
    w.Key("bandwidth_hz");
    w.Double(waveform.bandwidth);
    w.Key("chirp_duration_s");
    w.Double(waveform.chirp_duration);
    w.Key("chirp_repetition_interval_s");
    w.Double(waveform.chirp_repetition_interval);
    w.Key("num_chirps");
    w.Uint(waveform.num_chirps);
    w.Key("num_samples");
    w.Uint(waveform.num_samples);
    w.Key("sample_rate_hz");
    w.Double(waveform.sample_rate);
    w.Key("transmit_power_dbm");
    w.Double(waveform.transmit_power_dbm);
    w.EndObject();

    w.Key("antenna");
    w.StartObject();
    w.Key("transmitters");
    element_array(antenna.transmitters);
    w.Key("receivers");
    element_array(antenna.receivers);
    w.Key("multiplex");
    w.String(antenna.multiplex == ChRadarMultiplex::TDM ? "tdm" : "none");
    w.EndObject();

    w.Key("impairments");
    w.StartObject();
    w.Key("noise_figure_db");
    w.Double(impairments.noise_figure_db);
    w.Key("system_temperature_k");
    w.Double(impairments.system_temperature);
    w.Key("phase_noise_dbc_hz");
    w.Double(impairments.phase_noise_dbc_hz);
    w.Key("phase_noise_offset_hz");
    w.Double(impairments.phase_noise_offset);
    w.Key("phase_noise_decade_slope_db");
    w.Double(impairments.phase_noise_decade_slope);
    w.Key("transmit_leakage_db");
    w.Double(impairments.transmit_leakage_db);
    w.Key("adc_bits");
    w.Uint(impairments.adc_bits);
    w.Key("enable_thermal_noise");
    w.Bool(impairments.enable_thermal_noise);
    w.Key("enable_phase_noise");
    w.Bool(impairments.enable_phase_noise);
    w.Key("enable_transmit_leakage");
    w.Bool(impairments.enable_transmit_leakage);
    w.Key("enable_quantization");
    w.Bool(impairments.enable_quantization);
    w.EndObject();

    w.Key("dsp");
    w.StartObject();
    w.Key("range_window");
    w.String(WindowName(dsp.range_window));
    w.Key("doppler_window");
    w.String(WindowName(dsp.doppler_window));
    w.Key("num_azimuth_bins");
    w.Uint(dsp.num_azimuth_bins);
    w.Key("cfar_type");
    w.String(dsp.cfar_type == ChRadarCfarType::OS ? "os" : "ca");
    w.Key("cfar_guard_range");
    w.Uint(dsp.cfar_guard_range);
    w.Key("cfar_guard_doppler");
    w.Uint(dsp.cfar_guard_doppler);
    w.Key("cfar_train_range");
    w.Uint(dsp.cfar_train_range);
    w.Key("cfar_train_doppler");
    w.Uint(dsp.cfar_train_doppler);
    w.Key("false_alarm_rate");
    w.Double(dsp.false_alarm_rate);
    w.Key("os_rank_fraction");
    w.Double(dsp.os_rank_fraction);
    w.Key("suppress_stationary");
    w.Bool(dsp.suppress_stationary);
    w.Key("stationary_tolerance_mps");
    w.Double(dsp.stationary_tolerance);
    w.Key("max_detections");
    w.Uint(dsp.max_detections);
    w.EndObject();

    w.Key("tracker");
    w.StartObject();
    w.Key("confirm_hits");
    w.Uint(tracker.confirm_hits);
    w.Key("confirm_window");
    w.Uint(tracker.confirm_window);
    w.Key("coast_cycles");
    w.Uint(tracker.coast_cycles);
    w.Key("gate_distance_m");
    w.Double(tracker.gate_distance);
    w.Key("process_noise");
    w.Double(tracker.process_noise);
    w.Key("measurement_noise");
    w.Double(tracker.measurement_noise);
    w.EndObject();

    w.Key("ray_tracing");
    w.StartObject();
    w.Key("rays_azimuth");
    w.Uint(ray_tracing.rays_azimuth);
    w.Key("rays_elevation");
    w.Uint(ray_tracing.rays_elevation);
    w.Key("max_bounces");
    w.Uint(ray_tracing.max_bounces);
    w.Key("max_paths");
    w.Uint(ray_tracing.max_paths);
    w.Key("amplitude_cutoff_db");
    w.Double(ray_tracing.amplitude_cutoff_db);
    w.Key("seed");
    w.Uint(ray_tracing.seed);
    w.EndObject();

    w.EndObject();

    std::ofstream out(filename);
    if (!out.is_open())
        throw std::runtime_error("ChRadarModelConfig: cannot write '" + filename + "'");
    out << buffer.GetString() << "\n";
}

// --- Shipped default --------------------------------------------------------------------------

ChRadarModelConfig MakeDefaultFrontRadarConfig() {
    ChRadarModelConfig cfg;
    cfg.name = "front_radar_77ghz";
    cfg.field_of_view_azimuth = 60.0 * CH_DEG_TO_RAD;
    cfg.field_of_view_elevation = 10.0 * CH_DEG_TO_RAD;
    cfg.min_range = 0.3;
    cfg.max_range = 120.0;

    cfg.waveform.carrier_frequency = 76.5e9;
    cfg.waveform.bandwidth = 4.0e8;
    cfg.waveform.chirp_duration = 32.0e-6;
    cfg.waveform.chirp_repetition_interval = 40.0e-6;
    cfg.waveform.num_chirps = 256;
    cfg.waveform.num_samples = 512;
    cfg.waveform.sample_rate = 2.0e7;
    cfg.waveform.transmit_power_dbm = 12.0;

    ChRadarElementConfig tx;
    tx.position = ChVector3d(0, 0, 0);
    tx.gain_dbi = 18.0;
    tx.azimuth_beamwidth = 60.0 * CH_DEG_TO_RAD;
    tx.elevation_beamwidth = 12.0 * CH_DEG_TO_RAD;
    cfg.antenna.transmitters = {tx};

    // Half-wavelength uniform linear array along the sensor's lateral axis. Half-wavelength
    // spacing is what keeps the visible region free of grating lobes; widening it is how a
    // configuration buys angular resolution at the price of ambiguous angles.
    const double half_wavelength = 0.5 * cfg.GetWavelength();
    cfg.antenna.receivers.clear();
    for (int i = 0; i < 8; i++) {
        ChRadarElementConfig rx;
        rx.position = ChVector3d(0, (i - 3.5) * half_wavelength, 0);
        rx.gain_dbi = 12.0;
        rx.azimuth_beamwidth = 100.0 * CH_DEG_TO_RAD;
        rx.elevation_beamwidth = 20.0 * CH_DEG_TO_RAD;
        cfg.antenna.receivers.push_back(rx);
    }
    cfg.antenna.multiplex = ChRadarMultiplex::NONE;

    cfg.dsp.num_azimuth_bins = 96;
    cfg.ray_tracing.rays_azimuth = 512;
    cfg.ray_tracing.rays_elevation = 96;
    cfg.ray_tracing.max_bounces = 3;

    return cfg;
}

}  // namespace sensor
}  // namespace chrono
