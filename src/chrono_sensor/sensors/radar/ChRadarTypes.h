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
// Plain-old-data types shared by the wave-domain radar's ray-tracing shaders, its CUDA signal
// processing kernels and its host-side filters. Kept free of the standard library and of the
// CUDA runtime so it can be included from NVRTC-compiled shader sources.
//
// =============================================================================

#ifndef CHRADARTYPES_H
#define CHRADARTYPES_H

#include <vector_types.h>

/// The small helpers below are used from host code, from CUDA kernels and from the ray-tracing
/// shaders, so they are annotated only where the annotation is meaningful.
#ifdef __CUDACC__
    #define CH_RADAR_HOST_DEVICE __host__ __device__
#else
    #define CH_RADAR_HOST_DEVICE
#endif

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// One coherent propagation path from the transmit reference element, through one or more
/// surface interactions, to the receive reference element. The only quantity the radar ray
/// tracer produces; the signal chain forms everything else by coherent summation over these.
struct RadarPath {
    float length;       ///< total path length, transmitter to receiver, [m]
    float length_rate;  ///< rate of change of the path length [m/s]; Doppler is -length_rate/lambda
    float amplitude;    ///< voltage amplitude at the reference element [sqrt(W)]
    float phase;        ///< phase [rad], including the 2*pi*length/lambda propagation term
    float3 dir_rx;      ///< unit direction of arrival at the antenna, in the sensor frame
    unsigned int bounces;    ///< number of surface interactions along the path
    unsigned int object_id;  ///< ground-truth id of the last surface hit; provenance only, never
                             ///< read by the signal chain
    unsigned int flags;      ///< reserved for path classification
};

/// Interleaved complex sample of the range-Doppler cube.
struct RadarComplex {
    float re;  ///< in-phase component
    float im;  ///< quadrature component
};

/// Bit flags carried by a RadarDetection.
enum RadarDetectionFlags : unsigned int {
    RADAR_DET_MULTIPATH = 1u << 0,          ///< dominant path had more than one surface
                                            ///< interaction, i.e. the detection is a mirror ghost
    RADAR_DET_AMBIGUOUS_VELOCITY = 1u << 1  ///< peak sits in a Doppler bin the waveform folds
};

/// A detection reported by the constant false alarm rate stage. Field-mappable to ASAM OSI
/// RadarDetection; no OSI dependency is introduced.
struct RadarDetection {
    float range;             ///< [m]
    float range_rate;        ///< radial velocity, positive opening, [m/s]
    float azimuth;           ///< [rad] in the sensor frame
    float elevation;         ///< [rad] in the sensor frame
    float snr_db;            ///< signal-to-noise ratio of the detected cell [dB]
    float rcs_dbsm;          ///< radar cross section implied by the radar equation [dBsm]
    float amplitude;         ///< cell magnitude that crossed the threshold [sqrt(W)]
    unsigned int object_id;  ///< ground-truth provenance of the dominant path in the cell
    unsigned int flags;      ///< RadarDetectionFlags bit field
};

/// A tracked object, mirroring the semantics of an automotive radar object list.
struct RadarObject {
    unsigned int id;              ///< track id, stable across cycles
    float x;                      ///< position in the sensor frame [m]
    float y;                      ///< position in the sensor frame [m]
    float z;                      ///< position in the sensor frame [m]
    float vx;                     ///< velocity in the sensor frame [m/s]
    float vy;                     ///< velocity in the sensor frame [m/s]
    float range_rate;             ///< measured radial velocity [m/s]
    float rcs_dbsm;               ///< smoothed radar cross section estimate [dBsm]
    float existence_probability;  ///< 0..1
    unsigned int age_cycles;      ///< cycles since the track was first hit
    unsigned int dyn_prop;        ///< 0 moving, 1 stationary, 2 oncoming
    unsigned int object_id;       ///< ground-truth provenance of the contributing detections
};

/// Packed record of the strongest path that reached a range-Doppler cell: its amplitude in the
/// high word, so an atomic maximum picks the strongest, and its bounce count and ground-truth
/// object id in the low word. Reading these back is what lets a detection be scored against the
/// geometry that produced it, and a ghost be told from a target.
CH_RADAR_HOST_DEVICE inline unsigned int RadarProvenanceObjectId(unsigned long long key) {
    return static_cast<unsigned int>(key) & 0x00FFFFFFu;
}

CH_RADAR_HOST_DEVICE inline unsigned int RadarProvenanceBounces(unsigned long long key) {
    return (static_cast<unsigned int>(key) >> 24) & 0xFFu;
}

/// A transmit or receive antenna element, as seen by the ray tracer.
/// The pattern is the separable form G(az, el) = gain * cos^n_az(az) * cos^n_el(el), clipped to
/// the forward hemisphere. An exponent of zero gives an isotropic factor on that axis.
struct RadarAntennaElement {
    float3 position;  ///< phase centre in the sensor frame [m]
    float gain;       ///< peak gain, linear
    float az_exponent;
    float el_exponent;
};

/// Electromagnetic description of one surface at the radar carrier frequency.
///
/// The split between a coherent specular lobe and an incoherent diffuse lobe is what makes
/// fading, clutter and mirror ghosts emerge rather than be injected: the specular term carries
/// phase and interferes, the diffuse term decorrelates.
struct RadarMaterial {
    float specular_reflectivity;  ///< power fraction into the coherent specular lobe
    float diffuse_reflectivity;   ///< Lambertian albedo of the incoherent lobe
    float lobe_width;             ///< RMS width of the specular lobe [rad]; 0 for a mirror
    float transmission;           ///< power fraction passing through a thin layer
    int assigned;                 ///< 0 when no registry entry exists, in which case the shader
                                  ///< derives the response from the visual material instead
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
