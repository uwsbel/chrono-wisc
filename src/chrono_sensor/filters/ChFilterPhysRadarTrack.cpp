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

#include "chrono_sensor/filters/ChFilterPhysRadarTrack.h"

#include <algorithm>
#include <cmath>
#include <map>

#include <cuda_runtime.h>

namespace chrono {
namespace sensor {

namespace {

/// A target seen by the detector: the several detections it spread over, reduced to one.
struct Measurement {
    double x = 0.0;
    double y = 0.0;
    double range_rate = 0.0;
    double rcs_dbsm = -30.0;
    unsigned int object_id = 0;
    unsigned int flags = 0;
};

/// Disjoint set over detection indices, for single-link grouping.
class DisjointSet {
  public:
    explicit DisjointSet(size_t n) : m_parent(n) {
        for (size_t i = 0; i < n; i++)
            m_parent[i] = i;
    }

    size_t Find(size_t i) {
        while (m_parent[i] != i) {
            m_parent[i] = m_parent[m_parent[i]];
            i = m_parent[i];
        }
        return i;
    }

    void Union(size_t a, size_t b) {
        a = Find(a);
        b = Find(b);
        if (a != b)
            m_parent[b] = a;
    }

  private:
    std::vector<size_t> m_parent;
};

/// Group detections that belong to the same target.
///
/// Velocity joins the clustering space, so two vehicles overlapping in bearing stay apart as long
/// as they move differently. That separation is one a radar has and a range-only sensor does not.
///
/// Detections are bucketed on a grid of the gate spacing and only neighbouring buckets are
/// compared, which keeps a frame with a few thousand detections linear rather than quadratic.
std::vector<Measurement> GroupDetections(const std::vector<RadarDetection>& detections,
                                         double gate_distance,
                                         double velocity_weight) {
    std::vector<Measurement> measurements;
    if (detections.empty())
        return measurements;

    struct Point {
        double x, y, v;
    };
    std::vector<Point> points(detections.size());
    for (size_t i = 0; i < detections.size(); i++) {
        const RadarDetection& d = detections[i];
        points[i] = {d.range * std::cos(d.azimuth), d.range * std::sin(d.azimuth), d.range_rate * velocity_weight};
    }

    const double cell = std::max(gate_distance, 1e-3);
    const double gate_squared = gate_distance * gate_distance;
    std::map<std::pair<long, long>, std::vector<size_t>> buckets;
    for (size_t i = 0; i < points.size(); i++) {
        const long cx = (long)std::floor(points[i].x / cell);
        const long cy = (long)std::floor(points[i].y / cell);
        buckets[{cx, cy}].push_back(i);
    }

    DisjointSet groups(points.size());
    for (const auto& bucket : buckets) {
        for (long dx = 0; dx <= 1; dx++) {
            for (long dy = (dx == 0 ? 0 : -1); dy <= 1; dy++) {
                const auto neighbour = buckets.find({bucket.first.first + dx, bucket.first.second + dy});
                if (neighbour == buckets.end())
                    continue;
                for (size_t i : bucket.second) {
                    for (size_t j : neighbour->second) {
                        if (j <= i)
                            continue;
                        const double ex = points[i].x - points[j].x;
                        const double ey = points[i].y - points[j].y;
                        const double ev = points[i].v - points[j].v;
                        if (ex * ex + ey * ey + ev * ev <= gate_squared)
                            groups.Union(i, j);
                    }
                }
            }
        }
    }

    std::map<size_t, std::vector<size_t>> clusters;
    for (size_t i = 0; i < points.size(); i++)
        clusters[groups.Find(i)].push_back(i);

    for (const auto& cluster : clusters) {
        Measurement m;
        double weight_sum = 0.0;
        double power_sum = 0.0;
        std::map<unsigned int, double> provenance;
        for (size_t index : cluster.second) {
            const RadarDetection& d = detections[index];
            // Strong detections carry the centroid, since they localize the target best.
            const double weight = std::pow(10.0, 0.1 * d.snr_db);
            m.x += weight * points[index].x;
            m.y += weight * points[index].y;
            m.range_rate += weight * d.range_rate;
            weight_sum += weight;
            power_sum += std::pow(10.0, 0.1 * d.rcs_dbsm);
            provenance[d.object_id] += weight;
            m.flags |= d.flags;
        }
        if (weight_sum <= 0.0)
            continue;
        m.x /= weight_sum;
        m.y /= weight_sum;
        m.range_rate /= weight_sum;
        m.rcs_dbsm = 10.0 * std::log10(std::max(power_sum, 1e-6));
        m.object_id = std::max_element(provenance.begin(), provenance.end(),
                                       [](const auto& a, const auto& b) { return a.second < b.second; })
                          ->first;
        measurements.push_back(m);
    }
    return measurements;
}

}  // namespace

ChFilterPhysRadarTrack::ChFilterPhysRadarTrack(std::string name) : ChFilter(name) {}

ChFilterPhysRadarTrack::~ChFilterPhysRadarTrack() {}

void ChFilterPhysRadarTrack::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_radar = std::dynamic_pointer_cast<ChPhysRadarSensor>(pSensor);
    if (!m_radar)
        InvalidFilterGraphSensorTypeMismatch(pSensor);

    m_buffer = std::dynamic_pointer_cast<SensorDevicePhysRadarFrame>(bufferInOut);
    if (!m_buffer)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    m_cuda_stream = m_radar->GetCudaStream();
    m_detections.resize(m_buffer->DetectionCapacity);
    bufferInOut = m_buffer;
}

void ChFilterPhysRadarTrack::Apply() {
    const ChRadarTrackerConfig& cfg = m_radar->GetConfig().tracker;

    const unsigned int count = m_buffer->NumDetections;
    if (count > 0) {
        cudaMemcpyAsync(m_detections.data(), m_buffer->Detections.get(), count * sizeof(RadarDetection),
                        cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
    }
    const std::vector<RadarDetection> detections(m_detections.begin(), m_detections.begin() + count);

    double dt = m_started ? (double)(m_buffer->TimeStamp - m_last_time) : 0.0;
    if (dt <= 0.0)
        dt = 1.0 / std::max(1e-3f, m_radar->GetUpdateRate());
    m_last_time = m_buffer->TimeStamp;
    m_started = true;

    // Predict every track forward at constant velocity.
    ChMatrixNM<double, 4, 4> transition;
    transition.setIdentity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;

    const double q = cfg.process_noise * cfg.process_noise;
    ChMatrixNM<double, 4, 4> process;
    process.setZero();
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    for (int axis = 0; axis < 2; axis++) {
        process(axis, axis) = 0.25 * dt4 * q;
        process(axis, axis + 2) = 0.5 * dt3 * q;
        process(axis + 2, axis) = 0.5 * dt3 * q;
        process(axis + 2, axis + 2) = dt2 * q;
    }

    for (Track& t : m_tracks) {
        t.state = transition * t.state;
        t.covariance = transition * t.covariance * transition.transpose() + process;
    }

    const auto measurements = GroupDetections(detections, cfg.gate_distance, 1.0);

    // Greedy nearest-neighbour association, closest pair first so a short unambiguous match is
    // never spent on a distant track.
    std::vector<int> track_for_measurement(measurements.size(), -1);
    std::vector<bool> track_taken(m_tracks.size(), false);
    struct Candidate {
        double distance;
        size_t measurement;
        size_t track;
    };
    std::vector<Candidate> candidates;
    for (size_t mi = 0; mi < measurements.size(); mi++) {
        for (size_t ti = 0; ti < m_tracks.size(); ti++) {
            const double dx = measurements[mi].x - m_tracks[ti].state(0);
            const double dy = measurements[mi].y - m_tracks[ti].state(1);
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= cfg.gate_distance)
                candidates.push_back({distance, mi, ti});
        }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.distance < b.distance; });
    for (const Candidate& c : candidates) {
        if (track_for_measurement[c.measurement] >= 0 || track_taken[c.track])
            continue;
        track_for_measurement[c.measurement] = (int)c.track;
        track_taken[c.track] = true;
    }

    const size_t existing_tracks = m_tracks.size();
    std::vector<bool> updated(existing_tracks, false);
    std::vector<Track> new_tracks;

    ChMatrixNM<double, 2, 4> observation;
    observation.setZero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;
    ChMatrixNM<double, 2, 2> measurement_noise;
    measurement_noise.setZero();
    measurement_noise(0, 0) = cfg.measurement_noise * cfg.measurement_noise;
    measurement_noise(1, 1) = cfg.measurement_noise * cfg.measurement_noise;

    for (size_t mi = 0; mi < measurements.size(); mi++) {
        const Measurement& m = measurements[mi];
        const int ti = track_for_measurement[mi];
        if (ti < 0) {
            Track track;
            track.state.setZero();
            track.state(0) = m.x;
            track.state(1) = m.y;
            track.covariance.setIdentity();
            track.covariance *= cfg.gate_distance * cfg.gate_distance;
            track.range_rate = m.range_rate;
            track.rcs_dbsm = m.rcs_dbsm;
            track.id = m_next_id++;
            track.age = 1;
            track.object_id = m.object_id;
            track.history.push_back(true);
            new_tracks.push_back(track);
            continue;
        }

        Track& track = m_tracks[ti];
        ChVectorN<double, 2> innovation;
        innovation(0) = m.x - track.state(0);
        innovation(1) = m.y - track.state(1);
        const ChMatrixNM<double, 2, 2> innovation_covariance =
            observation * track.covariance * observation.transpose() + measurement_noise;
        const ChMatrixNM<double, 4, 2> gain =
            track.covariance * observation.transpose() * innovation_covariance.inverse();
        track.state += gain * innovation;
        ChMatrixNM<double, 4, 4> identity;
        identity.setIdentity();
        track.covariance = (identity - gain * observation) * track.covariance;

        track.range_rate = m.range_rate;
        track.rcs_dbsm = 0.7 * track.rcs_dbsm + 0.3 * m.rcs_dbsm;
        track.object_id = m.object_id;
        track.age++;
        track.misses = 0;
        track.history.push_back(true);
        updated[ti] = true;
    }

    for (size_t ti = 0; ti < existing_tracks; ti++) {
        if (!updated[ti]) {
            m_tracks[ti].misses++;
            m_tracks[ti].age++;
            m_tracks[ti].history.push_back(false);
        }
    }
    m_tracks.insert(m_tracks.end(), new_tracks.begin(), new_tracks.end());

    for (Track& t : m_tracks) {
        while (t.history.size() > cfg.confirm_window)
            t.history.pop_front();
    }

    m_tracks.erase(std::remove_if(m_tracks.begin(), m_tracks.end(),
                                  [&cfg](const Track& t) { return t.misses > cfg.coast_cycles; }),
                   m_tracks.end());

    const double ego_speed = m_radar->GetTranslationalVelocity().Length();

    m_buffer->Objects.clear();
    for (const Track& t : m_tracks) {
        const unsigned int hits = (unsigned int)std::count(t.history.begin(), t.history.end(), true);
        if (hits < cfg.confirm_hits)
            continue;

        RadarObject object;
        object.id = t.id;
        object.x = (float)t.state(0);
        object.y = (float)t.state(1);
        object.z = 0.f;
        object.vx = (float)t.state(2);
        object.vy = (float)t.state(3);
        object.range_rate = (float)t.range_rate;
        object.rcs_dbsm = (float)t.rcs_dbsm;
        object.existence_probability = (float)hits / (float)std::max(1u, (unsigned int)t.history.size());
        object.age_cycles = t.age;
        object.object_id = t.object_id;

        // A stationary object closes the range at the ego speed projected on its bearing.
        const double bearing = std::atan2(t.state(1), t.state(0));
        const double clutter_rate = -ego_speed * std::cos(bearing);
        if (std::abs(t.range_rate - clutter_rate) < 0.5)
            object.dyn_prop = 1;
        else if (t.range_rate < clutter_rate)
            object.dyn_prop = 2;
        else
            object.dyn_prop = 0;

        m_buffer->Objects.push_back(object);
    }
}

}  // namespace sensor
}  // namespace chrono
