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
// Authors: bgwitt
// =============================================================================
//
// Demo helper: terrain hazard avoidance for a rover driving to a goal. Ground
// steeper, or rougher under the rover's footprint, than the rover should drive
// is a hazard, grown by the rover's radius, with a cost for passing close to it.
// Planning runs at two scales, as with a rover planning from an orbital map:
// once, over the whole corridor to the goal on a coarse costmap, for every
// point's cheapest cost to reach the goal around the craters; and live, every
// update, on a fine costmap around the rover, for the path that best trades
// its cost against that cost-to-go, favoring the previous path so the route
// does not flip. Pure pursuit steers along the path with a limited steering
// rate. Craters, from the DEM or procedural, show up as steep walls.
//
// =============================================================================

#ifndef DEMO_PLANET_HAZARD_AVOIDANCE_H
#define DEMO_PLANET_HAZARD_AVOIDANCE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "chrono/core/ChVector2.h"
#include "chrono/utils/ChConstants.h"

/// Steers a rover toward a goal along a path clear of steep or rough ground.
class PlanetHazardAvoidance {
  public:
    struct Params {
        double cell = 0.25;                                  ///< local costmap resolution (m)
        double window = 20.0;                                ///< local costmap half size around the rover (m)
        double global_cell = 0.5;                            ///< global costmap resolution (m)
        double global_padding = 40.0;                        ///< the global costmap covers start and goal with this around them (m)
        double footprint = 0.75;                             ///< half size of the footprint slope and step are measured over (m)
        double slope_limit = 18 * chrono::CH_DEG_TO_RAD;     ///< steeper ground is a hazard
        double slope_comfort = 8 * chrono::CH_DEG_TO_RAD;    ///< slopes below this cost nothing extra
        double step_limit = 0.2;                             ///< ground departing further from a plane under the footprint is a hazard (m)
        double inflation = 1.2;                              ///< hazards grow by this, the rover's half width and some room (m)
        double margin = 2.5;                                 ///< ground within this of a grown hazard costs extra (m)
        double slope_weight = 2.0;                           ///< extra cost per meter at the slope limit
        double margin_weight = 4.0;                          ///< extra cost per meter right at a grown hazard
        double hazard_cost = 200.0;                          ///< cost per meter inside a grown hazard (finite, so a rover in one can leave)
        double goal_weight = 2.0;                            ///< cost per meter to the goal, outside the global costmap
        double route_bonus = 0.3;                            ///< cost reduction, as a fraction, on the previous plan's path
        double smoothing = 1.0;                              ///< the path is averaged over this length before it is followed (m)
        double lookahead = 2.5;                              ///< pure pursuit lookahead along the path (m)
        double max_steering_rate = 0.6;                      ///< steering changes no faster than this (rad/s)
        double half_wheelbase = 0.64;                        ///< with all four wheels steered, curvature = gain * tan(angle) / half_wheelbase
        double curvature_gain = 0.6;                         ///< turning achieved, as a fraction of the ideal (fitted for VIPER in soft soil)
        double max_steering = chrono::CH_PI / 6;             ///< steering limit (rad)
    };

    /// Result of one plan.
    struct Plan {
        double steering = 0;                   ///< steering angle to apply (rad, positive left)
        double max_slope = 0;                  ///< steepest footprint slope along the path (rad)
        bool blocked = false;                  ///< the path had to cross a hazard
        std::vector<chrono::ChVector2d> path;  ///< planned path from the rover, in the height function's plane (m)
    };

    /// `height(x, y)` is the terrain height at a point in the plane the rover drives in.
    explicit PlanetHazardAvoidance(std::function<double(double, double)> height) : m_height(std::move(height)) {}
    PlanetHazardAvoidance(std::function<double(double, double)> height, const Params& params)
        : m_height(std::move(height)), m_params(params) {}

    /// Set the goal. The global costmap is built on the next update, around that update's position and the goal.
    void SetGoal(const chrono::ChVector2d& goal) {
        m_goal = goal;
        m_global = Global();
    }
    const chrono::ChVector2d& GetGoal() const { return m_goal; }
    const Params& GetParams() const { return m_params; }

    /// Build the global costmap around a position and the goal now, rather than on the next update.
    void BuildRoute(const chrono::ChVector2d& from) { BuildGlobal(from); }

    /// The cheapest route to the goal over the global costmap, from a point, as the global plan sees it.
    std::vector<chrono::ChVector2d> GetGlobalRoute(const chrono::ChVector2d& from) const {
        std::vector<chrono::ChVector2d> route;
        if (m_global.cost_to_go.empty())
            return route;
        const double c = m_params.global_cell;
        int q = GlobalIndex(from);
        while (q >= 0) {
            route.emplace_back((m_global.i0 + q % m_global.nx) * c, (m_global.j0 + q / m_global.nx) * c);
            q = m_global.next[q];
        }
        return route;
    }

    /// Plan from a rover position and heading (rad, from +x toward +y) and return the steering to apply,
    /// `dt` seconds after the previous plan.
    Plan Update(const chrono::ChVector2d& position, double heading, double dt) {
        const Params& p = m_params;
        if (m_global.cost_to_go.empty())
            BuildGlobal(position);

        // Local costmap around the rover.
        const double c = p.cell;
        const int w = static_cast<int>(std::lround(p.window / c));  // window half size, cells
        const int n = 2 * w + 1;
        const int ci = static_cast<int>(std::lround(position.x() / c));
        const int cj = static_cast<int>(std::lround(position.y() / c));
        const Costs local = BuildCosts(ci - w, cj - w, n, n, c, [this](int i, int j) { return H(i, j); });
        std::vector<float> cost = local.cost;
        auto at = [n](int i, int j) { return static_cast<size_t>(j) * n + i; };

        // Favor the previous plan's path, so the route changes only when another is clearly better.
        for (const auto& pt : m_previous_path) {
            const int i = static_cast<int>(std::lround(pt.x() / c)) - (ci - w), j = static_cast<int>(std::lround(pt.y() / c)) - (cj - w);
            if (i >= 0 && j >= 0 && i < n && j < n)
                cost[at(i, j)] *= static_cast<float>(1 - p.route_bonus);
        }

        // Cheapest paths from the rover over the window.
        std::vector<int> parent;
        const std::vector<double> g = ShortestPaths(cost, n, n, c, {static_cast<int>(at(w, w))}, parent);

        // The path's end: the cell that best trades the cost of reaching it against its cost-to-go.
        int best = static_cast<int>(at(w, w));
        double best_score = std::numeric_limits<double>::infinity();
        for (int q = 0; q < n * n; ++q) {
            const double score = g[q] + CostToGo(chrono::ChVector2d((ci - w + q % n) * c, (cj - w + q / n) * c));
            if (score < best_score) {
                best_score = score;
                best = q;
            }
        }

        Plan plan;
        std::vector<int> cells;
        for (int q = best; q >= 0; q = parent[q])
            cells.push_back(q);
        std::reverse(cells.begin(), cells.end());
        for (int q : cells) {
            plan.path.emplace_back((ci - w + q % n) * c, (cj - w + q / n) * c);
            plan.max_slope = std::max(plan.max_slope, static_cast<double>(local.slope[q]));
            plan.blocked = plan.blocked || local.dist[q] < p.inflation;
        }
        m_previous_path = plan.path;

        // Pure pursuit toward the point a lookahead along the smoothed path: grid paths zig-zag, and following
        // their cells makes the steering jump.
        std::vector<chrono::ChVector2d> smooth(plan.path.size());
        const int half = std::max(0, static_cast<int>(std::lround(0.5 * p.smoothing / c)));
        for (size_t q = 0; q < plan.path.size(); ++q) {
            const size_t a = q >= static_cast<size_t>(half) ? q - half : 0, b = std::min(plan.path.size() - 1, q + half);
            chrono::ChVector2d sum(0, 0);
            for (size_t r = a; r <= b; ++r)
                sum += plan.path[r];
            smooth[q] = sum / static_cast<double>(b - a + 1);
        }
        chrono::ChVector2d target = smooth.back();
        double along = 0;
        for (size_t q = 1; q < smooth.size(); ++q) {
            const double seg = (smooth[q] - smooth[q - 1]).Length();
            if (along + seg >= p.lookahead) {
                target = smooth[q - 1] + (p.lookahead - along) / seg * (smooth[q] - smooth[q - 1]);
                break;
            }
            along += seg;
        }
        double steering = m_steering;
        const chrono::ChVector2d to_target = target - position;
        const double ld = to_target.Length();
        if (ld > 1e-6) {
            const double alpha = std::remainder(std::atan2(to_target.y(), to_target.x()) - heading, chrono::CH_2PI);
            const double curvature = std::abs(alpha) < chrono::CH_PI_2 ? 2 * std::sin(alpha) / ld : (alpha > 0 ? 1e3 : -1e3);
            steering = std::clamp(std::atan(curvature * p.half_wheelbase / p.curvature_gain), -p.max_steering, p.max_steering);
        }
        const double max_change = p.max_steering_rate * dt;
        m_steering += std::clamp(steering - m_steering, -max_change, max_change);
        plan.steering = m_steering;
        return plan;
    }

  private:
    /// Per-node terrain costs of a grid.
    struct Costs {
        std::vector<float> slope;  ///< footprint slope (rad)
        std::vector<float> dist;   ///< distance to the nearest hazard (m)
        std::vector<float> cost;   ///< cost per meter of driving through the node
    };

    /// The global costmap's cost-to-go, and the next node on each node's cheapest route to the goal.
    struct Global {
        int i0 = 0, j0 = 0, nx = 0, ny = 0;
        std::vector<double> cost_to_go;
        std::vector<int> next;
    };

    /// Costs of the nx x ny nodes from node (i0, j0) at the given spacing, with heights from height(i, j).
    template <class HeightFn>
    Costs BuildCosts(int i0, int j0, int nx, int ny, double cell, HeightFn height) const {
        const Params& p = m_params;
        const int k = std::max(1, static_cast<int>(std::lround(p.footprint / cell)));
        auto at = [nx](int i, int j) { return static_cast<size_t>(j) * nx + i; };

        // Heights, with a footprint's border around the grid.
        const int hx = nx + 2 * k, hy = ny + 2 * k;
        std::vector<double> h(static_cast<size_t>(hx) * hy);
        for (int j = 0; j < hy; ++j)
            for (int i = 0; i < hx; ++i)
                h[static_cast<size_t>(j) * hx + i] = height(i0 - k + i, j0 - k + j);
        auto hat = [&](int i, int j) { return h[static_cast<size_t>(j + k) * hx + (i + k)]; };

        // Footprint slope and step, and the hazards.
        Costs out;
        out.slope.resize(static_cast<size_t>(nx) * ny);
        std::vector<float> excess(out.slope.size());
        std::vector<char> hazard(out.slope.size());
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const double hc = hat(i, j), hf = hat(i + k, j), hb = hat(i - k, j), hl = hat(i, j + k), hr = hat(i, j - k);
                const double sx = (hf - hb) / (2 * k * cell), sy = (hl - hr) / (2 * k * cell);
                const double s = std::atan(std::sqrt(sx * sx + sy * sy));
                const double step = std::max(std::abs(hc - 0.25 * (hf + hb + hl + hr)), 0.5 * std::abs((hf + hb) - (hl + hr)));
                out.slope[at(i, j)] = static_cast<float>(s);
                excess[at(i, j)] = static_cast<float>(std::clamp((s - p.slope_comfort) / (p.slope_limit - p.slope_comfort), 0.0, 1.0));
                hazard[at(i, j)] = s > p.slope_limit || step > p.step_limit;
            }

        // Distance to the nearest hazard, by a two-pass chamfer transform.
        const float far = std::numeric_limits<float>::max() / 4;
        std::vector<float>& dist = out.dist;
        dist.resize(out.slope.size());
        for (size_t q = 0; q < dist.size(); ++q)
            dist[q] = hazard[q] ? 0.f : far;
        const float d1 = static_cast<float>(cell), d2 = static_cast<float>(cell * std::sqrt(2.0));
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                float& d = dist[at(i, j)];
                if (i > 0)
                    d = std::min(d, dist[at(i - 1, j)] + d1);
                if (j > 0)
                    d = std::min(d, dist[at(i, j - 1)] + d1);
                if (i > 0 && j > 0)
                    d = std::min(d, dist[at(i - 1, j - 1)] + d2);
                if (i + 1 < nx && j > 0)
                    d = std::min(d, dist[at(i + 1, j - 1)] + d2);
            }
        for (int j = ny - 1; j >= 0; --j)
            for (int i = nx - 1; i >= 0; --i) {
                float& d = dist[at(i, j)];
                if (i + 1 < nx)
                    d = std::min(d, dist[at(i + 1, j)] + d1);
                if (j + 1 < ny)
                    d = std::min(d, dist[at(i, j + 1)] + d1);
                if (i + 1 < nx && j + 1 < ny)
                    d = std::min(d, dist[at(i + 1, j + 1)] + d2);
                if (i > 0 && j + 1 < ny)
                    d = std::min(d, dist[at(i - 1, j + 1)] + d2);
            }

        // Cost per meter of driving through each node.
        out.cost.resize(out.slope.size());
        for (size_t q = 0; q < out.cost.size(); ++q) {
            if (dist[q] < p.inflation) {
                out.cost[q] = static_cast<float>(p.hazard_cost);
            } else {
                const double near = std::max(0.0, 1.0 - (dist[q] - p.inflation) / p.margin);
                out.cost[q] = static_cast<float>(1.0 + p.slope_weight * excess[q] + p.margin_weight * near);
            }
        }
        return out;
    }

    /// Cheapest path costs over an 8-connected nx x ny grid from the source nodes, and each node's predecessor.
    static std::vector<double> ShortestPaths(const std::vector<float>& cost, int nx, int ny, double cell, const std::vector<int>& sources,
                                             std::vector<int>& parent) {
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> g(cost.size(), inf);
        parent.assign(cost.size(), -1);
        using Entry = std::pair<double, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        for (int s : sources) {
            g[s] = 0;
            open.push({0.0, s});
        }
        static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        while (!open.empty()) {
            const auto [gq, q] = open.top();
            open.pop();
            if (gq > g[q])
                continue;
            const int qi = q % nx, qj = q / nx;
            for (int e = 0; e < 8; ++e) {
                const int ni = qi + di[e], nj = qj + dj[e];
                if (ni < 0 || nj < 0 || ni >= nx || nj >= ny)
                    continue;
                const int r = nj * nx + ni;
                const double len = e < 4 ? cell : cell * std::sqrt(2.0);
                const double gr = gq + len * 0.5 * (cost[q] + cost[r]);
                if (gr < g[r]) {
                    g[r] = gr;
                    parent[r] = q;
                    open.push({gr, r});
                }
            }
        }
        return g;
    }

    /// Build the global costmap over the corridor from a position to the goal, and its cost-to-go.
    void BuildGlobal(const chrono::ChVector2d& from) {
        const double c = m_params.global_cell, pad = m_params.global_padding;
        m_global.i0 = static_cast<int>(std::floor((std::min(from.x(), m_goal.x()) - pad) / c));
        m_global.j0 = static_cast<int>(std::floor((std::min(from.y(), m_goal.y()) - pad) / c));
        m_global.nx = static_cast<int>(std::ceil((std::max(from.x(), m_goal.x()) + pad) / c)) - m_global.i0 + 1;
        m_global.ny = static_cast<int>(std::ceil((std::max(from.y(), m_goal.y()) + pad) / c)) - m_global.j0 + 1;
        const Costs costs = BuildCosts(m_global.i0, m_global.j0, m_global.nx, m_global.ny, c,
                                       [this, c](int i, int j) { return m_height(i * c, j * c); });
        m_global.cost_to_go = ShortestPaths(costs.cost, m_global.nx, m_global.ny, c, {GlobalIndex(m_goal)}, m_global.next);
    }

    /// Nearest global costmap node to a point, clamped to the map.
    int GlobalIndex(const chrono::ChVector2d& pt) const {
        const double c = m_params.global_cell;
        const int i = std::clamp(static_cast<int>(std::lround(pt.x() / c)) - m_global.i0, 0, m_global.nx - 1);
        const int j = std::clamp(static_cast<int>(std::lround(pt.y() / c)) - m_global.j0, 0, m_global.ny - 1);
        return j * m_global.nx + i;
    }

    /// Cost to reach the goal from a point: from the global costmap, or by distance outside it.
    double CostToGo(const chrono::ChVector2d& pt) const {
        const double c = m_params.global_cell;
        const double fi = pt.x() / c - m_global.i0, fj = pt.y() / c - m_global.j0;
        if (m_global.cost_to_go.empty() || fi < 0 || fj < 0 || fi > m_global.nx - 1 || fj > m_global.ny - 1)
            return m_params.goal_weight * (m_goal - pt).Length();
        return m_global.cost_to_go[GlobalIndex(pt)];
    }

    // Terrain height at a local costmap node, sampled once.
    double H(int i, int j) {
        const int64_t key = (static_cast<int64_t>(i) << 32) ^ static_cast<uint32_t>(j);
        auto it = m_heights.find(key);
        if (it != m_heights.end())
            return it->second;
        const double h = m_height(i * m_params.cell, j * m_params.cell);
        m_heights.emplace(key, h);
        return h;
    }

    std::function<double(double, double)> m_height;
    Params m_params;
    chrono::ChVector2d m_goal = chrono::ChVector2d(1e6, 0);
    Global m_global;
    std::unordered_map<int64_t, double> m_heights;
    std::vector<chrono::ChVector2d> m_previous_path;
    double m_steering = 0;
};

#endif
