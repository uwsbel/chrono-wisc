#ifndef CH_PLANET_PARALLEL_H
#define CH_PLANET_PARALLEL_H

#include "chrono_planet/ChApiPlanet.h"

// OpenMP helpers for the tile build's grid passes. No-ops without OpenMP.

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <utility>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace chrono {
namespace planet {

namespace util {

// Grids below this run on the calling thread.
constexpr size_t kParallelMinSamples = 128 * 128;

// True when a grid of samples is large enough to be worth a team.
inline bool parallelGrid(size_t samples) { return samples >= kParallelMinSamples; }

// Physical cores from sysfs, or half the logical CPUs where that is unreadable.
inline int physicalCores() {
#ifdef _OPENMP
    const int logical = std::max(1, omp_get_num_procs());
#else
    const int logical = 1;
#endif
    std::set<std::pair<int, int>> cores;   // (package, core)
    for (int cpu = 0; cpu < logical; ++cpu) {
        const std::string base = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/";
        std::ifstream pkg(base + "physical_package_id"), core(base + "core_id");
        int p = 0, c = 0;
        if (!(pkg >> p) || !(core >> c)) {
            return std::max(1, logical / 2);
        }
        cores.insert({p, c});
    }
    return std::max(1, static_cast<int>(cores.size()));
}

// Team size. OMP_NUM_THREADS if set, else one thread per physical core.
inline int parallelThreads() {
#ifdef _OPENMP
    static const int team = [] {
        const int want = std::max(1, omp_get_max_threads());
        if (std::getenv("OMP_NUM_THREADS") != nullptr) {
            return want;
        }
        const char* wait = std::getenv("OMP_WAIT_POLICY");
        if (wait != nullptr && std::string(wait) == "passive") {
            return want;
        }
        return std::max(1, std::min(want, physicalCores()));
    }();
    return team;
#else
    return 1;
#endif
}

}  // namespace util

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_PARALLEL_H
