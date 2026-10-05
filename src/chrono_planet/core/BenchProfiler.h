#ifndef CH_PLANET_BENCH_PROFILER_H
#define CH_PLANET_BENCH_PROFILER_H

#include "chrono_planet/ChApiPlanet.h"

// Benchmark instrumentation. Prof is a per-thread hierarchical scope timer reported as folded stacks,
// Counters are flat per-frame tallies and live gauges. Both are inert until profiling() is set.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace chrono {
namespace planet {

namespace bench {

// Master switch, read once per scope.
std::atomic<bool>& profiling();

using Clock = std::chrono::steady_clock;

// One scope in a thread's timing tree, with inclusive time and call count.
struct CH_PLANET_API ProfNode {
    const char* name;
    ProfNode* parent;
    std::uint64_t ns = 0;   // inclusive
    std::uint64_t calls = 0;
    std::vector<ProfNode*> kids;
    ProfNode(const char* n, ProfNode* p) : name(n), parent(p) {}
};

// One timing tree per thread.
struct CH_PLANET_API ThreadProf {
    std::mutex m;
    std::deque<ProfNode> pool;   // stable addresses, never shrinks
    ProfNode* root = nullptr;
    ProfNode* cur = nullptr;
    const char* label = "thread";
    ThreadProf();
};

// Folded-stack entry, one line of "a;b;c <selfNs>".
struct CH_PLANET_API Folded {
    std::string path;
    std::uint64_t selfNs = 0;
    std::uint64_t inclNs = 0;
    std::uint64_t calls = 0;
};

// The hierarchical scope timer. A singleton.
class CH_PLANET_API Prof {
public:
    // The process-wide profiler.
    static Prof& get();
    Prof(const Prof&) = delete;
    Prof& operator=(const Prof&) = delete;

    // Names this thread's root. Threads sharing a label merge into one tree.
    void setLabel(const char* l);
    // Enters child scope name under this thread's current node.
    void push(const char* name);
    // Leaves the current scope, crediting it ns inclusive.
    void pop(std::uint64_t ns);

    // Adds time measured outside the scope stack under a full folded path such as gpu;terrain.
    void addRaw(const std::string& path, std::uint64_t ns);

    // Freezes the current totals, so folded() reports only what accumulates after this call.
    void mark();

    // Folded stacks since mark(). Order is not significant.
    std::vector<Folded> folded() const;

private:
    using Totals = std::map<std::string, std::pair<std::uint64_t, std::uint64_t>>;   // path -> (ns, calls)

    Prof() = default;

    // This thread's tree, created on first use.
    ThreadProf* self();
    ThreadProf* create();
    std::vector<Folded> collect() const;   // every thread tree plus raw entries

    mutable std::mutex regM_;
    std::vector<std::unique_ptr<ThreadProf>> threads_;
    mutable std::mutex rawM_;
    Totals raw_;
    Totals baseline_;
};

// RAII scope that enters name on construction and leaves on destruction. name must be a string literal.
struct CH_PLANET_API Scope {
    Clock::time_point t0;
    bool on;
    explicit Scope(const char* name);
    ~Scope();
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
};

#define BENCH_CAT2(a, b) a##b
#define BENCH_CAT(a, b) BENCH_CAT2(a, b)
// Times the rest of the enclosing block under name.
#define BENCH_SCOPE(name) ::chrono::planet::bench::Scope BENCH_CAT(bench_scope_, __LINE__)(name)

// Per-frame tallies and live gauges.
struct CH_PLANET_API Counters {
    // Reset every frame by resetFrame().
    std::atomic<long long> drawCalls{0}, triangles{0}, texBinds{0}, uniformCalls{0};
    std::atomic<long long> meshUploadBytes{0};
    std::atomic<long long> splitsDone{0}, splitsDeferred{0}, merges{0};
    std::atomic<long long> syncBuildNs{0};   // time spent building meshes synchronously
    std::atomic<long long> culledFrustum{0}, culledHorizon{0};
    std::atomic<long long> jobsStarted{0}, jobsFinished{0};
    std::atomic<long long> jobNsTotal{0}, jobNsMax{0};

    // Live gauges. Never reset, they describe the current residency.
    std::atomic<long long> gpuMeshBytes{0}, gpuBakeBytes{0};
    std::atomic<long long> residentMeshes{0}, residentBakes{0}, jobsInFlight{0};

    // Zeroes the per-frame tallies. The gauges are left alone.
    void resetFrame();
};

// The process-wide counters.
Counters& ctr();

// Adds to a counter. Relaxed, so not a synchronization point.
inline void bump(std::atomic<long long>& c, long long by = 1) { c.fetch_add(by, std::memory_order_relaxed); }
// Relaxed atomic load.
inline long long read(const std::atomic<long long>& c) { return c.load(std::memory_order_relaxed); }
// Lifts c to v if v is larger.
void bumpMax(std::atomic<long long>& c, long long v);

}   // namespace bench

}  // namespace planet
}  // namespace chrono

#endif   // CH_PLANET_BENCH_PROFILER_H
