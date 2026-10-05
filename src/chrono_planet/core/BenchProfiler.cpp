#include "chrono_planet/core/BenchProfiler.h"

namespace chrono {
namespace planet {

namespace bench {

std::atomic<bool>& profiling() {
    static std::atomic<bool> f{false};
    return f;
}

ThreadProf::ThreadProf() {
    pool.emplace_back("", nullptr);
    root = cur = &pool.back();
}

Prof& Prof::get() {
    static Prof p;
    return p;
}

void Prof::setLabel(const char* l) { self()->label = l; }

void Prof::push(const char* name) {
    ThreadProf* t = self();
    std::lock_guard<std::mutex> g(t->m);
    ProfNode* n = nullptr;
    for (ProfNode* k : t->cur->kids) {
        if (k->name == name) {
            n = k;
            break;
        }
    }
    if (!n) {
        t->pool.emplace_back(name, t->cur);
        n = &t->pool.back();
        t->cur->kids.push_back(n);
    }
    t->cur = n;
}

void Prof::pop(std::uint64_t ns) {
    ThreadProf* t = self();
    std::lock_guard<std::mutex> g(t->m);
    if (!t->cur->parent) {
        return;   // unbalanced pop, ignore
    }
    t->cur->ns += ns;
    t->cur->calls += 1;
    t->cur = t->cur->parent;
}

void Prof::addRaw(const std::string& path, std::uint64_t ns) {
    std::lock_guard<std::mutex> g(rawM_);
    auto& e = raw_[path];
    e.first += ns;
    e.second += 1;
}

void Prof::mark() {
    baseline_.clear();
    for (const Folded& f : collect()) {
        baseline_[f.path] = {f.inclNs, f.calls};
    }
}

std::vector<Folded> Prof::folded() const {
    // Inclusive totals since the mark.
    Totals incl;
    for (const Folded& f : collect()) {
        auto it = baseline_.find(f.path);
        std::uint64_t ns = f.inclNs, calls = f.calls;
        if (it != baseline_.end()) {
            ns = ns > it->second.first ? ns - it->second.first : 0;
            calls = calls > it->second.second ? calls - it->second.second : 0;
        }
        incl[f.path] = {ns, calls};
    }
    // Self is inclusive minus the direct children, the contiguous entries one separator deeper.
    std::vector<Folded> out;
    out.reserve(incl.size());
    for (const auto& kv : incl) {
        std::uint64_t kids = 0;
        const std::string pre = kv.first + ';';
        for (auto it = incl.lower_bound(pre); it != incl.end(); ++it) {
            if (it->first.compare(0, pre.size(), pre) != 0) {
                break;
            }
            if (it->first.find(';', pre.size()) == std::string::npos) {
                kids += it->second.first;
            }
        }
        Folded f;
        f.path = kv.first;
        f.inclNs = kv.second.first;
        f.calls = kv.second.second;
        f.selfNs = kv.second.first > kids ? kv.second.first - kids : 0;
        out.push_back(std::move(f));
    }
    return out;
}

ThreadProf* Prof::self() {
    thread_local ThreadProf* t = create();
    return t;
}

ThreadProf* Prof::create() {
    std::lock_guard<std::mutex> g(regM_);
    threads_.push_back(std::make_unique<ThreadProf>());
    return threads_.back().get();
}

namespace {
void walk(const ProfNode* n, const std::string& prefix, std::vector<Folded>& out) {
    for (const ProfNode* k : n->kids) {
        const std::string path = prefix + ';' + k->name;
        Folded f;
        f.path = path;
        f.inclNs = k->ns;
        f.calls = k->calls;
        out.push_back(std::move(f));
        walk(k, path, out);
    }
}
}   // namespace

std::vector<Folded> Prof::collect() const {
    std::vector<Folded> out;
    std::lock_guard<std::mutex> g(regM_);
    for (const auto& tp : threads_) {
        std::lock_guard<std::mutex> tg(tp->m);
        walk(tp->root, tp->label, out);
    }
    std::lock_guard<std::mutex> rg(rawM_);
    for (const auto& kv : raw_) {
        Folded f;
        f.path = kv.first;
        f.inclNs = kv.second.first;
        f.calls = kv.second.second;
        out.push_back(std::move(f));
    }
    return out;
}

Scope::Scope(const char* name) : on(profiling().load(std::memory_order_relaxed)) {
    if (on) {
        Prof::get().push(name);
        t0 = Clock::now();
    }
}

Scope::~Scope() {
    if (on) {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        Prof::get().pop(static_cast<std::uint64_t>(ns));
    }
}

void Counters::resetFrame() {
    for (std::atomic<long long>* c :
         {&drawCalls, &triangles, &texBinds, &uniformCalls, &meshUploadBytes,
          &splitsDone, &splitsDeferred, &merges, &syncBuildNs, &culledFrustum, &culledHorizon, &jobsStarted,
          &jobsFinished, &jobNsTotal, &jobNsMax}) {
        c->store(0, std::memory_order_relaxed);
    }
}

Counters& ctr() {
    static Counters c;
    return c;
}

void bumpMax(std::atomic<long long>& c, long long v) {
    long long cur = c.load(std::memory_order_relaxed);
    while (v > cur && !c.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
    }
}

}   // namespace bench

}  // namespace planet
}  // namespace chrono
