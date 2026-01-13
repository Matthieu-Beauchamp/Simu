////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

#include "profiler.hpp"

#if defined(SIMU_ENABLE_TRACY) || defined(SIMU_CUSTOM_PROFILER)

#    include <cstdlib>
#    include <new>


// See https://en.cppreference.com/w/cpp/memory/new/operator_new.html

// no inline, required by [replacement.functions]/3
void* operator new(std::size_t sz) {
    if (sz == 0)
        ++sz; // avoid std::malloc(0) which may return nullptr on success

    if (void* ptr = std::malloc(sz)) {
        SIMU_PROFILE_ALLOC(ptr, sz);
        return ptr;
    }

    throw std::bad_alloc{}; // required by [new.delete.single]/3
}

// no inline, required by [replacement.functions]/3
void* operator new[](std::size_t sz) {
    if (sz == 0)
        ++sz; // avoid std::malloc(0) which may return nullptr on success

    if (void* ptr = std::malloc(sz)) {
        SIMU_PROFILE_ALLOC(ptr, sz);
        return ptr;
    }

    throw std::bad_alloc{}; // required by [new.delete.single]/3
}

void operator delete(void* ptr) noexcept {
    SIMU_PROFILE_FREE(ptr);
    std::free(ptr);
}

void operator delete(void* ptr, [[maybe_unused]] std::size_t size) noexcept {
    SIMU_PROFILE_FREE(ptr);
    std::free(ptr);
}

void operator delete[](void* ptr) noexcept {
    SIMU_PROFILE_FREE(ptr);
    std::free(ptr);
}

void operator delete[](void* ptr, [[maybe_unused]] std::size_t size) noexcept {
    SIMU_PROFILE_FREE(ptr);
    std::free(ptr);
}

#endif

#if defined(SIMU_CUSTOM_PROFILER)

#    include <unordered_map>
#    include <ranges>


namespace
{

double percentile_from_hist(const simu::profiler::Stats& s, double percentile) {
    if (s.count == 0)
        return 0.0;

    uint64_t target = static_cast<uint64_t>(std::ceil(percentile * s.count));

    uint64_t cumulative = 0;
    for (int i = 0; i < simu::profiler::Stats::HIST_BINS; ++i) {
        cumulative += s.hist[i];
        if (cumulative >= target) {
            return std::pow(2.0, i);
        }
    }
    return s.max;
}

void write_csv_header(std::ostream& os) {
    os << "name,count,mean,stddev,cv,min,p50,p95,p99,max\n";
}

void write_csv_row(std::ostream& os, const std::string& name, const simu::profiler::Stats& s) {
    auto d = compute_derived(s);

    // clang-format off
    os << name << ","
       << s.count << ","
       << d.mean << ","
       << d.stddev << ","
       << d.cv << ","
       << s.min << ","
       << d.p50 << ","
       << d.p95 << ","
       << d.p99 << ","
       << s.max << "\n";
    // clang-format on
}

} // namespace

namespace simu::profiler
{

template <typename T>
struct malloc_allocator
{
    using value_type                             = T;
    using pointer                                = T*;
    using const_pointer                          = const T*;
    using reference                              = T&;
    using const_reference                        = const T&;
    using size_type                              = std::size_t;
    using difference_type                        = std::ptrdiff_t;
    using propagate_on_container_move_assignment = std::true_type;

    malloc_allocator() = default;

    template <typename U>
    malloc_allocator(const malloc_allocator<U>&) {}

    pointer allocate(size_type n) {
        return static_cast<pointer>(std::malloc(n * sizeof(T)));
    }

    void deallocate(pointer p, size_type) { std::free(p); }
};

// Allocator with untracked allocations
template <class K, class V>
using profiler_map
    = std::unordered_map<K, V, std::hash<K>, std::equal_to<K>, malloc_allocator<std::pair<const K, V>>>;

static profiler_map<void*, std::size_t> allocations;
static MemoryCounters                   mem_counters;


static profiler_map<const char*, CumulativeStats> stats_accumulator;
static profiler_map<const char*, ProfilerStats>   profiler_stats;

void reset() {
    allocations.clear();
    mem_counters = MemoryCounters{};
    stats_accumulator.clear();
    profiler_stats.clear();
}

void Stats::record(double value) {
    count++;
    sum += value;
    sumSq += value * value;
    min = std::min(min, value);
    max = std::max(max, value);

    // Log2 histogram (assumes value > 0)
    int bin = 0;
    if (value > 0.0) {
        bin = static_cast<int>(std::log2(value));
        if (bin < 0)
            bin = 0;
        if (bin >= HIST_BINS)
            bin = HIST_BINS - 1;
    }
    hist[bin]++;
}

DerivedStats compute_derived(const Stats& s) {
    DerivedStats d{};

    if (s.count == 0)
        return d;

    d.mean          = s.sum / s.count;
    double variance = (s.sumSq / s.count) - (d.mean * d.mean);

    d.stddev = variance > 0.0 ? std::sqrt(variance) : 0.0;
    d.cv     = (d.mean > 0.0) ? d.stddev / d.mean : 0.0;

    d.p50 = percentile_from_hist(s, 0.50);
    d.p95 = percentile_from_hist(s, 0.95);
    d.p99 = percentile_from_hist(s, 0.99);

    return d;
}

void record_alloc(void* ptr, size_t bytes) {
    allocations[ptr] = bytes;
    mem_counters.allocated += bytes;
    mem_counters.live += bytes;
    mem_counters.peak = std::max(mem_counters.peak, mem_counters.live);
    mem_counters.num_allocations++;
}

void record_free(void* ptr) {
    std::size_t bytes = allocations[ptr];
    allocations.erase(ptr);

    mem_counters.freed += bytes;
    mem_counters.live -= bytes;
}


ProfileScope::ProfileScope(const char* name)
    : name(name), mem_start(mem_counters) {}

ProfileScope::~ProfileScope() {
    MemoryCounters mem_end  = mem_counters;
    auto           time_end = std::chrono::steady_clock::now();

    auto& cumulative = stats_accumulator[name];
    cumulative.time += std::chrono::nanoseconds(time_end - time_start).count();
    cumulative.mem_usage += mem_end.allocated - mem_start.allocated;
    cumulative.mem_allocations += mem_end.num_allocations - mem_start.num_allocations;
    cumulative.num_calls++;
}

FrameScope::FrameScope() {
    // Empty on the first frame, since no profile scope wrote anything -> skipped
    for (const auto& it : stats_accumulator) {
        auto& stats = profiler_stats[it.first];
        stats.time.record(static_cast<double>(it.second.time));
        stats.mem_usage.record(static_cast<double>(it.second.mem_usage));
        stats.mem_allocations.record(static_cast<double>(it.second.mem_allocations));
        stats.num_calls.record(static_cast<double>(it.second.num_calls));
    }

    for (auto& stats : stats_accumulator | std::ranges::views::values) {
        stats = CumulativeStats{};
    }
}

void write_profiled_data(std::ostream& os) {
    write_csv_header(os);
    for (const auto& [name, stats] : profiler_stats) {
        write_csv_row(os, std::string(name) + " time (ns)", stats.time);
        write_csv_row(os, std::string(name) + " mem usage (B)", stats.mem_usage);
        write_csv_row(os, std::string(name) + " mem allocations", stats.mem_allocations);
        write_csv_row(os, std::string(name) + " num calls", stats.num_calls);
    }
}

} // namespace simu::profiler

#endif
