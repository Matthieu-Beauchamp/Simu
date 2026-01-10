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

#pragma once

#include "tracy/Tracy.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <array>
#include <chrono>
#include <ostream>
#include <ranges>
#include <unordered_map>

/////////////////////////////////////////////////
// Time tracking

struct Stats
{
    uint64_t count = 0;
    double   sum   = 0.0;
    double   sumSq = 0.0;
    double   min   = std::numeric_limits<double>::infinity();
    double   max   = 0.0;

    static constexpr int            HIST_BINS = 64;
    std::array<uint32_t, HIST_BINS> hist      = {};

    void record(double value) {
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
};

struct DerivedStats
{
    double mean;
    double stddev;
    double cv;
    double p50;
    double p95;
    double p99;
};

inline double percentile_from_hist(const Stats& s, double percentile) {
    if (s.count == 0)
        return 0.0;

    uint64_t target = static_cast<uint64_t>(std::ceil(percentile * s.count));

    uint64_t cumulative = 0;
    for (int i = 0; i < Stats::HIST_BINS; ++i) {
        cumulative += s.hist[i];
        if (cumulative >= target) {
            return std::pow(2.0, i);
        }
    }
    return s.max;
}

inline DerivedStats compute_derived(const Stats& s) {
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


/////////////////////////////////////////////////
// Memory tracking

struct MemoryCounters
{
    int64_t allocated = 0;
    int64_t freed     = 0;
    int64_t live      = 0;
    int64_t peak      = 0;

    int64_t num_allocations = 0;
};

thread_local MemoryCounters tls_mem;

void record_alloc(size_t bytes) {
    tls_mem.allocated += bytes;
    tls_mem.live += bytes;
    tls_mem.peak = std::max(tls_mem.peak, tls_mem.live);
    tls_mem.num_allocations++;
}

void record_free(size_t bytes) {
    tls_mem.freed += bytes;
    tls_mem.live -= bytes;
}


/////////////////////////////////////////////////
// Aggregation

struct CumulativeStats
{
    int64_t time = 0;

    int64_t mem_usage       = 0;
    int64_t mem_allocations = 0;

    int64_t num_calls = 0;
};

thread_local std::unordered_map<const char*, CumulativeStats> tls_accumulator;

struct ProfilerStats
{
    Stats time;
    Stats mem_usage;
    Stats mem_allocations;
    Stats num_calls;
};

thread_local std::unordered_map<const char*, ProfilerStats> tls_stats;


/////////////////////////////////////////////////
// Scope

struct ProfileScope
{
    const char*    name;
    MemoryCounters mem_start;
    auto           time_start = std::chrono::steady_clock::now();

    [[nodiscard]] ProfileScope(const char* name)
        : name(name), mem_start(tls_mem) {}

    ~ProfileScope() {
        MemoryCounters mem_end  = tls_mem;
        auto           time_end = std::chrono::steady_clock::now();

        auto& cumulative = tls_accumulator[name];
        cumulative.time += time_end - time_start;
        cumulative.mem_usage += mem_end.allocated - mem_start.allocated;
        cumulative.mem_allocations += mem_end.num_allocations - mem_start.num_allocations;
        cumulative.num_calls++;
    }
};

// TODO: Add bvh tree size tracking

struct FrameScope
{
    [[nodiscard]] FrameScope() {
        // Empty on the first frame, since no profile scope wrote anything -> skipped
        for (const auto& it : tls_accumulator) {
            tls_stats[it.first].time.record(static_cast<double>(it.second.time));
            tls_stats[it.first].mem_usage.record(static_cast<double>(it.second.mem_usage));
            tls_stats[it.first].mem_allocations.record(
                static_cast<double>(it.second.mem_allocations)
            );
            tls_stats[it.first].num_calls.record(static_cast<double>(it.second.num_calls));
        }

        for (auto& stats : tls_accumulator | std::ranges::views::values) {
            stats = CumulativeStats{};
        }
    }
};

/////////////////////////////////////////////////
// Output

inline void write_csv_header(std::ostream& os) {
    os << "name,count,mean,stddev,cv,min,p50,p95,p99,max\n";
}

inline void write_csv_row(std::ostream& os, const std::string& name, const Stats& s) {
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

inline void write_profiled_data(std::ostream& os) {
    write_csv_header(os);
    for (const auto& [name, stats] : tls_stats) {
        write_csv_row(os, std::string(name) + " time (us)", stats.time);
        write_csv_row(os, std::string(name) + " mem usage (B)", stats.mem_usage);
        write_csv_row(os, std::string(name) + " mem allocations", stats.mem_allocations);
        write_csv_row(os, std::string(name) + " num calls", stats.num_calls);
    }
}
