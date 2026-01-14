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


#if defined(SIMU_ENABLE_TRACY)

#    define TRACY_ENABLE

#    include "tracy/Tracy.hpp"

#    define SIMU_PROFILE_FRAME            FrameMark
#    define SIMU_PROFILE_SCOPE(name)      ZoneScopedN(name)

#    define SIMU_PROFILE_ALLOC(ptr, size) TracyAlloc(ptr, size)
#    define SIMU_PROFILE_FREE(ptr)        TracyFree(ptr)


#elif defined(SIMU_CUSTOM_PROFILER)

#    include <chrono>
#    include <array>
#    include <limits>
#    include <iosfwd>

#    define SIMU_PROFILER_VAR(x) auto x##__LINE__

#    define SIMU_PROFILE_FRAME                                                 \
        SIMU_PROFILER_VAR(__simu_profiler_frame) = simu::profiler::FrameScope {}
#    define SIMU_PROFILE_SCOPE(name)                                           \
        SIMU_PROFILER_VAR(__simu_profiler_scope) = simu::profiler::ProfileScope(name)

#    define SIMU_PROFILE_ALLOC(ptr, size)                                      \
        simu::profiler::record_alloc(ptr, size)
#    define SIMU_PROFILE_FREE(ptr) simu::profiler::record_free(ptr)

#    define SIMU_RESET_PROFILER    simu::profiler::reset()

namespace simu::profiler
{

void reset();

/////////////////////////////////////////////////
// Time tracking

struct Stats
{
    static constexpr int HIST_BINS = 64;

    std::uint64_t count = 0;
    double        mean   = 0.0;
    double        M2    = 0.0;
    double        min   = std::numeric_limits<double>::infinity();
    double        max   = 0.0;

    void record(double value);
};

struct DerivedStats
{
    double mean;
    double stddev;
    double cv;
};

DerivedStats compute_derived(const Stats& s);


/////////////////////////////////////////////////
// Memory tracking

struct MemoryCounters
{
    std::int64_t allocated = 0;
    std::int64_t freed     = 0;
    std::int64_t live      = 0;
    std::int64_t peak      = 0;

    std::int64_t num_allocations = 0;
};

void record_alloc(void* ptr, size_t bytes);
void record_free(void* ptr);


/////////////////////////////////////////////////
// Aggregation

struct CumulativeStats
{
    std::int64_t time = 0;

    std::int64_t mem_usage       = 0;
    std::int64_t mem_allocations = 0;

    std::int64_t num_calls = 0;
};

struct ProfilerStats
{
    Stats time;
    Stats mem_usage;
    Stats mem_allocations;
    Stats num_calls;
};


/////////////////////////////////////////////////
// Scope

struct ProfileScope
{
    const char*                                        name;
    MemoryCounters                                     mem_start;
    std::chrono::time_point<std::chrono::steady_clock> time_start
        = std::chrono::steady_clock::now();

    [[nodiscard]] ProfileScope(const char* name);

    ~ProfileScope();
};

// TODO: Add bvh tree size tracking

struct FrameScope
{
    [[nodiscard]] FrameScope();
    ~FrameScope() = default;
};

/////////////////////////////////////////////////
// Output

void write_profiled_data(std::ostream& os);

} // namespace simu::profiler

#else

#    define SIMU_PROFILE_FRAME
#    define SIMU_PROFILE_SCOPE(name)

#endif
