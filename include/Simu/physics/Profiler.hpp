////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2023 Matthieu Beauchamp-Boulay
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

#include <chrono>

#include "Simu/config.hpp"

namespace simu
{

class TimeEntry
{
public:

    typedef std::chrono::steady_clock  Clock;
    typedef typename Clock::time_point TimePoint;
    typedef typename Clock::duration   Duration;

    typedef std::chrono::duration<double> Seconds;

    TimeEntry() = default;

    double last() const { return last_.count(); }
    double average() const { return average_.count(); }
    double max() const { return max_.count(); }

    Int64 nMeasures() const { return nMeasures_; }

    class ScopedTimer;
    inline [[nodiscard]] ScopedTimer time();

    void                             startPartial() { last_ = Seconds{0.0}; }
    inline [[nodiscard]] ScopedTimer timePartial();
    void                             commitPartial() { addMeasure(last_); }

private:

    void addMeasure(Seconds seconds)
    {
        last_ = seconds;
        max_  = std::max(max_, seconds);

        double total = average_.count() * nMeasures_;
        average_     = Seconds{(total + seconds.count()) / ++nMeasures_};
    }

    Seconds last_{0.0};
    Seconds average_{0.0};
    Seconds max_{0.0};

    Int64 nMeasures_ = 0;
};

class TimeEntry::ScopedTimer
{
#if defined(SIMU_PROFILE)

public:

    ScopedTimer(TimeEntry& entry, bool partial)
        : entry_{&entry}, start_{Clock::now()}, partial_{partial}
    {
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&& other)
    {
        std::swap(entry_, other.entry_);
        entry_   = other.entry_;
        partial_ = other.partial_;
    }

    ~ScopedTimer()
    {
        if (entry_ != nullptr)
        {
            Duration dt = Clock::now() - start_;
            if (partial_)
                entry_->last_ += dt;
            else
                entry_->addMeasure(Seconds{dt});
        }
    }

private:

    TimeEntry* entry_ = nullptr;
    TimePoint  start_;
    bool       partial_;

#endif
};

typedef typename TimeEntry::ScopedTimer ScopedTimer;

ScopedTimer TimeEntry::time() { return ScopedTimer{*this, false}; }
ScopedTimer TimeEntry::timePartial() { return ScopedTimer{*this, true}; }


#if defined(SIMU_PROFILE)
#    define SIMU_PROFILE_ENTRY(timeEntry)                                      \
        ScopedTimer simu_timer { timeEntry.time() }
#    define SIMU_PROFILE_START_PARTIAL_ENTRY(timeEntry) timeEntry.startPartial()
#    define SIMU_PROFILE_PARTIAL_ENTRY(timeEntry)                              \
        ScopedTimer simu_timer { timeEntry.timePartial() }
#    define SIMU_PROFILE_COMMIT_PARTIAL_ENTRY(timeEntry)                       \
        timeEntry.commitPartial()

#    define SIMU_PROFILE_TREE_HEIGHT(profiler, tree)                           \
        profiler.treeHeight = tree.height()
#else
#    define SIMU_PROFILE_ENTRY(timeEntry)                (void)timeEntry
#    define SIMU_PROFILE_START_PARTIAL_ENTRY(timeEntry)  (void)timeEntry
#    define SIMU_PROFILE_PARTIAL_ENTRY(timeEntry)        (void)timeEntry
#    define SIMU_PROFILE_COMMIT_PARTIAL_ENTRY(timeEntry) (void)timeEntry
#    define SIMU_PROFILE_TREE_HEIGHT(profiler, tree)     profiler.treeHeight = 0
#endif

struct Profiler
{
    Profiler() = default;
    void reset() { *this = Profiler{}; }

    TimeEntry treeUpdateAndCollision{};
    TimeEntry narrowPhaseCollision{};
    Uint32    treeHeight{};

    TimeEntry islandConstruction{};
    TimeEntry solveVelocities{};
    TimeEntry solvePositions{};
};


} // namespace simu
