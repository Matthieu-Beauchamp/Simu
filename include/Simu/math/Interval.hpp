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

#include "Simu/config.hpp"
#include <cmath>

namespace simu
{


////////////////////////////////////////////////////////////
/// \ingroup math
/// \brief Interval or range of values
///
/// The min and max of the interval are always included.
/// if min > max, then the interval is degenerate and contains nothing.
/// if min == max, the interval contains a single value.
///
////////////////////////////////////////////////////////////
template <class T>
class Interval
{
public:

    Interval(T min, T max) : min_{min}, max_{max} {}

    ////////////////////////////////////////////////////////////
    /// Returns bool for most types.
    /// Returns a ComparisonMatrix if T is a Matrix type.
    ////////////////////////////////////////////////////////////
    auto contains(T value) const { return min_ <= value && value <= max_; }

    auto overlaps(const Interval& other) const
    {
        // divisions by 2 can be factorised out and removed from both sides.
        // Geometrical interpretation is easier with:
        //
        // span = (max - min)/2 + (other.max - other.min)/2
        //      is the sum of half widths
        //
        // dist = std::abs((max_ + min_)/2 - (other.max_ + other.min_)/2)
        //      is the absolute distance between midpoints.
        //
        // we have intersection if
        // dist <= span, or equivalently if 2*dist <= 2*span to remove divisions

        T span = (max_ - min_) + (other.max_ - other.min_);
        T dist = std::abs((max_ + min_) - (other.max_ + other.min_));

        return dist <= span;
    }

private:

    T min_;
    T max_;
};

////////////////////////////////////////////////////////////
/// \relates Interval
/// \brief defines an interval for the approximation: value +/- epsilon
///
////////////////////////////////////////////////////////////
template <class T>
Interval<T> approx(T value, T epsilon)
{
    epsilon = std::abs(epsilon);
    return Interval<T>{value - epsilon, value + epsilon};
}

} // namespace simu
