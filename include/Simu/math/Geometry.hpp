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

#include <concepts>
#include <vector>

#include "Simu/math/Matrix.hpp"
#include "Simu/math/Interval.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \defgroup Geometry
/// \ingroup math
///
/// \{
////////////////////////////////////////////////////////////

// clang-format off

////////////////////////////////////////////////////////////
/// \brief Any iterator that dereferences to a vector of dimension dim
/// 
////////////////////////////////////////////////////////////
template <class T>
concept VertexIterator = requires(T it) {
    { *it } -> std::convertible_to<Vec2>;
} && std::forward_iterator<T>;


////////////////////////////////////////////////////////////
/// \brief Any type that allows iterating over vertices
/// 
////////////////////////////////////////////////////////////
template<class T>
concept Geometry = requires(T geo){
    { geo.begin() } -> VertexIterator;
    { geo.end() }   -> VertexIterator;
};
// clang-format on


////////////////////////////////////////////////////////////
/// \brief The iterator type returned by T::begin()
///
////////////////////////////////////////////////////////////
template <class T>
using IteratorOf = decltype(std::declval<T>().begin());


////////////////////////////////////////////////////////////
/// \brief Orientation or winding order of a chain of vertices
///
////////////////////////////////////////////////////////////
enum class Orientation
{
    positive,
    collinear,
    negative
};

////////////////////////////////////////////////////////////
/// \brief Find the Orientation of 3 vertices
///
/// epsilon is used to approximate collinearity
////////////////////////////////////////////////////////////
Orientation orientation(Vec2 v0, Vec2 v1, Vec2 v2, float epsilon = simu::EPSILON)
{
    float c = cross(v1 - v0, v2 - v1);

    if (approx(0.f, epsilon).contains(c))
        return Orientation::collinear;

    if (c > 0)
        return Orientation::positive;

    return Orientation::negative;
}

////////////////////////////////////////////////////////////
/// \brief geometry must has at least one vertex.
///
////////////////////////////////////////////////////////////
template <Geometry T>
IteratorOf<T> furthestVertexInDirection(const T& geometry, Vec2 direction)
{
    SIMU_ASSERT(any(direction != Vec2::filled(0.f)), "Any vertex will be returned");

    auto  furthest = geometry.begin();
    float maxDist  = dot(*furthest, direction);
    for (auto it = geometry.begin(); it != geometry.end(); ++it)
    {
        float dist = dot(*it, direction);
        if (dist > maxDist)
        {
            maxDist  = dist;
            furthest = it;
        }
    }

    return furthest;
}

/// \}

} // namespace simu
