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
template <class T, Uint32 dim>
concept VertexIterator = requires(T it) {
    { *it } -> std::convertible_to<Vector<float, dim>>;
} && std::forward_iterator<T>;

////////////////////////////////////////////////////////////
/// \brief Any iterator that dereferences to a Vertex
/// 
////////////////////////////////////////////////////////////
template <class T>
concept VertexIterator2D = VertexIterator<T, 2>;

typedef Vec2                Vertex;
typedef std::vector<Vertex> Vertices;

////////////////////////////////////////////////////////////
/// \brief Any type that allows iterating over vertices
/// 
////////////////////////////////////////////////////////////
template<class T>
concept Geometry = requires(T geo){
    { geo.begin() } -> VertexIterator2D;
    { geo.end() }   -> VertexIterator2D;
};
// clang-format on

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
Orientation
orientation(Vertex v0, Vertex v1, Vertex v2, float epsilon = simu::EPSILON);

template <Geometry T>
Vertex furthestVertexInDirection(const T& geometry, Vec2 direction)
{
    Vec2  furthest = *geometry.begin();
    float maxDist  = dot(furthest, direction);
    for (const Vertex& v : geometry)
    {
        float dist = dot(v, direction);
        if (dist > maxDist)
        {
            maxDist  = dist;
            furthest = v;
        }
    }

    return furthest;
}


////////////////////////////////////////////////////////////
/// \brief Compute properties of some non self-intersecting Geometry.
///
/// Convex, concave and geometry containing holes are all valid input.
/// The hole(s) should be of opposite Orientation and attached to a vertex
///     of the outer (solid) perimeter.
///
/// if area == 0, then the geometry isDegenerate (collinear) and the
///     properties are undefined but no exception is raised
///
/// If vertices are negatively oriented, then the area is negative.
/// Centroid and momentOfArea are unaffected.
///
/// Assuming constant density p, then
///     mass    = p * |area|
///     inertia = p * momentOfArea
///
////////////////////////////////////////////////////////////
struct GeometricProperties
{
    GeometricProperties() = default;

    template <Geometry T>
    GeometricProperties(const T& geometry) noexcept;

    Vec2  centroid{};
    float area         = 0.f;
    float momentOfArea = 0.f;
    bool  isDegenerate = false;
};

/// \}

} // namespace simu

#include "Simu/math/Geometry.inl.hpp"
