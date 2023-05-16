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

#include "Simu/math/Gjk.hpp"
#include "Simu/math/BarycentricCoordinates.hpp"

namespace simu
{

void Simplex::pushPoint(Vertex v)
{
    if (keepBottomPoint_)
    {
        pointStack[1]    = pointStack[2];
        keepBottomPoint_ = false;
    }

    pointStack[2] = pointStack[1];
    pointStack[1] = pointStack[0];
    pointStack[0] = v;

    Vec2  ab   = pointStack[1] - pointStack[0];
    Vec2  ac   = pointStack[2] - pointStack[0];
    float k    = cross(ab, ac);
    normals[0] = perp(ab, k > 0);
    normals[1] = perp(ac, k < 0);

    ++nIterations;
}

Vec2 Simplex::nextDirection() const
{
    switch (nIterations)
    {
        case 0:
        {
            return Vec2::i(); // TODO: use difference between centroids
        }
        case 1:
        {
            if (dot(Vec2::i(), pointStack[0]) < 0)
                return -Vec2::i();

            return -pointStack[0];
        }
        case 2:
        {
            Vec2 edge = pointStack[0] - pointStack[1];
            if (all(edge == Vec2{0, 0}))
                return Vec2::j();
            else
                return perp(edge, cross(pointStack[1], pointStack[0]) < 0);
        }
        default:
        {
            bool norm1TowardOrigin = dot(normals[0], pointStack[0]) < 0;
            bool norm2TowardOrigin = dot(normals[1], pointStack[0]) < 0;

            // TODO: point did not pass origin, stop.
            if (norm1TowardOrigin && norm2TowardOrigin)
                return normalized(normals[0]) + normalized(normals[1]);

            if (norm1TowardOrigin)
                return normals[0];

            keepBottomPoint_ = true;
            return normals[1];
        }
    }
}

Vec2 Simplex::closestPoint(Vec2 Q) const
{
    switch (nIterations)
    {
        case 0: throw simu::Exception{"No simplex yet!"};
        case 1: return pointStack[0];
        case 2:
            return LineBarycentric{pointStack[0], pointStack[1], Q}.closestPoint;
        default:
            return TriangleBarycentric{
                pointStack[0],
                pointStack[1],
                pointStack[2],
                Q}
                .closestPoint;
    }
}


template <Collidable T>
Gjk<T>::Gjk(const T& first, const T& second) : first_{first}, second_{second}
{
    while (!done_)
    {
        Vec2   direction = simplex_.nextDirection();
        Vertex v         = furthestVertexInDirection(direction);
        if (dot(v, direction) < 0)
        {
            done_         = true;
            areColliding_ = false;
        }

        simplex_.pushPoint(v);

        if (simplex_.nIterations > 2)
        {
            if (all(simplex_.closestPoint(Vec2{0, 0}) == Vec2{0, 0}))
            {
                areColliding_ = true;
                done_         = true;
            }
        }
    }
}

template <Collidable T>
Vec2 Gjk<T>::separation()
{
    if (areColliding())
        return Vec2{};

    while (true)
    {
        Vertex previous = simplex_.pointStack[0];
        Vertex current  = furthestVertexInDirection(simplex_.nextDirection());

        if (simplex_.nIterations < 3
            || normSquared(previous) > normSquared(current))
            simplex_.pushPoint(current);
        else
            break;
    }

    return -simplex_.closestPoint(Vec2{0, 0});
}

template <Collidable T>
Vec2 Gjk<T>::penetration()
{
    if (!areColliding())
        return Vec2{};

    std::vector<Vertex> polytope{
        simplex_.pointStack.begin(),
        simplex_.pointStack.end()};

    while (true)
    {
        // TODO: Degenerate edges (duplicate points) will divide by zero when trying to normalize

        auto previous = std::prev(polytope.end());
        auto current  = polytope.begin();

        Vec2  edgeNormal       = normalized(perp(*current - *previous));
        float shortestDistance = std::abs(dot(edgeNormal, *current));
        auto  insertionPos     = current;

        previous = current++;
        for (; current != polytope.end(); ++current)
        {
            Vec2  normal   = normalized(perp(*current - *previous));
            float distance = std::abs(dot(normal, *current));

            if (distance < shortestDistance)
            {
                edgeNormal       = normal;
                shortestDistance = distance;
                insertionPos     = current;
            }

            previous = current;
        }

        Vertex next = furthestVertexInDirection(edgeNormal);
        if (std::abs(dot(edgeNormal, next)) > shortestDistance)
            polytope.insert(insertionPos, next);
        else
            return LineBarycentric{
                *std::prev(
                    insertionPos == polytope.begin() ? polytope.end() : insertionPos
                ),
                *insertionPos,
                Vec2{0, 0}
            }.closestPoint;
    }
}

template <Collidable T>
Vec2 Gjk<T>::furthestVertexInDirection(const Vec2& direction) const
{
    return first_.furthestVertexInDirection(direction)
           - second_.furthestVertexInDirection(-direction);
}

} // namespace simu
