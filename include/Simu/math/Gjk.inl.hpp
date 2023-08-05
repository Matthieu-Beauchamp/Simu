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

template <Geometry T>
Gjk<T>::Gjk(const T& first, const T& second, Vec2 initialDir)
    : first_{first}, second_{second}
{
    Vec2 direction = initialDir;
    if (all(direction == Vec2{}))
        direction = simplex_.nextDirection();

    while (!done_)
    {
        Vertex v = furthestVertexInDirection(direction);
        if (dot(v, direction) < 0)
        {
            done_         = true;
            areColliding_ = false;
        }

        bool isNewPoint = simplex_.pushPoint(v);

        if (simplex_.nIterations > 2)
        {
            if (all(simplex_.closestPoint(Vec2{0, 0}) == Vec2{0, 0}))
            {
                areColliding_ = true;
                done_         = true;
            }
            else if (!isNewPoint)
            {
                areColliding_ = false;
                done_         = true;
            }
        }

        direction = simplex_.nextDirection();
    }
}

template <Geometry T>
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

template <Geometry T>
Vec2 Gjk<T>::penetration() const
{
    if (!areColliding())
        return Vec2{};

    priv::Polytope polytope{simplex_};

    while (true)
    {
        auto  edges    = edgesOf(polytope.vertices);
        auto  best     = *edges.begin();
        float bestDist = best.distanceSquaredToOrigin();

        for (const auto& e : edges)
        {
            float dist = e.distanceSquaredToOrigin();
            if (dist < bestDist)
            {
                best     = e;
                bestDist = dist;
            }
        }

        Vertex next = furthestVertexInDirection(best.normal());

        if (!polytope.addVertex(edges, best, next))
            return LineBarycentric{
                best.from(),
                best.to(),
                Vec2{0, 0}
            }
                .closestPoint;
    }
}

template <Geometry T>
Vec2 Gjk<T>::furthestVertexInDirection(const Vec2& direction) const
{
    return simu::furthestVertexInDirection(first_, direction)
           - simu::furthestVertexInDirection(second_, -direction);
}

} // namespace simu
