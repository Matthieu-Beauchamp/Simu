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

namespace priv
{

Simplex::Simplex(Vec2 initialSearchDir) : initialSearchDir_{initialSearchDir}
{
    if (all(initialSearchDir_ == Vec2::filled(0.f)))
        initialSearchDir_ = Vec2::i();
}

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

    Vec2 ab = pointStack[1] - pointStack[0];
    Vec2 bc = pointStack[2] - pointStack[1];
    Vec2 ca = pointStack[0] - pointStack[2];

    bool positivelyOriented = cross(ab, bc) >= 0.f;

    normals[0] = perp(ab, positivelyOriented);
    normals[1] = perp(ca, positivelyOriented);
    normals[2] = perp(bc, positivelyOriented);

    ++nIterations;
}

Vec2 Simplex::nextDirection() const
{
    switch (nIterations)
    {
        case 0:
        {
            return initialSearchDir_;
        }
        case 1:
        {
            if (dot(initialSearchDir_, pointStack[0]) > 0)
                return -pointStack[0];

            return -initialSearchDir_;
        }
        case 2:
        {
            Vec2  edge = pointStack[0] - pointStack[1];
            float k    = cross(pointStack[1], pointStack[0]);
            if (k == 0.f)
                return perp(initialSearchDir_);

            return perp(edge, k < 0);
        }
        default:
        {
            bool norm1TowardsOrigin = dot(normals[0], pointStack[0]) < 0;
            // bool norm2TowardsOrigin = dot(normals[1], pointStack[0]) < 0;

            if (norm1TowardsOrigin)
                return normals[0];

            keepBottomPoint_ = true;
            return normals[1];
        }
    }
}

bool Simplex::hasPoint(Vertex v) const
{
    bool   hasP = false;
    Uint32 max = std::min(nIterations, static_cast<Uint32>(pointStack.size()));
    for (Uint32 i = 0; i < max; ++i)
        hasP = hasP || all(pointStack[i] == v);

    return hasP;
}


bool Simplex::containsOrigin() const
{
    return dot(pointStack[0], normals[0]) >= 0.f
           && dot(pointStack[0], normals[1]) >= 0.f
           && dot(pointStack[1], normals[2]) >= 0.f;
}


Polytope::Polytope(const Simplex& simplex)
{
    Vertex first  = simplex.pointStack[0];
    Vertex second = simplex.pointStack[1];
    Vertex third  = simplex.pointStack[2];

    vertices.emplace_back(first);
    vertices.emplace_back(second);
    vertices.emplace_back(third);

    if (orientation(first, second, third, 0.f) == Orientation::negative)
        std::reverse(vertices.begin(), vertices.end());
}

bool Polytope::addVertex(const Edge& where, Vertex v)
{
    if (where.isOutside(v))
    {
        // this is not ideal, but prevents almost collinear edges from
        //  putting us in an infinite loop:
        // Consider A-B-C three almost collinear vertices of the Polytope.
        // Sometimes when edge BC is the closest edge to the origin, the furthest
        //  vertex returned will be A, then creating a degenerate polytope
        //  with the sequence A-B-A-C and preventing the algorithm from terminating.
        // This could be probably be solved by using an epsilon value to prevent
        //  such collinear chains from being accepted in the Polytope.
        // Currently Gjk does not require any epsilon, and we prefer to keep it that way when possible.
        // If our Geometry had many more vertices, this loop becomes expensive and undesirable.
        for (const Vertex& vert : vertices)
            if (all(vert == v))
                return false;

        auto insertPos = where.toIt();
        vertices.insert(insertPos, v);

        return true;
    }

    return false;
}

} // namespace priv


template <Geometry T>
Gjk<T>::Gjk(const T& first, const T& second, Vec2 initialDir)
    : first_{first}, second_{second}, simplex_{initialDir}
{
    while (!done_)
    {
        Vec2 direction = simplex_.nextDirection();

        Vertex v = furthestVertexInDirection(direction);
        if (dot(v, direction) < 0.f)
        {
            done_         = true;
            areColliding_ = false;
        }

        bool isNewPoint = !simplex_.hasPoint(v);
        simplex_.pushPoint(v);

        if (simplex_.nIterations > 2)
        {
            if (simplex_.containsOrigin())
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
    }
}

template <Geometry T>
Vec2 Gjk<T>::separation()
{
    if (areColliding())
        return Vec2{};

    while (true)
    {
        Vec2   dir = simplex_.nextDirection();
        Vertex v   = furthestVertexInDirection(dir);

        if (simplex_.hasPoint(v))
            break;
        else
            simplex_.pushPoint(v);
    }

    switch (simplex_.nIterations)
    {
        case 1: return -simplex_.pointStack[0];
        case 2:
            return -LineBarycentric{
                simplex_.pointStack[0], simplex_.pointStack[1], Vec2{0, 0}
            }
                        .closestPoint;
        default:
            return -TriangleBarycentric{
                simplex_.pointStack[0],
                simplex_.pointStack[1],
                simplex_.pointStack[2],
                Vec2{0.f, 0.f}
            }
                        .closestPoint;
    }
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

        if (!polytope.addVertex(best, next))
            return LineBarycentric{
                best.from(), best.to(), Vec2{0, 0}
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
