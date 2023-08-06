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

#include "Simu/math/Gjk.hpp"

namespace simu
{

namespace priv
{

bool Simplex::pushPoint(Vertex v)
{
    if (keepBottomPoint_)
    {
        pointStack[1]    = pointStack[2];
        keepBottomPoint_ = false;
    }

    bool isNewPoint = true;
    for (Vec2 p : pointStack)
        isNewPoint = isNewPoint && !all(p == v);

    pointStack[2] = pointStack[1];
    pointStack[1] = pointStack[0];
    pointStack[0] = v;

    Vec2  ab   = pointStack[1] - pointStack[0];
    Vec2  ac   = pointStack[2] - pointStack[0];
    float k    = cross(ab, ac);
    normals[0] = perp(ab, k > 0);
    normals[1] = perp(ac, k < 0);

    ++nIterations;

    return isNewPoint;
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


Polytope::Polytope(const Simplex& simplex)
{
    Vertex first  = simplex.pointStack[0];
    Vertex second = simplex.pointStack[1];
    Vertex third  = simplex.pointStack[2];

    vertices.emplace_back(first);

    // TODO: If we cannot add the three vertices, then it is an error since we cannot garantee the ordering of vertices...
    if (any(second != first))
    {
        vertices.emplace_back(second);
        if (orientation(first, second, third, 0.f) != Orientation::collinear)
        {
            vertices.emplace_back(third);
            if (orientation(first, second, third, 0.f) == Orientation::negative)
                std::reverse(vertices.begin(), vertices.end());
        }
    }
    else if (any(third != first))
    {
        vertices.emplace_back(third);
    }
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


} // namespace simu
