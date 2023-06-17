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


Polytope::Polytope(const Simplex& simplex)
{
    Vertex first  = simplex.pointStack[0];
    Vertex second = simplex.pointStack[1];
    Vertex third  = simplex.pointStack[2];

    vertices.emplace_back(first);

    if (any(second != first))
    {
        vertices.emplace_back(second);
        if (orientation(first, second, third) != Orientation::collinear)
            vertices.emplace_back(third);
    }
    else if (any(third != first))
    {
        vertices.emplace_back(third);
    }
}

bool Polytope::addVertex(const Edge& where, Vertex v)
{
    bool areCollinear
        = orientation(*where.from, *where.to, v, 0.f) == Orientation::collinear;

    bool isFurther = std::abs(dot(where.normal, v))
                     > std::abs(dot(where.normal, *where.to));

    if (!areCollinear && isFurther)
    {
        vertices.insert(where.to, v);
        return true;
    }

    return false;
}

typename Polytope::Edge Polytope::getEdge(std::size_t index)
{
    Edge edge;
    edge.to = vertices.begin() + index;
    edge.from
        = std::prev(edge.to == vertices.begin() ? vertices.end() : edge.to);

    edge.normal = perp(*edge.to - *edge.from);
    return edge;
}

} // namespace priv


} // namespace simu
