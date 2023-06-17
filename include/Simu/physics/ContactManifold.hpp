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
#include "Simu/math/Geometry.hpp"
#include "Simu/physics/PointerArray.hpp"

namespace simu
{

template <Geometry T>
IteratorOf<T> previousVertex(T& geo, IteratorOf<T> vertex)
{
    return (vertex == geo.begin()) ? std::prev(geo.end()) : std::prev(vertex);
}

template <Geometry T>
IteratorOf<T> nextVertex(T& geo, IteratorOf<T> vertex)
{
    return (vertex == std::prev(geo.end())) ? geo.begin() : std::next(vertex);
}


template <VertexIterator2D It>
struct Edge
{
    It from;
    It to;

    Vec2 direction() const { return *to - *from; }

    // assuming positive orientation, the normal points outwards
    Vec2 normal() const { return perp(direction(), true); }
    Vec2 normalizedNormal() const { return normalized(normal()); }

    // if this is parallel to other, assumes they are overlapping.
    // returns the vertex on this closest to other which is towards the inside of other.
    Vertex clipInside(const Edge& other) const
    {
        Vec2 n = -other.normal(); // inside facing normal

        if (dot(direction(), n) == 0.f)
            return *from;

        Edge directionTowardsInterior
            = dot(direction(), n) > 0 ? *this : Edge{to, from};

        if (dot(*directionTowardsInterior.from - *other.from, n) >= 0.f)
            return *directionTowardsInterior.from;

        Vec2 parametricCoefficients = solve(
            Mat2::fromCols(
                {directionTowardsInterior.direction(), other.direction()}
            ),
            *other.from - *directionTowardsInterior.from
        );

        SIMU_ASSERT(
            Interval<float>(0.f, 1.f).contains(parametricCoefficients[0]),
            "Intersection is not within this edge, edges were assumed to be "
            "intersecting"
        );

        return *directionTowardsInterior.from
               + parametricCoefficients[0] * directionTowardsInterior.direction();
    }
};


template <Geometry T>
class ContactManifold
{
public:

    typedef decltype(std::declval<const T>().begin()) It;

    std::array<Vertex, 2> contacts; // are on the incident face
    Uint32                nContacts = 0;

    Vec2 contactNormal; // points outwards of the reference body

    PointerArray<T, 2, true> bodies;
    std::array<Edge<It>, 2>  contactEdges;

    Uint32 referenceIndex() const { return referenceIndex_; }
    Uint32 incidentIndex() const { return (referenceIndex() == 0) ? 1 : 0; }

private:

    Uint32 referenceIndex_ = 0;


public:

    // mtv is such that translating bodies[1] by mtv makes the bodies only touch,
    //  the bodies must be colliding.
    // (mtv points outwards of bodies[0])
    ContactManifold(const PointerArray<T, 2, true>& bodies, Vec2 mtv) : bodies{bodies}
    {
        computeContactEdges(mtv);

        float e1Coeff = dot(contactEdges[0].normalizedNormal(), mtv);
        float e2Coeff = dot(contactEdges[1].normalizedNormal(), -mtv);

        if (e1Coeff > e2Coeff)
        {
            referenceIndex_ = 0;
            contactNormal   = mtv;
        }
        else
        {
            referenceIndex_ = 1;
            contactNormal   = -mtv;
        }

        computeContacts();
    }

private:

    void computeContacts()
    {
        const Uint32 ref = referenceIndex();
        const Uint32 inc = incidentIndex();

        Edge<It> previousOfRef{
            previousVertex(*bodies[ref], contactEdges[ref].from),
            contactEdges[ref].from};
        Edge<It> nextOfRef{
            contactEdges[ref].to,
            nextVertex(*bodies[ref], contactEdges[ref].to)};

        Vec2 n = -contactNormal;

        Vertex contact1 = contactEdges[inc].clipInside(previousOfRef);
        if (dot(contact1 - *contactEdges[ref].from, n) > 0.f)
            contacts[nContacts++] = contact1;

        Vertex contact2 = contactEdges[inc].clipInside(nextOfRef);
        if (dot(contact2 - *contactEdges[ref].from, n) > 0.f)
            contacts[nContacts++] = contact2;
    }

    void computeContactEdges(Vec2 mtv)
    {
        contactEdges = {
            computeContactEdge(*bodies[0], mtv),
            computeContactEdge(*bodies[1], -mtv),
        };
    }

    static Edge<It> computeContactEdge(const T& body, Vec2 direction)
    {
        // TODO: Check for furthest vertex first, then check neighbor edges

        auto from = std::prev(body.end());
        auto to   = body.begin();

        Edge  edge{from, to};
        float parallelCoefficient = dot(edge.normalizedNormal(), direction);

        from = to++;
        while (to != body.end())
        {
            Edge  tmp = {from, to};
            float c   = dot(tmp.normalizedNormal(), direction);
            if (c > parallelCoefficient)
            {
                parallelCoefficient = c;
                edge                = tmp;
            }

            from = to++;
        }

        return edge;
    }
};


} // namespace simu
