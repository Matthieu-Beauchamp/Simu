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

#include <algorithm>
#include <array>

#include "Simu/config.hpp"
#include "Simu/math/Geometry.hpp"
#include "Simu/math/BarycentricCoordinates.hpp"

namespace simu
{

// requires that the geometry be positively oriented.
template <Geometry T>
class ContactManifold
{
public:

    typedef typename Edges<T>::Edge Edge;

    std::array<std::array<Vertex, 2>, 2> contacts;
    Uint32                               nContacts = 0;

    Vec2 contactNormal; // points outwards of the reference body

    std::array<const T*, 2> bodies;
    std::array<Edge, 2>     contactEdges;

    Uint32 referenceIndex() const { return referenceIndex_; }
    Uint32 incidentIndex() const { return (referenceIndex() == 0) ? 1 : 0; }

private:

    Uint32 referenceIndex_ = 0;


public:

    // mtv is such that translating bodies[1] by mtv makes the bodies only touch,
    //  the bodies must be colliding.
    // (mtv points outwards of bodies[0])
    ContactManifold(const T* body0, const T* body1, Vec2 mtv)
        : bodies{body0, body1}, contactEdges{computeContactEdges(bodies, mtv)}
    {
        Vec2 normal1 = contactEdges[0].normalizedNormal();
        Vec2 normal2 = contactEdges[1].normalizedNormal();

        float e1Coeff = dot(normal1, mtv);
        float e2Coeff = dot(normal2, -mtv);

        if (e1Coeff > e2Coeff)
        {
            referenceIndex_ = 0;
            contactNormal   = normal1;
        }
        else
        {
            referenceIndex_ = 1;
            contactNormal   = normal2;
        }

        computeContacts();
    }

private:

    void computeContacts()
    {
        const Uint32 ref = referenceIndex();
        const Uint32 inc = incidentIndex();

        Edges<T> refEdges = edgesOf(*bodies[ref]);

        Edge previousOfRef{*refEdges.previous(contactEdges[ref])};
        Edge nextOfRef{*refEdges.next(contactEdges[ref])};

        Vec2 n = -contactNormal;

        Vertex contact1 = contactEdges[inc].clipInside(previousOfRef);
        if (dot(contact1 - contactEdges[ref].from(), n) >= 0.f)
            contacts[inc][nContacts++] = contact1;

        Vertex contact2 = contactEdges[inc].clipInside(nextOfRef);
        if (dot(contact2 - contactEdges[ref].from(), n) >= 0.f)
        {
            contacts[inc][nContacts++] = contact2;
            if (nContacts == 2 && all(contacts[inc][0] == contacts[inc][1]))
                --nContacts;
        }

        for (Uint32 i = 0; i < nContacts; ++i)
        {
            contacts[ref][i]
                = LineBarycentric{contactEdges[ref].from(), contactEdges[ref].to(), contacts[inc][i]}
                      .closestPoint;
        }
    }

    static std::array<Edge, 2>
    computeContactEdges(std::array<const T*, 2> bodies, Vec2 mtv)
    {
        return {
            computeContactEdge(*bodies[0], mtv),
            computeContactEdge(*bodies[1], -mtv),
        };
    }

    static Edge computeContactEdge(const T& body, Vec2 direction)
    {
        // TODO: Check for furthest vertex first, then check neighbor edges

        Vec2 v = furthestVertexInDirection(body, direction);

        auto edges = edgesOf(body);

        auto previous = std::find_if(
            edges.begin(),
            edges.end(),
            [=](typename Edges<T>::Edge e) { return all(e.to() == v); }
        );

        auto next = edges.next(previous);

        if (dot(next->normalizedNormal(), direction)
            > dot(previous->normalizedNormal(), direction))
            return *next;

        return *previous;
    }
};


} // namespace simu
