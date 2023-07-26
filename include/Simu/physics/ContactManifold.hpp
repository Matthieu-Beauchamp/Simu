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

#include "Simu/math/Gjk.hpp"

#include "Simu/physics/Body.hpp"

namespace simu
{

class ContactManifold
{
public:

    ContactManifold(const Body* body0, const Body* body1)
        : bodies_{body0, body1}, 
          minPen_{CombinableProperty{body0->material().penetration, 
                                     body1->material().penetration}.value}
    {
        update();
    }

    void update()
    {
        Vec2 searchDir = nContacts_ == 0
                             ? (bodies_[1]->centroid()
                                - bodies_[0]->centroid())
                             : contactNormal();

        Gjk<Collider> gjk{
            bodies_[0]->collider(),
            bodies_[1]->collider(),
            searchDir};
        Vec2 mtv = gjk.penetration();

        nContacts_ = 0;

        if (normSquared(mtv) < minPen_ * minPen_)
            return;

        std::array<Edge, 2> contactEdges = computeContactEdges(mtv);

        Vec2 normal1 = contactEdges[0].normalizedNormal();
        Vec2 normal2 = contactEdges[1].normalizedNormal();

        float e1Coeff = dot(normal1, mtv);
        float e2Coeff = dot(normal2, -mtv);

        if (e1Coeff > e2Coeff)
        {
            referenceIndex_ = 0;
            contactNormal_  = normal1;
        }
        else
        {
            referenceIndex_ = 1;
            contactNormal_  = normal2;
        }

        computeContacts(contactEdges);

        contactNormal_ = Transform::linear(
            bodies_[referenceIndex()]->toLocalSpace(),
            contactNormal_
        );

        for (Uint32 b = 0; b < 2; ++b)
            for (Uint32 c = 0; c < nContacts(); ++c)
                contacts_[b][c] = bodies_[b]->toLocalSpace() * contacts_[b][c];
    }


    struct FrameManifold
    {
        Uint32                               nContacts;
        std::array<std::array<Vertex, 2>, 2> worldContacts;
        Vec2                                 normal;
        Vec2                                 tangent;
    };

    FrameManifold frameManifold() const
    {
        return FrameManifold{
            nContacts(),
            contacts(),
            contactNormal(),
            contactTangent()};
    }


    // if nContacts() is 0, then other getters will provide undefined results.
    Uint32 nContacts() const { return nContacts_; }

    /// contacts()[referenceIndex()][1] -> second contact point of the reference body
    std::array<std::array<Vertex, 2>, 2> contacts() const
    {
        std::array<std::array<Vertex, 2>, 2> worldContacts;

        for (Uint32 b = 0; b < 2; ++b)
            for (Uint32 c = 0; c < nContacts(); ++c)
                worldContacts[b][c]
                    = bodies_[b]->toWorldSpace() * contacts_[b][c];

        return worldContacts;
    }

    Vec2 contactNormal() const
    {
        return Transform::linear(
            bodies_[referenceIndex()]->toWorldSpace(),
            contactNormal_
        );
    }

    Vec2 contactTangent() const { return perp(contactNormal()); }

    Uint32 referenceIndex() const { return referenceIndex_; }
    Uint32 incidentIndex() const { return (referenceIndex() == 0) ? 1 : 0; }

    // minimum penetration norm for a contact to be considered.
    void  setMinimumPenetration(float minPen) { minPen_ = minPen; }
    float minimumPenetration() const { return minPen_; }

private:


    typedef typename Edges<Collider>::Edge Edge;

    std::array<Edge, 2> computeContactEdges(Vec2 mtv)
    {
        return {
            computeContactEdge(bodies_[0]->collider(), mtv),
            computeContactEdge(bodies_[1]->collider(), -mtv),
        };
    }

    static Edge computeContactEdge(const Collider& collider, Vec2 direction)
    {
        Vec2 v = furthestVertexInDirection(collider, direction);

        auto edges = edgesOf(collider);

        auto previous = std::find_if(edges.begin(), edges.end(), [=](Edge e) {
            return all(e.to() == v);
        });

        auto next = edges.next(previous);

        if (dot(next->normalizedNormal(), direction)
            > dot(previous->normalizedNormal(), direction))
            return *next;

        return *previous;
    }

    void computeContacts(std::array<Edge, 2> contactEdges)
    {
        const Uint32 ref = referenceIndex();
        const Uint32 inc = incidentIndex();

        auto refEdges = edgesOf(bodies_[ref]->collider());

        Edge previousOfRef{*refEdges.previous(contactEdges[ref])};
        Edge nextOfRef{*refEdges.next(contactEdges[ref])};

        Vec2 n = -contactNormal_;

        Vertex contact1 = contactEdges[inc].clipInside(previousOfRef);
        if (dot(contact1 - contactEdges[ref].from(), n) >= 0.f)
            contacts_[inc][nContacts_++] = contact1;

        Vertex contact2 = contactEdges[inc].clipInside(nextOfRef);
        if (dot(contact2 - contactEdges[ref].from(), n) >= 0.f)
        {
            contacts_[inc][nContacts_++] = contact2;
            if (nContacts_ == 2 && all(contacts_[inc][0] == contacts_[inc][1]))
                --nContacts_;
        }

        for (Uint32 i = 0; i < nContacts_; ++i)
        {
            contacts_[ref][i]
                = LineBarycentric{contactEdges[ref].from(), contactEdges[ref].to(), contacts_[inc][i]}
                      .closestPoint;
        }
    }

    std::array<std::array<Vertex, 2>, 2> contacts_{};
    Uint32                               nContacts_ = 0;

    Vec2 contactNormal_{}; // points outwards of the reference body

    // minimum penetration norm to compute a manifold when calling update()
    float minPen_;

    std::array<const Body*, 2> bodies_;

    Uint32 referenceIndex_ = 0;
};


} // namespace simu
