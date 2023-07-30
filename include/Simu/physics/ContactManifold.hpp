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

#include "Simu/physics/Bodies.hpp"

namespace simu
{

class ContactManifold
{
public:

    ContactManifold(const Bodies& bodies)
        : minPen_{
            CombinableProperty{
                               bodies.bodies()[0]->material().penetration,
                               bodies.bodies()[1]->material().penetration}
                .value
    }
    {
        update(bodies);
    }

    void update(const Bodies& bodies)
    {
        auto b = bodies.bodies();

        Vec2 searchDir = nContacts_ == 0
                             ? (b[1]->centroid() - b[0]->centroid())
                             : b[referenceIndex()]->toWorldSpace().rotation()
                                   * contactNormal_;

        nContacts_ = 0;

        if (!b[0]->collider().boundingBox().overlaps(
                b[1]->collider().boundingBox()
            ))
            return;

        Gjk<Collider> gjk{b[0]->collider(), b[1]->collider(), searchDir};
        Vec2          mtv = gjk.penetration();


        if (normSquared(mtv) < minPen_ * minPen_)
            return;

        std::array<Edge, 2> contactEdges = computeContactEdges(bodies, mtv);

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

        computeContacts(bodies, contactEdges);

        contactNormal_
            = b[referenceIndex()]->toLocalSpace().rotation() * contactNormal_;


        for (Uint32 i = 0; i < 2; ++i)
            for (Uint32 c = 0; c < nContacts(); ++c)
                contacts_[i][c] = b[i]->toLocalSpace() * contacts_[i][c];
    }


    struct FrameManifold
    {
        Uint32                               nContacts;
        std::array<std::array<Vertex, 2>, 2> worldContacts;
        Vec2                                 normal;
        Vec2                                 tangent;
    };

    FrameManifold frameManifold(const Bodies& bodies) const
    {
        Transforms Ts;

        if (bodies.isSolving())
        {
            auto p = bodies.proxies();
            Ts     = {p[0]->toWorldSpace(), p[1]->toWorldSpace()};
        }
        else
        {
            auto b = bodies.bodies();
            Ts     = {b[0]->toWorldSpace(), b[1]->toWorldSpace()};
        }

        return FrameManifold{
            nContacts(),
            contacts(Ts),
            contactNormal(Ts),
            contactTangent(Ts)};
    }


    // if nContacts() is 0, then other getters will provide undefined results.
    Uint32 nContacts() const { return nContacts_; }


    Uint32 referenceIndex() const { return referenceIndex_; }
    Uint32 incidentIndex() const { return (referenceIndex() == 0) ? 1 : 0; }

    // minimum penetration norm for a contact to be considered.
    void  setMinimumPenetration(float minPen) { minPen_ = minPen; }
    float minimumPenetration() const { return minPen_; }

private:

    typedef std::array<Transform, 2> Transforms;

    /// contacts()[referenceIndex()][1] -> second contact point of the reference body
    std::array<std::array<Vertex, 2>, 2> contacts(const Transforms& Ts) const
    {
        std::array<std::array<Vertex, 2>, 2> worldContacts;

        for (Uint32 b = 0; b < 2; ++b)
            for (Uint32 c = 0; c < nContacts(); ++c)
                worldContacts[b][c] = Ts[b] * contacts_[b][c];

        return worldContacts;
    }

    Vec2 contactNormal(const Transforms& Ts) const
    {
        return Ts[referenceIndex()].rotation() * contactNormal_;
    }

    Vec2 contactTangent(const Transforms& Ts) const
    {
        return perp(contactNormal(Ts));
    }


    typedef typename Edges<Collider>::Edge Edge;

    std::array<Edge, 2> computeContactEdges(const Bodies& bodies, Vec2 mtv)
    {
        auto b = bodies.bodies();

        return {
            computeContactEdge(b[0]->collider(), mtv),
            computeContactEdge(b[1]->collider(), -mtv),
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

    void computeContacts(const Bodies& bodies, std::array<Edge, 2> contactEdges)
    {
        auto b = bodies.bodies();

        const Uint32 ref = referenceIndex();
        const Uint32 inc = incidentIndex();

        auto refEdges = edgesOf(b[ref]->collider());

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

    Uint32 referenceIndex_ = 0;
};


} // namespace simu
