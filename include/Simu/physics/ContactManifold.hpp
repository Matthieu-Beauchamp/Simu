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

    ContactManifold(const Collider& first, const Collider& second)
        : colliders_{&first, &second},
          minPen_{CombinableProperty{first.material().penetration,
                                     second.material().penetration}.value}
    {
        update();
    }

    const std::array<const Collider*, 2>& colliders() const
    {
        return colliders_;
    }

    void update()
    {
        std::array<const Body*, 2> bodies{
            colliders_[0]->body(),
            colliders_[1]->body()};

        Vec2 searchDir = nContacts_ == 0
                             ? (bodies[1]->centroid() - bodies[0]->centroid())
                             : bodies[referenceIndex()]->toWorldSpace().rotation(
                               ) * contactNormal_;

        nContacts_ = 0;

        if (!colliders_[0]->boundingBox().overlaps(colliders_[1]->boundingBox()))
            return;

        Gjk<Collider> gjk{*colliders_[0], *colliders_[1], searchDir};
        Vec2          mtv = gjk.penetration();


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

        contactNormal_ = bodies[referenceIndex()]->toLocalSpace().rotation()
                         * contactNormal_;


        for (Uint32 i = 0; i < 2; ++i)
            for (Uint32 c = 0; c < nContacts(); ++c)
                contacts_[i][c] = bodies[i]->toLocalSpace() * contacts_[i][c];
    }


    struct FrameManifold
    {
        Uint32                               nContacts;
        std::array<std::array<Vertex, 2>, 2> worldContacts;
        Vec2                                 normal; // points out of bodies[1]
        Vec2                                 tangent;
    };

    typedef std::array<Transform, 2> Transforms;

    FrameManifold frameManifold(const Bodies& bodies) const
    {
        return frameManifold(
            Transforms{bodies[0]->toWorldSpace(), bodies[1]->toWorldSpace()}
        );
    }

    FrameManifold frameManifold(const Proxies& proxies) const
    {
        return frameManifold(
            Transforms{proxies[0].toWorldSpace(), proxies[1].toWorldSpace()}
        );
    }


    // if nContacts() is 0, then other getters will provide undefined results.
    Uint32 nContacts() const { return nContacts_; }


    Uint32 referenceIndex() const { return referenceIndex_; }
    Uint32 incidentIndex() const { return (referenceIndex() == 0) ? 1 : 0; }

    // minimum penetration norm for a contact to be considered.
    void  setMinimumPenetration(float minPen) { minPen_ = minPen; }
    float minimumPenetration() const { return minPen_; }

private:

    FrameManifold frameManifold(Transforms Ts) const
    {
        FrameManifold frame;
        frame.nContacts     = nContacts();
        frame.worldContacts = contacts(Ts);
        frame.normal        = contactNormal(Ts);
        frame.tangent       = contactTangent(Ts);

        if (referenceIndex() == 0)
        {
            frame.normal  = -frame.normal;
            frame.tangent = -frame.tangent;
        }

        return frame;
    }

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

    std::array<Edge, 2> computeContactEdges(Vec2 mtv)
    {
        return {
            computeContactEdge(*colliders_[0], mtv),
            computeContactEdge(*colliders_[1], -mtv),
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

        auto refEdges = edgesOf(*colliders_[ref]);

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

    std::array<const Collider*, 2> colliders_;

    std::array<std::array<Vertex, 2>, 2> contacts_{};
    Uint32                               nContacts_ = 0;

    Vec2 contactNormal_{}; // points outwards of the reference body

    // minimum penetration norm to compute a manifold when calling update()
    float minPen_;

    Uint32 referenceIndex_ = 0;
};


} // namespace simu
