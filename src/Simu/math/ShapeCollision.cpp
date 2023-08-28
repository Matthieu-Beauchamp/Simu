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

#include "Simu/math/ShapeCollision.hpp"

#include "Simu/math/Polygon.hpp"
#include "Simu/math/Circle.hpp"

#include "Simu/math/Geometry.hpp"
#include "Simu/math/Edges.hpp"
#include "Simu/math/BarycentricCoordinates.hpp"

#include "Simu/math/Gjk.hpp"
#include "Simu/math/Sat.hpp"

#include "Simu/utility/Algo.hpp"

namespace simu
{

/// \param mtv translation vector such that translating B by mtv makes the polygons only touch.
CollisionManifold polygonManifold(const Polygon& A, const Polygon& B, Vec2 mtv)
{
    auto findContactEdge = [](const Polygon& poly, Vec2 dir) {
        auto v = furthestVertexInDirection(poly, dir);

        auto edges = edgesOf(poly);

        auto previous = edges.previousEdgeOf(v);
        auto next     = edges.nextEdgeOf(v);

        if (dot(next->normalizedNormal(), dir)
            > dot(previous->normalizedNormal(), dir))
            return *next;

        return *previous;
    };

    auto edgeA = findContactEdge(A, mtv);
    auto edgeB = findContactEdge(B, -mtv);

    bool isReferenceA = dot(edgeA.normalizedNormal(), mtv)
                        > dot(edgeB.normalizedNormal(), -mtv);

    auto computeManifold = [](const Polygon& ref, auto incEdge, auto refEdge) {
        auto edges = edgesOf(ref);

        auto previous = *edges.previous(refEdge);
        auto next     = *edges.next(refEdge);

        CollisionManifold mani{};
        mani.normal  = refEdge.normalizedNormal();
        mani.tangent = perp(mani.normal);
        Vec2 n       = -mani.normal;

        Vec2 contact1 = incEdge.clipInside(previous);
        if (dot(contact1 - refEdge.from(), n) >= 0.f)
            mani.contactsA[mani.nContacts++] = contact1;

        Vec2 contact2 = incEdge.clipInside(next);
        if (dot(contact2 - refEdge.from(), n) >= 0.f)
        {
            mani.contactsA[mani.nContacts++] = contact2;
            if (mani.nContacts == 2 && all(mani.contactsA[0] == mani.contactsA[1]))
                --mani.nContacts;
        }

        for (Uint32 i = 0; i < mani.nContacts; ++i)
        {
            mani.contactsB[i]
                = LineBarycentric{refEdge.from(), refEdge.to(), mani.contactsA[i]}
                      .closestPoint;
        }

        return mani;
    };

    const Polygon& ref     = isReferenceA ? A : B;
    auto           incEdge = isReferenceA ? edgeB : edgeA;
    auto           refEdge = isReferenceA ? edgeA : edgeB;

    CollisionManifold mani = computeManifold(ref, incEdge, refEdge);
    if (isReferenceA)
        mani.invert();

    return mani;
}

CollisionManifold collidePolygonsSat(const Shape& A, const Shape& B)
{
    const Polygon& pA = static_cast<const Polygon&>(A);
    const Polygon& pB = static_cast<const Polygon&>(B);

    CollisionManifold mani{};

    Sat<Polygon> sat{pA, pB};
    if (!sat.areColliding())
        return mani;

    Vec2 mtv = sat.penetration();

    // Minimum mtv is not required here since the mtv is always
    //  parallel to the contact edge normal.
    // Unlike with Gjk where we might have numerical error due to the
    //  projection.

    return polygonManifold(pA, pB, mtv);
}

CollisionManifold collidePolygonsGjkEpa(const Shape& A, const Shape& B)
{
    const Polygon& pA = static_cast<const Polygon&>(A);
    const Polygon& pB = static_cast<const Polygon&>(B);

    CollisionManifold mani{};

    Gjk<Polygon> gjk{pA, pB, pA.properties().centroid - pB.properties().centroid};
    if (!gjk.areColliding())
        return mani;

    Vec2 mtv = gjk.penetration();

    // TODO: minimum penetration length to avoid incorrect manifolds.
    //  (this should use Material::penetration in physics.)
    if (normSquared(mtv) < squared(0.005))
        return mani;

    return polygonManifold(pA, pB, mtv);
}

CollisionManifold collidePolygonWithCircle(const Shape& A, const Shape& B)
{
    const Polygon& pA = static_cast<const Polygon&>(A);
    const Circle&  cB = static_cast<const Circle&>(B);

    CollisionManifold mani{};

    Vec2 dir = pA.properties().centroid - cB.center();
    if (normSquared(dir) == 0.f)
        return mani;

    auto v        = furthestVertexInDirection(pA, -dir);
    auto edges    = edgesOf(pA);
    auto next     = edges.nextEdgeOf(v);
    auto previous = edges.previousEdgeOf(v);

    LineBarycentric nextBary{next->from(), next->to(), cB.center()};
    LineBarycentric prevBary{previous->from(), previous->to(), cB.center()};


    // in case cB.center() == closest,
    // We could also return no contact and wait for next frame...
    Vec2 backupNormalDir;
    Vec2 closest;
    if (nextBary.u == 1.f && prevBary.v == 1.f)
    {
        closest         = *v;
        backupNormalDir = pA.properties().centroid - closest;
    }
    else
    {
        Vec2 nextToCircle = cB.center() - nextBary.closestPoint;
        Vec2 prevToCircle = cB.center() - prevBary.closestPoint;
        if (normSquared(nextToCircle) < normSquared(prevToCircle))
        {
            closest         = nextBary.closestPoint;
            backupNormalDir = -next->normal();
        }
        else
        {
            closest         = prevBary.closestPoint;
            backupNormalDir = -previous->normal();
        }
    }

    // TODO: Can this early exit check be done earlier?
    dir = closest - cB.center();
    Vec2 outwards = closest - pA.properties().centroid;

    bool circleIsInsidePoly = dot(dir, outwards) > 0.f;
    if (circleIsInsidePoly)
        dir = -dir;
    else if (normSquared(dir) > squared(cB.radius()))
        return mani;


    if (normSquared(dir) == 0.f)
        mani.normal = normalized(backupNormalDir);
    else
        mani.normal = normalized(dir);

    mani.tangent      = perp(mani.normal);
    mani.nContacts    = 1;
    mani.contactsA[0] = closest;
    mani.contactsB[0] = cB.center() + mani.normal * cB.radius();

    return mani;
}

CollisionManifold collideCircles(const Shape& A, const Shape& B)
{
    const Circle& cA = static_cast<const Circle&>(A);
    const Circle& cB = static_cast<const Circle&>(B);

    CollisionManifold mani{};

    Vec2 n = cA.center() - cB.center();

    float d = normSquared(n);
    if (d == 0.f)
        return mani;

    mani.normal  = normalized(n);
    mani.tangent = perp(mani.normal);
    if (d < squared(cA.radius() + cB.radius()))
    {
        mani.nContacts    = 1;
        mani.contactsA[0] = cA.center() - mani.normal * cA.radius();
        mani.contactsB[0] = cB.center() + mani.normal * cB.radius();
    }

    return mani;
}


} // namespace simu
