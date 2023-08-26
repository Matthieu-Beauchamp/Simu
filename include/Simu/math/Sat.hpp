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

#include <array>

#include "Simu/math/Polygon.hpp"
#include "Simu/math/Edges.hpp"

namespace simu
{

// currently made to fit the Gjk interface,
// Could add getters for reference edge to remove some work from the CollisionManifold
//  computation
template <Geometry T = simu::Polygon>
class Sat
{
public:

    Sat(const T& first, const T& second)
    {
        SeparationEdge fromFirst  = maxSeparation(first, second);
        SeparationEdge fromSecond = maxSeparation(second, first);

        if (fromFirst.separation > fromSecond.separation)
        {
            maxSep_       = fromFirst;
            ownerIsFirst_ = true;
        }
        else
        {
            maxSep_       = fromSecond;
            ownerIsFirst_ = false;
        }
    }

    bool areColliding() const { return maxSep_.separation < 0.f; }

    ////////////////////////////////////////////////////////////
    /// \brief Minimum translation vector such that areColliding() would be true
    ///
    /// It is the vector of the closest feature of first to the closest feature of second
    ///
    /// If areColliding(), returns a null vector.
    ///
    /// In order to make first and second only touch, both are equivalent:
    ///     - translate first by separation()
    ///     - translate seccond by -separation()
    ///
    ////////////////////////////////////////////////////////////
    Vec2 separation()
    {
        float sep  = std::max(maxSep_.separation, 0.f);
        bool  flip = !ownerIsFirst_;
        return maxSep_.edge.normalizedNormal() * sep * (flip ? -1.f : 1.f);
    }

    ////////////////////////////////////////////////////////////
    /// \brief Minimum translation vector such that first and second are only touching
    ///
    /// It is the vector of the penetration of second into first
    ///
    /// If not areColliding(), returns a null vector.
    ///
    /// In order to make first and second only touch, both are equivalent:
    ///     - translate first by -penetration()
    ///     - translate second by penetration()
    ///
    ////////////////////////////////////////////////////////////
    Vec2 penetration() const
    {
        float sep  = std::min(maxSep_.separation, 0.f);
        bool  flip = ownerIsFirst_;
        return maxSep_.edge.normalizedNormal() * sep * (flip ? -1.f : 1.f);
    }

    typedef typename Edges<T>::Edge Edge;

    struct SeparationEdge
    {
        float separation;
        Edge  edge;
    };

    static SeparationEdge maxSeparation(const T& theseEdges, const T& theseVertices)
    {
        SeparationEdge maxSep;
        maxSep.separation = -std::numeric_limits<float>::max();
        for (const Edge& e : edgesOf(theseEdges))
        {
            float sep = separation(e, theseVertices);
            if (sep > maxSep.separation)
            {
                maxSep.separation = sep;
                maxSep.edge       = e;
            }
        }

        return maxSep;
    }

    static float separation(Edge edge, const T& theseVertices)
    {
        Vec2 n = edge.normalizedNormal();

        // TODO: Cache this vertex iterator for polygonManifold.
        auto nearest = furthestVertexInDirection(theseVertices, -n);

        return dot(*nearest - edge.from(), n);
    }

private:

    SeparationEdge maxSep_;
    bool           ownerIsFirst_;
};


} // namespace simu
