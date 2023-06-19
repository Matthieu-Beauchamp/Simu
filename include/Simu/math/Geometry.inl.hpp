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

#include "Simu/math/Geometry.hpp"
#include "Simu/math/Edges.hpp"

namespace simu
{

template <Geometry T>
GeometricProperties::GeometricProperties(const T& geometry) noexcept
{
    // These formulas can be derived from the definition with a double integral
    //  and using Green's theorem reducing to an integral over the contour
    //  (edges) and evaluating to a sum.

    for (const auto& edge : edgesOf(geometry))
    {
        float vertexCross = cross(edge.from(), edge.to());
        area += vertexCross;

        centroid += (edge.from() + edge.to()) * vertexCross;

        momentOfArea += vertexCross
                        * (normSquared(edge.from()) + dot(edge.from(), edge.to())
                           + normSquared(edge.to()));
    }

    if (area == 0.f)
    {
        isDegenerate = true;
    }
    else
    {
        area /= 2;
        centroid /= 6 * area;
        momentOfArea /= 12;
        momentOfArea -= area * normSquared(centroid);
        momentOfArea = std::abs(momentOfArea);
    }
}


} // namespace simu
