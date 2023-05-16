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

#include "Simu/math/GeometricProperties.hpp"
#include "Simu/math/Polygon.hpp"

namespace simu
{

GeometricProperties::GeometricProperties(const Polygon& polygon)
{
    Vertex previous = *std::prev(polygon.end());
    for (const Vertex& vertex : polygon)
    {
        float vertexCross = cross(previous, vertex);
        area += vertexCross;

        centroid += (previous + vertex) * vertexCross;

        momentOfArea += vertexCross
                        * (dot(previous, previous) + dot(previous, vertex)
                           + dot(vertex, vertex));

        previous = vertex;
    }

    if (area == 0.f)
    {
        isDegenerate = true;
    }
    else
    {
        area /= 2;
        centroid /= 6 * std::abs(area);
        momentOfArea /= 12;
        momentOfArea -= std::abs(area) * normSquared(centroid);
    }
}


} // namespace simu
