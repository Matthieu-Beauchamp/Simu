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

#include "Simu/math/Polygon.hpp"

namespace simu
{

template <VertexIterator2D It>
Polygon::Polygon(It begin, It end)
{
    SIMU_ASSERT(
        std::distance(begin, end) >= 3,
        "Convex Geometry must have at least 3 vertices"
    );

    while (begin != end)
        vertices_.emplace_back(*begin++);

    properties_ = GeometricProperties{*this};
    if (properties().area < 0)
    {
        std::reverse(vertices_.begin(), vertices_.end());
        properties_.area *= -1;
    }
}

} // namespace simu
