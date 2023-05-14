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

#include "Simu/math/Gjk.hpp"

namespace simu
{

template <Collidable T>
Gjk<T>::Gjk(const T& first, const T& second) : first_{first}, second_{second}
{
    Vec2   direction = Vec2::i(); // TODO: use difference between centroids
    Vertex v         = furthestVertexInDirection(direction);
    updateSimplex(v);
    direction = -v;

    while (!done_)
    {
        v = furthestVertexInDirection(direction);
        if (dot(v, direction) < 0)
        {
            done_ = true;
            // TODO: Distance
        }

        direction = updateSimplex(v);
    }
}

template <Collidable T>
Vec2 Gjk<T>::furthestVertexInDirection(const Vec2& direction) const
{
    return first_.furthestVertexInDirection(direction)
           - second_.furthestVertexInDirection(direction);
}

template <Collidable T>
Vec2 Gjk<T>::updateSimplex(const Vertex& v)
{
    switch (simplexIndex_)
    {
        case 0:
        {
            simplex_[simplexIndex_++] = v;
            return -v;
        }
        case 1:
        {
            simplex_[simplexIndex_++] = v;
            Vec2 edge = simplex_[1] - simplex_[0];
            return perp(edge, cross(simplex_[0], simplex_[1]) < 0);
        }
        default:
        {
            simplex_[simplexIndex_] = v;
            

        }
    }
}


} // namespace simu
