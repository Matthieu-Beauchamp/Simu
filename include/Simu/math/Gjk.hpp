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

namespace simu
{

template <class T>
concept Collidable = requires(T collidable) {
    {
        collidable.furthestVertexInDirection(Vec2{})
    } -> std::convertible_to<Vec2>;
};

template<Collidable T = simu::ConvexPolygon>
class Gjk
{
public:

    Gjk(const T& first, const T& second);

private:

    Vec2 furthestVertexInDirection(const Vec2& direction) const;

    Vec2 updateSimplex(const Vertex& v);

    const T& first_;
    const T& second_;
    float                 distance_;

    std::array<Vertex, 3> simplex_{};
    std::size_t           simplexIndex_ = 0;
    bool                  done_         = false;
};


} // namespace simu
