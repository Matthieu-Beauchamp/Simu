////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2025 Matthieu Beauchamp-Boulay
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
#include "Simu/math/Matrix.hpp"

namespace simu
{

class Capsule
{
public:

    /// @param bottom_center The bottom center of the capsule (ground)
    /// @param top_center The top center of the capsule
    /// @param radius The radius of the capsule
    Capsule(Vec2 bottom_center, Vec2 top_center, float radius)
        : _bottom_center{bottom_center}, _top_center{top_center}, _radius{radius} {}

    [[nodiscard]] Vec2 up_axis() const { return _top_center - _bottom_center; }

    [[nodiscard]] Vec2 top_center() const { return _top_center; }
    [[nodiscard]] Vec2 bottom_center() const { return _bottom_center; }

    [[nodiscard]] Vec2 bottom() const {
        return _bottom_center + radius() * normalized(_bottom_center - _top_center);
    }
    [[nodiscard]] Vec2 top() const {
        return _top_center + radius() * normalized(_top_center - _bottom_center);
    }

    [[nodiscard]] float radius() const { return _radius; }

private:

    Vec2  _bottom_center;
    Vec2  _top_center;
    float _radius;
};

} // namespace simu
