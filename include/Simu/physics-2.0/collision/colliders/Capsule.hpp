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

    /// The top and bottom points are not inside the capsule but are the very edges of it
    /// @param bottom The bottom point of the capsule (ground)
    /// @param top The top point of the capsule
    /// @param radius The radius of the capsule
    Capsule(Vec2 bottom, Vec2 top, float radius)
        : _bottom{bottom}, _top{top}, _radius{radius} {}

    Vec2  bottom() const { return _bottom; }
    Vec2  top() const { return _top; }
    float radius() const { return _radius; }

private:

    Vec2  _bottom;
    Vec2  _top;
    float _radius;
};

} // namespace simu
