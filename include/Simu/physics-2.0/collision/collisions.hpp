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
#include "colliders/BoundingBox.hpp"
#include "colliders/Polygon.hpp"
#include "colliders/Circle.hpp"
#include "colliders/Capsule.hpp"

namespace simu
{

inline bool collides(const BoundingBox& a, const BoundingBox& b) {
    if (!a.isValid() || !b.isValid())
        return false;

    return (
        a.max()[0] >= b.min()[0] && b.max()[0] >= a.min()[0]
        && a.max()[1] >= b.min()[1] && b.max()[1] >= a.min()[1]
    );
}

inline bool collides(const Circle& a, const Circle& b) {
    float min_dist = a.radius() + b.radius();
    return normSquared(a.center() - b.center()) <= min_dist * min_dist;
}

inline bool collides(const Circle& a, const Capsule& b) {}

inline bool collides(const Circle& a, const Polygon& b) {}

inline bool collides(const Capsule& a, const Capsule& b) {}

inline bool collides(const Capsule& a, const Polygon& b) {}

inline bool collides(const Polygon& a, const Polygon& b) {}

} // namespace simu
