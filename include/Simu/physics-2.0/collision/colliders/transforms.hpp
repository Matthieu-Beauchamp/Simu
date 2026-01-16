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
#include "BoundingBox.hpp"
#include "Capsule.hpp"
#include "Circle.hpp"
#include "Polygon.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/physics/Transform.hpp"

namespace simu
{

inline BoundingBox operator*(const Translation& translation, const BoundingBox& box) {
    return BoundingBox(translation * box.min(), translation * box.max());
}

inline Circle operator*(const Translation& translation, const Circle& circle) {
    return Circle(translation * circle.center(), circle.radius());
}

inline Circle operator*(const Transform& transform, const Circle& circle) {
    return transform.translation() * circle;
}

inline Capsule operator*(const Translation& translation, const Capsule& capsule) {
    return Capsule(
        translation * capsule.bottom_center(), translation * capsule.top_center(), capsule.radius()
    );
}

inline Capsule operator*(const Transform& transform, const Capsule& capsule) {
    return Capsule(
        transform * capsule.bottom_center(), transform * capsule.top_center(), capsule.radius()
    );
}

inline Polygon operator*(const Transform& transform, Polygon polygon) {
    return Polygon(polygon | std::views::transform([&](auto& vertex) {
                       return transform * vertex;
                   }));
}

} // namespace simu
