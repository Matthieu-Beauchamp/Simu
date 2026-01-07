////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
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
#include "ColliderPool.hpp"
#include "ObjectId.hpp"
#include "../collision/colliders/BoundingBox.hpp"

namespace simu
{

inline BoundingBox bounding_box(const Circle& circle) {
    return BoundingBox(
        circle.center() - Vec2::filled(circle.radius()),
        circle.center() + Vec2::filled(circle.radius())
    );
}

inline BoundingBox bounding_box(const Capsule& capsule) {
    BoundingBox bottom = bounding_box(Circle(capsule.bottom(), capsule.radius()));
    BoundingBox top = bounding_box(Circle(capsule.top(), capsule.radius()));
    return bottom.combined(top);
}

inline BoundingBox bounding_box(const Polygon& polygon) {
    return BoundingBox(polygon);
}

inline BoundingBox bounding_box(ObjectId collider_id, const ColliderPool& colliders) {
    ColliderType type = colliders.get_type(collider_id);
    switch (type) {
        case ColliderType::Circle:
            return bounding_box(colliders.circle(collider_id));
        case ColliderType::Capsule:
            return bounding_box(colliders.capsule(collider_id));
        case ColliderType::Polygon:
            return bounding_box(colliders.polygon(collider_id));
    }

    SIMU_ASSERT(false, "Unknown collider type");
}

} // namespace simu
