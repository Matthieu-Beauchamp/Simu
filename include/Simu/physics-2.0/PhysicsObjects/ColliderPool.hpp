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
#include "ObjectId.hpp"
#include "../collision/colliders/Capsule.hpp"
#include "../collision/colliders/Circle.hpp"
#include "../collision/colliders/ColliderType.hpp"
#include "../collision/colliders/Polygon.hpp"


#include <vector>

namespace simu
{


class ColliderPool
{
    static constexpr std::uint32_t type_mask = 0xC0000000;
    static constexpr std::uint32_t last_type = type_mask >> 30;
    static_assert(last_type == 3);

    // Must have at least as many available as possible objects
    static constexpr std::uint32_t index_mask = ~type_mask;
    static_assert(index_mask >= ObjectId::index_mask);

    std::vector<Circle>        circles;
    std::vector<std::uint32_t> free_circles;

    std::vector<Capsule>       capsules;
    std::vector<std::uint32_t> free_capsules;

    std::vector<Polygon>       polygons;
    std::vector<std::uint32_t> free_polygons;

    std::vector<uint32_t> object_map;
    std::vector<ObjectId> free_ids;

    std::uint32_t next_circle_id  = ObjectId::first_id;
    std::uint32_t next_capsule_id = ObjectId::first_id;
    std::uint32_t next_polygon_id = ObjectId::first_id;
    std::uint32_t next_id         = ObjectId::first_id;

    static constexpr ObjectId::ObjectType object_type = ObjectId::ObjectType::Collider;

    static constexpr std::size_t initial_size          = 64;
    static constexpr std::size_t initial_collider_size = 16;

public:

    ColliderPool() {
        circles.reserve(initial_collider_size);
        capsules.reserve(initial_collider_size);
        polygons.reserve(initial_collider_size);
        object_map.reserve(initial_size);
    }

    ObjectId allocate(ColliderType type) {
        return get_next_object_id(collider_index(allocate_collider_slot(type), type));
    }

    void give_back(ObjectId id) {
        SIMU_ASSERT(id.type() == object_type, "ObjectPool::give_back object type mismatch");
        free_ids.push_back(id);

        std::uint32_t collider_index = object_map[id.as_index() - 1];
        ColliderType type = static_cast<ColliderType>((collider_index & type_mask) >> 30);
        object_map[id.as_index() - 1] = -1;

        switch (type) {
            case ColliderType::Circle:
            {
                free_circles.push_back(collider_index & index_mask);
                circles[collider_index & index_mask] = Circle(Vec2(0, 0), 0);
                return;
            }
            case ColliderType::Capsule:
            {
                free_capsules.push_back(collider_index & index_mask);
                capsules[collider_index & index_mask] = Capsule(
                    Vec2(0, 0), Vec2(0, 0), 0
                );
                return;
            }
            case ColliderType::Polygon:
            {
                free_polygons.push_back(collider_index & index_mask);
                polygons[collider_index & index_mask] = Polygon({});
                return;
            }
        }
    }

    ColliderType get_type(ObjectId id) const {
        return static_cast<ColliderType>((object_map[id.as_index() - 1] & type_mask) >> 30);
    }

    Circle& circle(ObjectId id) {
        SIMU_ASSERT(get_type(id) == ColliderType::Circle, "object is not a circle");
        return circles[object_map[id.as_index() - 1] & index_mask];
    }
    const Circle& circle(ObjectId id) const {
        SIMU_ASSERT(get_type(id) == ColliderType::Circle, "object is not a circle");
        return circles[object_map[id.as_index() - 1] & index_mask];
    }

    Capsule& capsule(ObjectId id) {
        SIMU_ASSERT(get_type(id) == ColliderType::Capsule, "object is not a capsule");
        return capsules[object_map[id.as_index() - 1] & index_mask];
    }
    const Capsule& capsule(ObjectId id) const {
        SIMU_ASSERT(get_type(id) == ColliderType::Capsule, "object is not a capsule");
        return capsules[object_map[id.as_index() - 1] & index_mask];
    }

    Polygon& polygon(ObjectId id) {
        SIMU_ASSERT(get_type(id) == ColliderType::Polygon, "object is not a polygon");
        return polygons[object_map[id.as_index() - 1] & index_mask];
    }
    const Polygon& polygon(ObjectId id) const {
        SIMU_ASSERT(get_type(id) == ColliderType::Polygon, "object is not a polygon");
        return polygons[object_map[id.as_index() - 1] & index_mask];
    }

private:

    std::uint32_t allocate_collider_slot(ColliderType type) {
        switch (type) {
            case ColliderType::Circle:
            {
                if (free_circles.empty()) {
                    circles.emplace_back(Circle(Vec2(0, 0), 0));
                    return circles.size() - 1;
                }

                std::uint32_t index = free_circles.back();
                free_circles.pop_back();
                return index;
            }
            case ColliderType::Capsule:
            {
                if (free_capsules.empty()) {
                    capsules.emplace_back(Capsule(Vec2(0, 0), Vec2(0, 0), 0));
                    return capsules.size() - 1;
                }

                std::uint32_t index = free_capsules.back();
                free_capsules.pop_back();
                return index;
            }
            case ColliderType::Polygon:
            {
                if (free_polygons.empty()) {
                    polygons.emplace_back(Polygon({}));
                    return polygons.size() - 1;
                }

                std::uint32_t index = free_polygons.back();
                free_polygons.pop_back();
                return index;
            }
        }

        SIMU_ASSERT(false, "Invalid collider type");
    }

    std::uint32_t collider_index(std::size_t index, ColliderType type) const {
        return index | static_cast<std::uint32_t>(type) << 30;
    }

    ObjectId get_next_object_id(std::uint32_t collider_index) {
        if (free_ids.empty()) {
            object_map.emplace_back(collider_index);
            return ObjectId(0, object_type, next_id++);
        }

        ObjectId id = free_ids.back();
        free_ids.pop_back();
        object_map[id.as_index() - 1] = collider_index;
        return ObjectId(id.generation() + 1, object_type, id.as_index());
    }
};

} // namespace simu
