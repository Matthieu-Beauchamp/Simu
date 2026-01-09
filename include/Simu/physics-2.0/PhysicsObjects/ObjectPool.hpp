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


#include <vector>

namespace simu
{

namespace detail
{
template <class T>
concept PoolableObject = requires(T& obj) {
    { obj.id } -> std::same_as<ObjectId&>;
} && std::is_default_constructible_v<T>;

} // namespace detail

template <detail::PoolableObject T, ObjectId::ObjectType object_type>
class ObjectPool
{
    std::vector<T>        _objects;
    std::vector<ObjectId> free_ids;

    std::uint32_t next_id = ObjectId::first_id;

    static constexpr std::size_t initial_size = 64;

public:

    ObjectPool() { _objects.reserve(initial_size); }

    ObjectId allocate() {
        if (free_ids.empty()) {
            _objects.emplace_back();
            return ObjectId(0, object_type, next_id++);
        }

        ObjectId id = free_ids.back();
        free_ids.pop_back();
        return ObjectId(id.generation() + 1, object_type, id.as_index());
    }

    void give_back(ObjectId id) {
        SIMU_ASSERT(id.type() == object_type, "ObjectPool::give_back object type mismatch");
        free_ids.push_back(id);
        _objects[id.as_index() - 1] = T{};
    }

    T& operator[](ObjectId id) {
        SIMU_ASSERT(id.type() == object_type, "Object type mismatch");
        // TODO: Can't valide on write access since it could be initializing the object
        // SIMU_ASSERT(_objects[id.as_index() - 1].id.is_valid(), "No such object");
        // SIMU_ASSERT(_objects[id.as_index() - 1].id == id, "Object changed generation");
        return _objects[id.as_index() - 1];
    }
    const T& operator[](ObjectId id) const {
        SIMU_ASSERT(id.type() == object_type, "Object type mismatch");
        SIMU_ASSERT(_objects[id.as_index() - 1].id.is_valid(), "No such object");
        SIMU_ASSERT(_objects[id.as_index() - 1].id == id, "Object changed generation");
        return _objects[id.as_index() - 1];
    }

    auto objects() {
        return std::ranges::views::filter(_objects, [](const T& obj) {
            return obj.id.is_valid();
        });
    }

    auto objects() const {
        return std::ranges::views::filter(_objects, [](const T& obj) {
            return obj.id.is_valid();
        });
    }
};

} // namespace simu
