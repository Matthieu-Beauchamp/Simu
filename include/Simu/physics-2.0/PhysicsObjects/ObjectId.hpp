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

#include "Simu/config.hpp"
#include <functional>

namespace simu
{

class ObjectId
{
    std::uint32_t _id;

public:

    enum ObjectType : std::uint32_t
    {
        DynamicPhysicsObject = 1,
        StaticPhysicsObject  = 2,
        Collider             = 3
    };

    // Reserved for tagging in BVH nodes
    static constexpr std::uint32_t reserved_bit = 0x80000000;

    // Incremented when recycling the id.
    // Assumes that any references to a deleted object are gone after 8 generations
    static constexpr std::uint32_t generation_mask = 0x70000000;
    static constexpr std::uint32_t last_generation = generation_mask >> 28;
    static_assert(last_generation == 7);

    static constexpr std::uint32_t type_mask = 0x0C000000;
    static constexpr std::uint32_t last_type = type_mask >> 26;
    static_assert(last_type == 3);

    static constexpr std::uint32_t id_mask = ~reserved_bit;
    static constexpr std::uint32_t index_mask = ~(reserved_bit | generation_mask | type_mask);

    static constexpr ObjectId      unset() { return ObjectId(0); }
    static constexpr std::uint32_t first_id = 1;

    explicit constexpr ObjectId(std::uint32_t id) : _id(id & id_mask) {}
    explicit constexpr ObjectId(std::uint32_t generation, std::uint32_t type, std::uint32_t id)
        : _id((generation << 28) | (type << 26) | (id & index_mask)) {
        SIMU_ASSERT(generation <= last_generation, "Invalid generation");
        SIMU_ASSERT(type > 0, "Missing type");
        SIMU_ASSERT(type <= last_type, "Invalid type");
        SIMU_ASSERT(id <= index_mask, "Invalid id");
    }

    // Use to identify an object uniquely
    [[nodiscard]] std::uint32_t id() const { return _id; }

    // Use to place an object inside an array in the slot of the previous generation after subtracting 1
    [[nodiscard]] std::uint32_t as_index() const { return _id & index_mask; }

    // Use to identify the type of object
    [[nodiscard]] ObjectType type() const {
        return static_cast<ObjectType>((_id & type_mask) >> 26);
    }

    // The generation of the object
    [[nodiscard]] std::uint32_t generation() const {
        return (_id & generation_mask) >> 28;
    }

    // Do not compare directly with ObjectId::unset()
    [[nodiscard]] bool is_valid() const { return static_cast<bool>(*this); }
    explicit operator bool() const { return as_index() != unset().as_index(); }

    bool operator==(const ObjectId& other) const = default;
};

} // namespace simu

template <>
struct std::hash<simu::ObjectId>
{
    std::size_t operator()(const simu::ObjectId& s) const noexcept {
        return std::hash<std::uint32_t>{}(s.id());
    }
};
