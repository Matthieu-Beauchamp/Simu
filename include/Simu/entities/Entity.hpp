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


#include <cstddef>
#include <functional>

namespace simu
{

namespace internal
{

class EntityGenerator;
class BvhNodeData;

} // namespace internal

class Entity
{
    // Reserved for tagging in BVH nodes
    static constexpr std::uint32_t reserved_bit    = 0x80000000;

    // TODO: Add id recycling
    static constexpr std::uint32_t generation_mask = 0x7F000000;

    // Allows a maximum of ~16 million objects
    static constexpr std::uint32_t id_mask = ~(reserved_bit | generation_mask);


    std::uint64_t _id;

    explicit Entity(std::uint32_t id) : _id(id & id_mask) {}
    friend internal::EntityGenerator;
    friend internal::BvhNodeData;

public:

    [[nodiscard]] std::uint32_t id() const { return _id; }

    bool operator==(const Entity& other) const = default;
};


namespace internal
{

class EntityGenerator
{
    std::uint32_t next_id = 1;

public:

    Entity create() { return Entity(next_id++); }
};

} // namespace internal

} // namespace simu

template <>
struct std::hash<simu::Entity>
{
    std::uint32_t operator()(const simu::Entity& s) const noexcept {
        return std::hash<std::uint32_t>{}(s.id());
    }
};
