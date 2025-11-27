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

#include "Simu/config.hpp"
#include "Simu/entities/Entity.hpp"
#include "Simu/physics-2.0/collision/colliders/BoundingBox.hpp"

namespace simu
{

namespace internal
{

class BvhNodeData
{
    static constexpr std::uint64_t left_mask  = 0xFFFFFFFFu;
    static constexpr std::uint64_t right_mask = left_mask << 32;
    std::uint64_t                  bits;

    explicit BvhNodeData(std::uint64_t bits) : bits{bits} {}

public:

    static BvhNodeData makeLeaf(uint64_t id) {
        return BvhNodeData(Entity::reserved_bit | id);
    }

    static BvhNodeData makeInternal(uint32_t L, uint32_t R) {
        return BvhNodeData(
            static_cast<std::uint64_t>(L) | (static_cast<std::uint64_t>(R) << 32)
        );
    }

    [[nodiscard]] bool isLeaf() const { return bits >> 63; }

    [[nodiscard]] Entity leaf() const { return Entity(bits); }
    void setLeaf(Entity e) { bits = e.id() | Entity::reserved_bit; }

    [[nodiscard]] std::uint32_t left() const { return bits & left_mask; }
    void setLeft(std::uint32_t L) { bits = (bits & ~left_mask) | L; }

    [[nodiscard]] std::uint32_t right() const { return bits >> 32; }
    void                        setRight(std::uint32_t R) {
        bits = (bits & ~right_mask) | (static_cast<std::uint64_t>(R) << 32);
    }
};

} // namespace internal

class DynamicBvh
{
    struct Node
    {
        BoundingBox bounds;
        internal::BvhNodeData data;
    };

public:

private:
};

} // namespace simu
