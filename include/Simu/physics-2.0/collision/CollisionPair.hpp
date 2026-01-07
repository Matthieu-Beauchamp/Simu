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

#include "Simu/entities/Entities.hpp"
#include "colliders/collisions.hpp"

namespace simu
{


class CollisionPair
{
public:

    ObjectId a;
    ObjectId b;

    CollisionPair(ObjectId a, ObjectId b) : a{a}, b{b} {
        if (a.id() > b.id()) {
            std::swap(this->a, this->b);
        }
    }

    [[nodiscard]] std::uint64_t id() const { return a.id() | (b.id() << 32); }

    bool operator==(const CollisionPair& other) const {
        return id() == other.id();
    }
};


} // namespace simu

template <>
struct std::hash<simu::CollisionPair>
{
    size_t operator()(const simu::CollisionPair& pair) const noexcept {
        return std::hash<std::uint64_t>{}(pair.id());
    }
};
