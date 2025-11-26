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
#include "BoundingBox.hpp"
#include "Polygon.hpp"
#include "Circle.hpp"
#include "Capsule.hpp"

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

// TODO: In case of no contact, store the separating axis as the normal.
//          It can be reused in the next timestep to quickly determine no contact
template <std::size_t max_contacts>
struct Contacts
{
    static constexpr Contacts none() { return Contacts{}; }

    /// The unit normal pointing out of the `a` body
    Vec2 normal;

    /// The contacts on body a
    std::array<Vec2, max_contacts> contacts_a;
    /// The contacts on body b
    std::array<Vec2, max_contacts> contacts_b;

    /// Actual number of contacts, 0 if no collision
    Uint32 n_contacts = 0;
};

Contacts<1> collides(const Circle& a, const Circle& b);

Contacts<1> collides(const Circle& a, const Capsule& b);

Contacts<1> collides(const Circle& a, const Polygon& b);

Contacts<2> collides(const Capsule& a, const Capsule& b, float epsilon);

Contacts<2> collides(const Capsule& a, const Polygon& b, float epsilon);

Contacts<2> collides(const Polygon& a, const Polygon& b, float epsilon);

} // namespace simu
