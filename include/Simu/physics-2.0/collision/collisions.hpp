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

inline Contacts<1> collides(const Circle& a, const Circle& b) {
    float min_dist = a.radius() + b.radius();
    Vec2  dir      = b.center() - a.center();
    bool  collides = normSquared(dir) <= min_dist * min_dist;
    dir            = normalized(dir);

    if (collides) {
        return {
            .normal     = dir,
            .contacts_a = {a.center() + a.radius() * dir},
            .contacts_b = {b.center() - b.radius() * dir},
            .n_contacts = 1
        };
    } else {
        return Contacts<1>::none();
    }
}

inline Contacts<1> collides(const Circle& a, const Capsule& b) {
    Vec2  axis     = normalized(b.top() - b.bottom());
    float axis_len = norm(b.top() - b.bottom());

    float circle_pos_along_axis = dot(a.center() - b.bottom(), axis);

    bool is_under = circle_pos_along_axis + a.radius() <= 0.f;
    bool is_over  = circle_pos_along_axis - a.radius() >= axis_len;
    if (is_under || is_over) {
        return Contacts<1>::none();
    }

    Vec2  perp         = simu::perp(axis);
    float perp_dist    = dot(a.center() - b.bottom(), perp);
    bool  is_near_axis = std::abs(perp_dist) <= a.radius() + b.radius();
    if (!is_near_axis) {
        return Contacts<1>::none();
    }

    if (circle_pos_along_axis <= b.radius()) {
        // Collision with bottom circle
        return collides(a, Circle(b.bottom() + axis * b.radius(), b.radius()));
    }

    if (circle_pos_along_axis >= axis_len - b.radius()) {
        // Collision with top circle
        return collides(a, Circle(b.top() - axis * b.radius(), b.radius()));
    }

    // Collision with axis
    if (perp_dist > 0.f) {
        return {
            .normal     = -perp,
            .contacts_a = {a.center() - perp * a.radius()},
            .contacts_b = {b.bottom() + axis * circle_pos_along_axis + perp * b.radius()},
            .n_contacts = 1
        };
    } else {
        return {
            .normal     = perp,
            .contacts_a = {a.center() + perp * a.radius()},
            .contacts_b = {b.bottom() + axis * circle_pos_along_axis - perp * b.radius()},
            .n_contacts = 1
        };
    }
}

inline bool collides(const Circle& a, const Polygon& b) {}

inline bool collides(const Capsule& a, const Capsule& b) {}

inline bool collides(const Capsule& a, const Polygon& b) {}

inline bool collides(const Polygon& a, const Polygon& b) {}

} // namespace simu
