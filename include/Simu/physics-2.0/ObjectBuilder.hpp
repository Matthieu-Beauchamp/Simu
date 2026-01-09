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
#include "collision.hpp"
#include "components/Mass.hpp"
#include "components/Position.hpp"
#include "components/Velocity.hpp"

namespace simu
{

class ObjectBuilder
{
    friend class Simulation;

    Position position_{};
    Velocity velocity_{};
    Mass     mass_{};

    ColliderType collider_type_ = ColliderType::Circle;
    union
    {
        Circle  circle_ = Circle(Vec2(0, 0), 0);
        Capsule capsule_;
        Polygon polygon_;
    };
    bool has_collider_ = false;

    float density_                   = 1;
    bool  compute_mass_from_geometry = true;

    bool is_static_ = false;

public:
    ObjectBuilder() = default;

    ObjectBuilder& set_position(const Position& position) {
        position_ = position;
        return *this;
    }
    ObjectBuilder& set_position(Vec2 pos, float angle = 0.f) {
        return set_position(Position(pos, angle));
    }
    ObjectBuilder& set_velocity(const Velocity& velocity) {
        velocity_ = velocity;
        return *this;
    }
    ObjectBuilder& set_velocity(Vec2 linear, float angular = 0.f) {
        return set_velocity(Velocity(linear, angular));
    }
    ObjectBuilder& set_mass(const Mass& mass) {
        mass_ = mass;
        return *this;
    }

    ObjectBuilder& set_collider(Circle circle) {
        collider_type_ = ColliderType::Circle;
        circle_        = circle;
        has_collider_  = true;
        return *this;
    }
    ObjectBuilder& set_collider(Capsule capsule) {
        collider_type_ = ColliderType::Capsule;
        capsule_       = capsule;
        has_collider_  = true;
        return *this;
    }
    ObjectBuilder& set_collider(Polygon polygon) {
        collider_type_ = ColliderType::Polygon;
        polygon_       = polygon;
        has_collider_  = true;
        return *this;
    }

    ObjectBuilder& set_density(float density) {
        this->density_                   = density;
        this->compute_mass_from_geometry = true;
        return *this;
    }

    ObjectBuilder& set_mass(Mass mass) {
        this->mass_                      = mass;
        this->compute_mass_from_geometry = false;
        return *this;
    }

    ObjectBuilder& set_static(bool is_static = true) {
        this->is_static_ = is_static;
        return *this;
    }
};

} // namespace simu
