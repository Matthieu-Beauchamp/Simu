////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2023 Matthieu Beauchamp-Boulay
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

#include <type_traits>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"

#include "Simu/physics/PhysicsObject.hpp"
#include "Simu/physics/Transform.hpp"
#include "Simu/physics/Collider.hpp"
#include "Simu/physics/Material.hpp"
#include "Simu/physics/ConstraintInterfaces.hpp"

#include "Simu/utility/View.hpp"

namespace simu
{

class Constraint;

struct BodyDescriptor
{
    Polygon  polygon;
    Material material{};

    Vec2  position{};
    float orientation{};

    float dominance = 1.f;
};

struct MassProperties
{
    MassProperties() = default;
    MassProperties(const GeometricProperties& properties, float density)
        : centroid{properties.centroid},
          mass{properties.area * density},
          inertia{properties.momentOfArea * density}
    {
        if (density < 0)
            throw simu::Exception{"Invalid density"};
    }

    Vec2  centroid;
    float mass;
    float inertia;
};


class PhysicsWorld;

class PhysicsBody : public PhysicsObject
{
public:

    PhysicsBody(const BodyDescriptor& descriptor)
        : position_{descriptor.position},
          orientation_{descriptor.orientation},
          material_{descriptor.material},
          properties_{
              descriptor.polygon.properties(),
              descriptor.material.density},
          collider_{
              descriptor.polygon,
              Transform::transform(orientation_, position_)},
          dominance_{descriptor.dominance}
    {
    }

    ~PhysicsBody() override = default;

    PhysicsBody(const PhysicsBody& other)            = default;
    PhysicsBody& operator=(const PhysicsBody& other) = default;

    PhysicsBody(PhysicsBody&& other)            = default;
    PhysicsBody& operator=(PhysicsBody&& other) = default;

    void applyImpulse(Vec2 force, float dt, Vec2 whereFromCentroid = Vec2{0, 0})
    {
        applyImpulse(force * dt, whereFromCentroid);
    }

    void applyImpulse(Vec2 impulse, Vec2 whereFromCentroid = Vec2{0, 0})
    {
        velocity_ += impulse / properties_.mass;
        angularVelocity_
            += cross(whereFromCentroid, impulse) / properties_.inertia;
    }

    void setVelocity(Vec2 velocity)
    {
        velocity_ = velocity;
        wake();
    }

    const Vec2& velocity() const { return velocity_; }

    void setAngularVelocity(float angularVelocity)
    {
        angularVelocity_ = angularVelocity;
        wake();
    }

    const float& angularVelocity() const { return angularVelocity_; }

    const Vec2& position() const { return position_; }
    float       orientation() const { return orientation_; }

    const Collider& collider() const { return collider_; }

    Mat3 toWorldSpace() const
    {
        return Transform::transformAround(
            orientation_,
            position_,
            localProperties().centroid
        );
    }
    Mat3 toLocalSpace() const
    {
        return Transform::transformAround(
            -orientation_,
            -position_,
            properties().centroid
        );
    }

    MassProperties properties() const
    {
        MassProperties transformedProperties{properties_};
        transformedProperties.centroid = toWorldSpace() * properties_.centroid;
        return transformedProperties;
    }

    MassProperties localProperties() const { return properties_; }

    Material material() const { return material_; }

    bool isStructural() const { return dominance() == 0.f; }
    bool interactsAsStructural() const; // true if is structural in all its constraints

    float dominance() const { return dominance_; }

    void step(float dt)
    {
        position_ += velocity_ * dt;
        orientation_ += angularVelocity_ * dt;

        collider_.update(toWorldSpace());
    }

    bool isAsleep() const { return isAsleep_; }
    void wake() { isAsleep_ = false; }

    auto constraints()
    {
        return makeView(constraints_.begin(), constraints_.end());
    }

    auto constraints() const
    {
        return makeView(constraints_.begin(), constraints_.end());
    }

private:

    friend class PhysicsWorld;

    template <Uint32 n>
    friend class Bodies;

    bool isImmobile(float velocityTreshold, float angularVelocityTreshold) const
    {
        return normSquared(velocity()) < velocityTreshold * velocityTreshold
               && angularVelocity() < angularVelocityTreshold;
    }

    void
    updateTimeImmobile(float dt, float velocityTreshold, float angularVelocityTreshold)
    {
        if (isImmobile(velocityTreshold, angularVelocityTreshold))
            timeImmobile_ += dt;
        else
            timeImmobile_ = 0.f;
    }

    bool canSleep(float minTimeImmobile) const
    {
        return timeImmobile_ >= minTimeImmobile;
    }

    void sleep() { isAsleep_ = true; }


    Vec2 position_;
    Vec2 velocity_{};

    float orientation_;
    float angularVelocity_{};

    Material material_;

    MassProperties properties_;
    Collider       collider_;

    float dominance_;

    bool  isAsleep_     = false;
    float timeImmobile_ = 0.f;

    // on one hand, bodies should not know about their constraints,
    // on the other, putting an std::unordered_multimap<PhysicsBody*, Constraint*> in PhysicsWorld feels pretty ugly
    std::vector<Constraint*> constraints_;
};


} // namespace simu
