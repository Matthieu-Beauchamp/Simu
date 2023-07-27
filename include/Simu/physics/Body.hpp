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
#include "Simu/physics/BodyTree.hpp"
#include "Simu/physics/Transform.hpp"
#include "Simu/physics/Collider.hpp"
#include "Simu/physics/Material.hpp"
#include "Simu/physics/ConstraintInterfaces.hpp"

#include "Simu/utility/View.hpp"

namespace simu
{

class Constraint;

////////////////////////////////////////////////////////////
/// \brief Describes a Body for construction
///
////////////////////////////////////////////////////////////
struct BodyDescriptor
{
    ////////////////////////////////////////////////////////////
    /// \brief The local-space Geometry for the Body. Does not need to be centered on origin.
    ///
    /// It is recommended to center the Polygon on the origin to make placing it more intuitive
    ///
    /// \see orientation
    ////////////////////////////////////////////////////////////
    Polygon polygon;

    ////////////////////////////////////////////////////////////
    /// \brief The Material of the Body. Affects how it interacts with other bodies.
    ///
    ////////////////////////////////////////////////////////////
    Material material{};

    ////////////////////////////////////////////////////////////
    /// \brief The initial position of the Body. This is a translation of the local-space geometry.
    ///
    ////////////////////////////////////////////////////////////
    Vec2 position{};

    ////////////////////////////////////////////////////////////
    /// \brief The initial orientation of the Body. This is a rotation around the local-space polygon's centroid.
    ///
    ////////////////////////////////////////////////////////////
    float orientation{};

    ////////////////////////////////////////////////////////////
    /// \brief dominance scales the inverse mass of the Body when interacting in constraints.
    ///
    /// A dominance of 0 means that the body has infinite mass and is considered
    /// structural. Ie when constrained with another body, only the other bodies will be acted upon.
    ///
    /// A structural Body is not affected by any ForceField. Otherwise,
    ///     ForceFields should not take dominance into account.
    ///
    /// This parameter can be overriden for specific Constraints when passing Bodies.
    ///
    /// \see Bodies
    ////////////////////////////////////////////////////////////
    float dominance = 1.f;
};

////////////////////////////////////////////////////////////
/// \brief Mass properties of a Body
///
/// The mass and inertia are always strictly positive for a Body.
///
////////////////////////////////////////////////////////////
struct MassProperties
{
    MassProperties() = default;
    MassProperties(const GeometricProperties& properties, float density)
        : centroid{properties.centroid},
          mass{properties.area * density},
          inertia{properties.momentOfArea * density}
    {
        SIMU_ASSERT(density > 0, "Invalid density");
    }

    Vec2  centroid;
    float mass;
    float inertia;
};


class World;

class Body : public PhysicsObject
{
public:

    typedef FreeListAllocator<Constraint*> Alloc;

    ////////////////////////////////////////////////////////////
    /// \brief Constructs a Body
    ///
    ////////////////////////////////////////////////////////////
    Body(const BodyDescriptor& descriptor)
        : position_{descriptor.position},
          orientation_{descriptor.orientation},
          material_{descriptor.material},
          properties_{
              descriptor.polygon.properties(),
              descriptor.material.density},
          collider_{descriptor.polygon, Transform::identity(), Alloc{}},
          dominance_{descriptor.dominance}
    {
        update();
    }

    void setAllocator(const PhysicsAlloc& alloc) override
    {
        replaceAllocator(constraints_, alloc);
        collider_.replaceAlloc(alloc);
    }

    ~Body() override = default;

    ////////////////////////////////////////////////////////////
    /// \brief Applies a force for some time on an object
    ///
    /// whereFromCentroid does not need to lie inside the Body's world space geometry.
    ///
    ////////////////////////////////////////////////////////////
    void applyForce(Vec2 force, float dt, Vec2 whereFromCentroid = Vec2{0, 0})
    {
        applyImpulse(force * dt, whereFromCentroid);
    }

    ////////////////////////////////////////////////////////////
    /// \brief Applies an impulse on an object
    ///
    /// whereFromCentroid does not need to lie inside the Body's world space geometry.
    ///
    ////////////////////////////////////////////////////////////
    void applyImpulse(Vec2 impulse, Vec2 whereFromCentroid = Vec2{0, 0})
    {
        setVelocity(velocity() + impulse / properties_.mass);
        setAngularVelocity(
            angularVelocity()
            + cross(whereFromCentroid, impulse) / properties_.inertia
        );
    }

    ////////////////////////////////////////////////////////////
    /// \brief The Body's current linear velocity
    ///
    ////////////////////////////////////////////////////////////
    const Vec2& velocity() const { return velocity_; }

    ////////////////////////////////////////////////////////////
    /// \brief The Body's current angular velocity in radians/s
    ///
    ////////////////////////////////////////////////////////////
    const float& angularVelocity() const { return angularVelocity_; }

    ////////////////////////////////////////////////////////////
    /// \brief Change the Body's linear velocity to velocity
    ///
    ////////////////////////////////////////////////////////////
    void setVelocity(Vec2 velocity)
    {
        velocity_ = velocity;
        wake();
    }

    ////////////////////////////////////////////////////////////
    /// \brief Change the Body's angularVelocity to angularVelocity in radians/s
    ///
    ////////////////////////////////////////////////////////////
    void setAngularVelocity(float angularVelocity)
    {
        angularVelocity_ = angularVelocity;
        wake();
    }

    ////////////////////////////////////////////////////////////
    /// \brief The Body's world space position
    ///
    /// This is an offset of the Body's local space geometry
    ///
    ////////////////////////////////////////////////////////////
    const Vec2& position() const { return position_; }

    ////////////////////////////////////////////////////////////
    /// \brief The Body's world space orientation in radians
    ///
    /// The orientation rotates the Body's local space geometry around its centroid
    ///
    ////////////////////////////////////////////////////////////
    float orientation() const { return orientation_; }

    ////////////////////////////////////////////////////////////
    /// \brief Returns a read only reference to the Body's Collider.
    ///
    ////////////////////////////////////////////////////////////
    const Collider& collider() const { return collider_; }

    ////////////////////////////////////////////////////////////
    /// \brief Transformation matrix to convert from the Body's local space to world space
    ///
    ////////////////////////////////////////////////////////////
    const Transform& toWorldSpace() const { return toWorldSpace_; }

    ////////////////////////////////////////////////////////////
    /// \brief Transformation matrix to convert from world space to the Body's local space
    ///
    ////////////////////////////////////////////////////////////
    Transform toLocalSpace() const
    {
        return Transform::transformAround(-orientation_, -position_, centroid());
    }

    float mass() const { return properties_.mass; }
    float inertia() const { return properties_.inertia; }
    Vec2  centroid() const { return worldProperties().centroid; }

    ////////////////////////////////////////////////////////////
    /// \brief The MassProperties of the Body with the centroid given in world space
    ///
    ////////////////////////////////////////////////////////////
    MassProperties worldProperties() const
    {
        MassProperties transformedProperties{properties_};
        transformedProperties.centroid = toWorldSpace() * properties_.centroid;
        return transformedProperties;
    }

    ////////////////////////////////////////////////////////////
    /// \brief The MassProperties of the Body, ignoring its current world position.
    ///
    ////////////////////////////////////////////////////////////
    MassProperties localProperties() const { return properties_; }

    ////////////////////////////////////////////////////////////
    /// \brief The Body's Material
    ///
    ////////////////////////////////////////////////////////////
    Material material() const { return material_; }

    ////////////////////////////////////////////////////////////
    /// \brief Whether the body is structural or not according to its dominance
    ///
    /// \see BodyDescriptor::dominance
    ////////////////////////////////////////////////////////////
    bool isStructural() const { return dominance() == 0.f; }

    ////////////////////////////////////////////////////////////
    /// \brief true if the body is structural in all of its constraints
    ///
    /// This is mostly used internally.
    ///
    /// \see BodyDescriptor::dominance
    ////////////////////////////////////////////////////////////
    bool interactsAsStructural() const;

    ////////////////////////////////////////////////////////////
    /// \brief The Body's dominance
    ///
    /// \see BodyDescriptor::dominance
    ////////////////////////////////////////////////////////////
    float dominance() const { return dominance_; }

    // not currently used.
    bool isAsleep() const { return isAsleep_; }
    void wake() { isAsleep_ = false; }

    // TODO: This allows users to modify this->constraints_
    //  (gives Constraint*&)
    ////////////////////////////////////////////////////////////
    /// \brief Returns an iterable view of Constraint*
    ///
    /// ie: for (Constraint* c : body.constraints())
    ///
    ////////////////////////////////////////////////////////////
    auto constraints()
    {
        return makeView(constraints_.begin(), constraints_.end());
    }

    ////////////////////////////////////////////////////////////
    /// \brief Returns an iterable view of const Constraint*
    ///
    /// ie: for (const Constraint* c : body.constraints())
    ///
    ////////////////////////////////////////////////////////////
    auto constraints() const
    {
        return makeView(constraints_.begin(), constraints_.end());
    }

private:

    friend class World;

    template <Uint32 n>
    friend class Bodies;


    void step(float dt)
    {
        position_ += velocity_ * dt;
        orientation_ += angularVelocity_ * dt;

        update();
    }

    void update()
    {
        toWorldSpace_ = Transform::transformAround(
            orientation_,
            position_,
            localProperties().centroid
        );

        collider_.update(toWorldSpace());
    }

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

    std::vector<Constraint*, Alloc> constraints_;
    typename BodyTree::iterator treeLocation_;

    Transform toWorldSpace_;
};


} // namespace simu
