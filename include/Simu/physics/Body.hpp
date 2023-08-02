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

#include "Simu/utility/View.hpp"

namespace simu
{

class Constraint;
class ContactConstraint;

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
class Mass
{
public:

    Mass() = default;

    Mass(float mass, float inertia)
        : invMass_{1.f / mass}, invInertia_{1.f / inertia}
    {
        SIMU_ASSERT(mass > 0, "Invalid mass");
        SIMU_ASSERT(inertia > 0, "Invalid inertia");
    }

    Mass(const GeometricProperties& properties, float density)
        : Mass(properties.area * density, properties.momentOfArea * density)
    {
        SIMU_ASSERT(density > 0, "Invalid density");
    }

    float invMass() const { return invMass_; }
    float invInertia() const { return invInertia_; }

    float mass() const { return 1.f / invMass_; }
    float inertia() const { return 1.f / invInertia_; }

private:

    float invMass_;
    float invInertia_;
};


// TODO: By enforcing that local space geometry has its centroid at {0,0}
//  we can simplify the toWorldSpace() transform and avoid storing the local centroid
//
// This represent a transformation from one frame to another, and provides a way
//  to access current position in world space.
class Position
{
public:

    Position(Vec2 position, float orientation, Vec2 localSpaceCentroid)
        : toWorldSpace_{Rotation{orientation}, Translation{position}},
          localCentroid_{localSpaceCentroid}
    {
    }

    Vec2  position() const { return toWorldSpace_.translation().offset(); }
    float orientation() const { return toWorldSpace_.rotation().theta(); }
    Vec2  centroid() const { return toWorldSpace() * localCentroid_; }

    void advance(Vec2 dPos, float dTheta)
    {
        toWorldSpace_.translation() *= Translation(dPos);
        toWorldSpace_.rotation() *= Rotation(dTheta);
    }

    Transform toWorldSpace() const
    {
        return Translation{localCentroid_}
               * toWorldSpace_* Translation{-localCentroid_};
    }

    Transform toLocalSpace() const { return toWorldSpace().inverse(); }


private:

    Transform toWorldSpace_;
    Vec2      localCentroid_{};
};

class World;

class Body : public PhysicsObject
{
public:

    typedef typename PhysicsObject::PhysicsAlloc Alloc;

    ////////////////////////////////////////////////////////////
    /// \brief Constructs a Body
    ///
    ////////////////////////////////////////////////////////////
    Body(const BodyDescriptor& descriptor)
        : position_{descriptor.position, descriptor.orientation, descriptor.polygon.properties().centroid},
          material_{descriptor.material},
          mass_{descriptor.polygon.properties(), descriptor.material.density},
          collider_{descriptor.polygon, Transform::identity(), Alloc{}},
          dominance_{descriptor.dominance}
    {
        update();
    }

    void setAllocator(const PhysicsAlloc& alloc) override
    {
        replaceAllocator(constraints_, alloc);
        replaceAllocator(contacts_, alloc);
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
        setVelocity(velocity() + impulse * invMass());
        setAngularVelocity(
            angularVelocity() + cross(whereFromCentroid, impulse) * invInertia()
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
    const Vec2& position() const { return position_.position(); }

    ////////////////////////////////////////////////////////////
    /// \brief The Body's world space orientation in radians
    ///
    /// The orientation rotates the Body's local space geometry around its centroid
    ///
    ////////////////////////////////////////////////////////////
    float orientation() const { return position_.orientation(); }

    ////////////////////////////////////////////////////////////
    /// \brief Returns a read only reference to the Body's Collider.
    ///
    ////////////////////////////////////////////////////////////
    const Collider& collider() const { return collider_; }

    ////////////////////////////////////////////////////////////
    /// \brief Transformation matrix to convert from the Body's local space to world space
    ///
    ////////////////////////////////////////////////////////////
    Transform toWorldSpace() const { return position_.toWorldSpace(); }

    ////////////////////////////////////////////////////////////
    /// \brief Transformation matrix to convert from world space to the Body's local space
    ///
    ////////////////////////////////////////////////////////////
    Transform toLocalSpace() const { return position_.toLocalSpace(); }

    Vec2 centroid() const { return position_.centroid(); }

    float invMass() const { return mass_.invMass(); }
    float invInertia() const { return mass_.invInertia(); }

    float mass() const { return mass_.mass(); }
    float inertia() const { return mass_.inertia(); }

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
    /// \brief true if the body is structural in all of its constraints and has no velocity
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
    auto constraints() { return makeView(constraints_); }

    ////////////////////////////////////////////////////////////
    /// \brief Returns an iterable view of const Constraint*
    ///
    /// ie: for (const Constraint* c : body.constraints())
    ///
    ////////////////////////////////////////////////////////////
    auto constraints() const { return makeView(constraints_); }

    auto contacts() { return makeView(contacts_); }
    auto contacts() const { return makeView(contacts_); }

protected:

    // for VisibleBody...
    typename BodyTree::iterator treeLocation_;

private:

    friend class Island;
    friend class SolverProxy;
    friend class World;

    friend class Bodies;


    void update() { collider_.update(toWorldSpace()); }

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


    Position position_;

    Vec2  velocity_{};
    float angularVelocity_{};

    Material material_;

    Mass     mass_;
    Collider collider_;

    float dominance_;

    bool  isAsleep_     = false;
    float timeImmobile_ = 0.f;

    std::vector<Constraint*, ReboundTo<Alloc, Constraint*>> constraints_;
    std::vector<ContactConstraint*, ReboundTo<Alloc, ContactConstraint*>> contacts_;

    static constexpr Int32 NO_INDEX   = -1;
    Int32                  proxyIndex = NO_INDEX;
};


} // namespace simu
