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

#include <array>

#include "Simu/config.hpp"

#include "Simu/math/ShapeCollision.hpp"

#include "Simu/physics/PhysicsObject.hpp"
#include "Simu/physics/ColliderTree.hpp"
#include "Simu/physics/Profiler.hpp"

#include "Simu/utility/View.hpp"
#include "Simu/utility/PolymorphicList.hpp"


namespace simu
{

class Body;
struct BodyDescriptor;
class Collider;

class Constraint;
class ContactConstraint;

class ForceField;

class Island;

} // namespace simu


namespace details
{

// https://youngforest.github.io/2020/05/27/best-implement-to-use-pair-as-key-to-std-unordered-map-in-C/
// from boost (functional/hash):
// see http://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html template
template <typename T>
inline void hash_combine(std::size_t& seed, const T& val)
{
    seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// auxiliary generic functions to create a hash value using a seed
template <typename T>
inline void hash_val(std::size_t& seed, const T& val)
{
    hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t& seed, const T& val, const Types&... args)
{
    hash_combine(seed, val);
    hash_val(seed, args...);
}

template <typename... Types>
inline std::size_t hash_val(const Types&... args)
{
    std::size_t seed = 0;
    hash_val(seed, args...);
    return seed;
}

} // namespace details

namespace std
{

template <>
struct hash<std::array<simu::Collider*, 2>>
{
    size_t operator()(const std::array<simu::Collider*, 2>& colliders) const
    {
        return ::details::hash_val(colliders[0], colliders[1]);
    }
};

} // namespace std

#include <unordered_map>


namespace simu
{

////////////////////////////////////////////////////////////
/// \brief The World class makes the ForceField, Body and Constraint classes interact.
///
/// Collisions are detected discretely, and the corresponding constraints are
///     managed by the world.
///
///
/// The World owns all objects that are put into it.
/// For object lifetime management, see PhysicsObject.
///
////////////////////////////////////////////////////////////
class World
{
public:

    typedef typename PhysicsObject::PhysicsAlloc Alloc;

    typedef PolymorphicList<ContactConstraint, Alloc> ContactList;
    typedef typename ContactList::iterator            ContactIterator;

    // must construct a single contact constraint in the contact list and return
    //  its iterator.
    typedef std::function<ContactIterator(Collider&, Collider&, CollisionCallback, ContactList&)>
        ContactFactory;

    static ContactFactory defaultContactFactory;

    ////////////////////////////////////////////////////////////
    /// \brief Construct an empty world
    ///
    ////////////////////////////////////////////////////////////
    World(ContactFactory makeContact = defaultContactFactory);

    World(const World& other) = delete;
    World(World&& other)      = delete;

    void clear();

    void setContactFactory(ContactFactory makeContact);

    auto&       shapeCollider() { return shapeCollider_; }
    const auto& shapeCollider() const { return shapeCollider_; }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the Bodies in the world
    ///
    /// ie for ( [const] Body& body : world.bodies())
    ///
    ////////////////////////////////////////////////////////////
    auto bodies() { return makeView(bodies_); }
    auto bodies() const { return makeView(bodies_); }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the ForceFields in the world
    ///
    /// ie for ( [const] ForceField& force : world.forceFields())
    ///
    ////////////////////////////////////////////////////////////
    auto forceFields() { return makeView(forces_); }
    auto forceFields() const { return makeView(forces_); }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the Constraints in the world
    ///
    /// ie for ( [const] Constraint& constraint : world.constraints())
    ///
    ////////////////////////////////////////////////////////////
    auto constraints() { return makeView(constraints_); }
    auto constraints() const { return makeView(constraints_); }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the ContactConstraints in the world
    ///
    /// ie for ( [const] ContactConstraint& contact : world.contacts())
    ///
    ////////////////////////////////////////////////////////////
    auto contacts() { return makeView(contacts_); }
    auto contacts() const { return makeView(contacts_); }


    ////////////////////////////////////////////////////////////
    /// \brief Construct a Body and puts it in the world
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<Body> T, class... Args>
    T*    makeBody(Args&&... args);
    Body* makeBody(const BodyDescriptor& descr);

    ////////////////////////////////////////////////////////////
    /// \brief Construct a Constraint and puts it in the world
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<Constraint> T, class... Args>
    T* makeConstraint(Args&&... args);

    ////////////////////////////////////////////////////////////
    /// \brief Construct a ForceField and puts it in the world
    ///
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<ForceField> T, class... Args>
    T* makeForceField(Args&&... args);

    ////////////////////////////////////////////////////////////
    /// \brief Makes the world progress in time.
    ///
    /// ForceFields are applied to non-structural bodies, contacts are updated,
    /// killed objects are removed, constraints are enforce and bodies are moved.
    ///
    ////////////////////////////////////////////////////////////
    void step(float dt);

    struct Settings
    {
        ////////////////////////////////////////////////////////////
        /// \brief Number of velocity solver iterations
        ////////////////////////////////////////////////////////////
        Uint32 nVelocityIterations = 8;

        ////////////////////////////////////////////////////////////
        /// \brief Number of position solver iterations
        ////////////////////////////////////////////////////////////
        Uint32 nPositionIterations = 2;

        // TODO: Epsilons, iterate until change < epsilon or iter >= maxIter.

        ////////////////////////////////////////////////////////////
        /// \brief Enable constraints to guess impulse based on previous step.
        ////////////////////////////////////////////////////////////
        bool enableWarmstarting = true;


        // below is currently unused
        bool  enableSleeping                = false;
        float inactivitySleepDelay          = 1.f;
        float velocitySleepThreshold        = 1e-3f;
        float angularVelocitySleepThreshold = 1e-3f;
    };

    ////////////////////////////////////////////////////////////
    /// \brief Updates the world's settings
    ///
    ////////////////////////////////////////////////////////////
    void updateSettings(const Settings& settings);

    ////////////////////////////////////////////////////////////
    /// \brief Read the world's settings
    ///
    ////////////////////////////////////////////////////////////
    const Settings& settings() const { return settings_; }


    template <Callable<void(Body*)> F>
    void forEachIn(BoundingBox box, const F& func);

    template <Callable<void(Body*)> F>
    void forEachAt(Vec2 point, const F& func);

    Profiler&       profiler() { return profiler_; }
    const Profiler& profiler() const { return profiler_; }

private:

    friend Body;
    void addCollider(Collider* collider);
    void removeCollider(Collider* collider);

    ContactIterator makeContactConstraint(Collider& first, Collider& second);

    template <std::derived_from<PhysicsObject> T, class A, class... Args>
    UniquePtr<T> makeObject(A& alloc, Args&&... args);


    void applyForces(float dt);

    struct Cleaner;
    void cleanup();

    void updateBodies(float dt);


    Alloc miscAlloc_{};

    typedef ReboundTo<Alloc, UniquePtr<Body>> BodyAlloc;
    BodyAlloc                                 bAlloc_{miscAlloc_};

    typedef ReboundTo<Alloc, UniquePtr<Constraint>> ConstraintAlloc;
    ConstraintAlloc                                 cAlloc_{bAlloc_};

    typedef ReboundTo<Alloc, UniquePtr<ForceField>> ForceFieldAlloc;
    ForceFieldAlloc                                 fAlloc_{miscAlloc_};

    PolymorphicList<Constraint, Alloc> constraints_{cAlloc_};
    ContactList                        contacts_{cAlloc_};

    PolymorphicList<ForceField, Alloc> forces_{fAlloc_};

    ColliderTree                 colliderTree_{bAlloc_};
    PolymorphicList<Body, Alloc> bodies_{bAlloc_};

    struct ContactStatus
    {
        ContactIterator existingContact{};
        bool            hit = false;
    };

    typedef std::unordered_map<
        std::array<simu::Collider*, 2>,
        ContactStatus,
        std::hash<std::array<simu::Collider*, 2>>,
        std::equal_to<std::array<simu::Collider*, 2>>,
        ReboundTo<Alloc, std::pair<const std::array<simu::Collider*, 2>, ContactStatus>>>
        ContactTable;

    ContactTable contactsTable_{miscAlloc_};

    typename ContactTable::iterator
    inContacts(const std::array<simu::Collider*, 2>& colliders);


    Settings settings_;
    Profiler profiler_;

    ContactFactory       makeContactConstraint_;
    ShapeCollider<Alloc> shapeCollider_{miscAlloc_};
};


} // namespace simu

#include "Simu/physics/World.inl.hpp"
