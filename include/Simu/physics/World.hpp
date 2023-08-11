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

#include "Simu/config.hpp"
#include "Simu/utility/View.hpp"
#include "Simu/utility/Memory.hpp"

#include "Simu/physics/BodyTree.hpp"
#include "Simu/physics/Body.hpp"
#include "Simu/physics/Bodies.hpp"
#include "Simu/physics/ForceField.hpp"

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

#include <list>
#include <unordered_map>


namespace simu
{

class Constraint;
class ContactConstraint;
class Island;

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

    typedef ReboundTo<Alloc, UniquePtr<ContactConstraint>> ContactAlloc;

    typedef std::function<
        UniquePtr<ContactConstraint>(Collider&, Collider&, const ContactAlloc&)>
        ContactFactory;

    static ContactFactory defaultContactFactory;

    ////////////////////////////////////////////////////////////
    /// \brief Construct an empty world
    ///
    ////////////////////////////////////////////////////////////
    World(ContactFactory makeContact = defaultContactFactory);

    World(const World& other) = delete;
    World(World&& other)      = delete;

    void clear()
    {
        forces_.clear();
        contacts_.clear();
        constraints_.clear();
        bodies_.clear();
        colliderTree_.clear();
    }

    void setContactFactory(ContactFactory makeContact)
    {
        contacts_.clear();
        makeContactConstraint_ = makeContact;
    }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the Bodies in the world
    ///
    /// ie for ( [const] Body& body : world.bodies())
    ///
    ////////////////////////////////////////////////////////////
    auto bodies() { return makeView(bodies_, DoubleDereference{}); }
    auto bodies() const { return makeView(bodies_, DoubleDereference{}); }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the ForceFields in the world
    ///
    /// ie for ( [const] ForceField& force : world.forceFields())
    ///
    ////////////////////////////////////////////////////////////
    auto forceFields() { return makeView(forces_, DoubleDereference{}); }
    auto forceFields() const { return makeView(forces_, DoubleDereference{}); }

    ////////////////////////////////////////////////////////////
    /// \brief Gives a view over the Constraints in the world
    ///
    /// ie for ( [const] Constraint& constraint : world.constraints())
    ///
    ////////////////////////////////////////////////////////////
    auto constraints() { return makeView(constraints_, DoubleDereference{}); }
    auto constraints() const
    {
        return makeView(constraints_, DoubleDereference{});
    }


    ////////////////////////////////////////////////////////////
    /// \brief Construct a Body and puts it in the world
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<Body> T, class... Args>
    T* makeBody(Args&&... args)
    {
        auto body = makeObject<T>(bAlloc_, std::forward<Args>(args)...);

        T* b = static_cast<T*>(bodies_.emplace_back(std::move(body)).get());
        b->world_ = this;

        return b;
    }
    Body* makeBody(const BodyDescriptor& descr)
    {
        return makeBody<Body>(descr);
    }

    ////////////////////////////////////////////////////////////
    /// \brief Construct a Constraint and puts it in the world
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<Constraint> T, class... Args>
    T* makeConstraint(Args&&... args)
    {
        T* c = static_cast<T*>(
            constraints_
                .emplace_back(makeObject<T>(cAlloc_, std::forward<Args>(args)...))
                .get()
        );

        for (Body* body : c->bodies())
        {
            body->constraints_.emplace_back(c);
            body->wake();
        }

        return c;
    }

    ////////////////////////////////////////////////////////////
    /// \brief Construct a ForceField and puts it in the world
    ///
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<ForceField> T, class... Args>
    T* makeForceField(Args&&... args)
    {
        T* f = static_cast<T*>(
            forces_
                .emplace_back(makeObject<T>(fAlloc_, std::forward<Args>(args)...))
                .get()
        );

        if (f->domain().type == ForceField::DomainType::global)
            for (Body& body : bodies())
                body.wake();
        else
            colliderTree_.forEachIn(
                f->domain().region,
                [](ColliderTree::iterator it) { (*it)->body()->wake(); }
            );

        return f;
    }

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

        // below is currently unused
        bool enableWarmstarting = true;

        bool  enableSleeping                = false;
        float inactivitySleepDelay          = 1.f;
        float velocitySleepThreshold        = 1e-3f;
        float angularVelocitySleepThreshold = 1e-3f;
    };

    ////////////////////////////////////////////////////////////
    /// \brief Updates the world's settings
    ///
    ////////////////////////////////////////////////////////////
    void updateSettings(const Settings& settings)
    {
        if (!settings.enableSleeping && settings_.enableSleeping)
            for (Body& body : bodies())
                body.wake();

        settings_ = settings;
    }

    ////////////////////////////////////////////////////////////
    /// \brief Read the world's settings
    ///
    ////////////////////////////////////////////////////////////
    const Settings& settings() const { return settings_; }


    template <Callable<void(Body*)> F>
    void forEachIn(BoundingBox box, const F& func)
    {
        colliderTree_.forEachIn(box, [&](ColliderTree::iterator it) {
            func((*it)->body());
        });
    }

    template <Callable<void(Body*)> F>
    void forEachAt(Vec2 point, const F& func)
    {
        colliderTree_.forEachAt(point, [&](ColliderTree::iterator it) {
            func((*it)->body());
        });
    }

private:

    friend Body;
    void addCollider(Collider* collider)
    {
        collider->treeLocation_ = colliderTree_.emplace(
            collider->boundingBox(),
            collider
        );
    }

    void removeCollider(Collider* collider)
    {
        colliderTree_.erase(collider->treeLocation_);
    }

    UniquePtr<ContactConstraint>
    makeContactConstraint(Collider& first, Collider& second);

    template <std::derived_from<PhysicsObject> T, class A, class... Args>
    UniquePtr<T> makeObject(A& alloc, Args&&... args)
    {
        auto obj = makeUnique<T>(alloc, std::forward<Args>(args)...);
        obj->setAllocator(alloc);
        obj->onConstruction(*this);
        return obj;
    }


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


    ColliderTree colliderTree_{bAlloc_};

    typedef std::list<UniquePtr<Body>, BodyAlloc> BodyList;
    BodyList                                      bodies_{bAlloc_};


    typedef std::list<UniquePtr<Constraint>, ConstraintAlloc> ConstraintList;
    ConstraintList constraints_{cAlloc_};


    typedef std::list<UniquePtr<ForceField>, ForceFieldAlloc> ForceFieldList;

    ForceFieldList forces_{fAlloc_};


    struct ContactStatus
    {
        UniquePtr<ContactConstraint> existingContact = nullptr;

        bool hit = false;
    };

    typedef std::unordered_map<
        std::array<simu::Collider*, 2>,
        ContactStatus,
        std::hash<std::array<simu::Collider*, 2>>,
        std::equal_to<std::array<simu::Collider*, 2>>,
        ReboundTo<Alloc, std::pair<const std::array<simu::Collider*, 2>, ContactStatus>>>
        ContactList;

    ContactList contacts_{miscAlloc_};

    ContactList::iterator
    inContacts(const std::array<simu::Collider*, 2>& colliders)
    {
        auto asIs = contacts_.find(colliders);
        if (asIs != contacts_.end())
            return asIs;
        else
            return contacts_.find(
                std::array<simu::Collider*, 2>{colliders[1], colliders[0]}
            );
    }

    Settings settings_;

    ContactFactory makeContactConstraint_;
};


} // namespace simu
