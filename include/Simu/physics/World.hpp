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

#include <memory>
#include <functional>

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
struct hash<simu::Bodies>
{
    size_t operator()(const simu::Bodies& bodies) const
    {
        auto b = bodies.bodies();
        return ::details::hash_val(b[0], b[1]);
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

    template <class U>
    using UniquePtr = std::unique_ptr<U, typename Alloc::Deleter>;

    typedef typename Alloc::rebind<UniquePtr<Constraint>>::other ConstraintAlloc;

    typedef std::function<UniquePtr<ContactConstraint>(Bodies, ConstraintAlloc&)>
        ContactFactory;

    static ContactFactory defaultContactFactory;

    ////////////////////////////////////////////////////////////
    /// \brief Construct an empty world
    ///
    ////////////////////////////////////////////////////////////
    World(ContactFactory makeContact = defaultContactFactory);

    // clang-format off

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
    auto constraints() const { return makeView(constraints_, DoubleDereference{}); }
    // clang-format on


    ////////////////////////////////////////////////////////////
    /// \brief Construct a Body and puts it in the world
    ///
    ////////////////////////////////////////////////////////////
    template <std::derived_from<Body> T, class... Args>
    T* makeBody(Args&&... args)
    {
        auto body = makeObject<T>(bAlloc_, std::forward<Args>(args)...);

        BoundingBox bounds = boundsOf(body.get());
        T* b = static_cast<T*>(bodies_.emplace_back(std::move(body)).get());
        b->treeLocation_ = bodyTree_.emplace(bounds, b);
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

        for (Body* body : c->bodies().bodies())
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
            bodyTree_.forEachIn(f->domain().region, [](BodyTree::iterator it) {
                (*it)->wake();
            });

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

    // TODO: Contact conflicts needs testing
    // TODO: This is no longer needed since bodies knows their constraint
    //  constraint should be queried to know wether or not they prevent contact between two of their bodies.
    ////////////////////////////////////////////////////////////
    /// \brief Declares that contacts between 2 bodies should not be enforced
    ///
    /// If there already exist a contact between them, it is killed.
    /// As long as at least one conflict exists between the 2 bodies, they will
    ///     never collide.
    ///
    ////////////////////////////////////////////////////////////
    void declareContactConflict(const Bodies& bodies);

    ////////////////////////////////////////////////////////////
    /// \brief Removes a contact conflict between 2 bodies.
    ///
    /// \see declareContactConflict
    ////////////////////////////////////////////////////////////
    void removeContactConflict(const Bodies& bodies);

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


    template <std::invocable<Body*> F>
    void forEachIn(BoundingBox box, const F& func)
    {
        bodyTree_.forEachIn(box, [&](BodyTree::iterator it) { func(*it); });
    }

    template <std::invocable<Body*> F>
    void forEachAt(Vec2 point, const F& func)
    {
        bodyTree_.forEachAt(point, [&](BodyTree::iterator it) { func(*it); });
    }

private:


    ContactConstraint* makeContactConstraint(Bodies bodies);

    template <std::derived_from<PhysicsObject> T, class A, class... Args>
    UniquePtr<T> makeObject(A& alloc, Args&&... args)
    {
        auto obj = alloc.makeUnique<T>(std::forward<Args>(args)...);
        obj->setAllocator(alloc);
        obj->onConstruction(*this);
        return obj;
    }


    void applyForces(float dt);

    void detectContacts();

    struct Cleaner;
    void cleanup();

    // TODO: Move this code into island.
    //  for each island
    //      applyVelocityConstraints on active constraints
    //      integratePositions
    //      applyPositionConstraints on previously active constraints
    //
    // integrate positions of excluded bodies (structural)

    void applyVelocityConstraints(Island& island, float dt);
    void integrateBodies(float dt);
    void applyPositionConstraints(Island& island);

    void updateBodies(float dt);


    Alloc miscAlloc_{};


    typedef typename Alloc::rebind<UniquePtr<Body>>::other BodyAlloc;
    typedef std::list<UniquePtr<Body>, BodyAlloc>          BodyList;

    BodyAlloc bAlloc_{};
    BodyList  bodies_{bAlloc_};


    BodyTree bodyTree_{};


    typedef std::list<UniquePtr<Constraint>, ConstraintAlloc> ConstraintList;

    ConstraintAlloc cAlloc_{};
    ConstraintList  constraints_{cAlloc_};


    typedef typename Alloc::rebind<UniquePtr<ForceField>>::other ForceFieldAlloc;
    typedef std::list<UniquePtr<ForceField>, ForceFieldAlloc> ForceFieldList;

    ForceFieldAlloc fAlloc_{};
    ForceFieldList  forces_{fAlloc_};


    struct ContactStatus
    {
        Int32       nConflictingConstraints = 0;
        Constraint* existingContact         = nullptr;
    };

    typedef std::unordered_map<
        Bodies,
        ContactStatus,
        std::hash<Bodies>,
        std::equal_to<Bodies>,
        Alloc::rebind<std::pair<const Bodies, ContactStatus>>::other>
        ContactList;

    ContactList contacts_{miscAlloc_};

    ContactList::iterator inContacts(Bodies bodies)
    {
        auto b    = bodies.bodies();

        auto asIs = contacts_.find(bodies);
        return (asIs != contacts_.end()) ? asIs
                                         : contacts_.find(Bodies{b[1], b[0]});
    }

    Settings settings_;

    ContactFactory makeContactConstraint_;

    static constexpr float boundsScale = 1.2f;

    static BoundingBox boundsOf(const Body* body)
    {
        return BoundingBox::scaled(body->collider().boundingBox(), boundsScale);
    }
};


} // namespace simu
