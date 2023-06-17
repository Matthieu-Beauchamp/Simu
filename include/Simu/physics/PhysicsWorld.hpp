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
#include "Simu/physics/RTree.hpp"
#include "Simu/physics/PhysicsBody.hpp"

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

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
        return hash_val(p.first, p.second);
    }
};

} // namespace details

namespace std
{
template <>
struct hash<pair<simu::PhysicsBody*, simu::PhysicsBody*>>
{
    size_t operator()(pair<simu::PhysicsBody*, simu::PhysicsBody*> bodies) const
    {
        return ::details::pair_hash{}(bodies);
    }
};

} // namespace std

#include <list>
#include <unordered_map>

#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/ForceField.hpp"
#include "Simu/physics/Range.hpp"

namespace simu
{

class PhysicsWorld
{
    typedef RTree<std::unique_ptr<PhysicsBody>>   BodyTree;
    typedef std::pair<PhysicsBody*, PhysicsBody*> BodyPair;

public:

    typedef PhysicsBody*       BodyPtr;
    typedef const PhysicsBody* ConstBodyPtr;

    typedef Constraint*       ConstraintPtr;
    typedef const Constraint* ConstConstraintPtr;

    typedef ForceField*       ForceFieldPtr;
    typedef const ForceField* ConstForceFieldPtr;

    PhysicsWorld() = default;

    template <std::derived_from<PhysicsBody> T = PhysicsBody, class... Args>
    BodyPtr makeBody(Args&&... args)
    {
        std::unique_ptr<T> body
            = std::make_unique<T>(std::forward<Args>(args)...);
        BoundingBox bounds = body->collider().boundingBox();
        return bodies_.emplace(bounds, std::move(body))->get();
    }

    template <std::derived_from<Constraint> T, class... Args>
    T* makeConstraint(Args&&... args)
    {
        return static_cast<T*>(
            constraints_
                .emplace_back(std::make_unique<T>(std::forward<Args>(args)...))
                .get()
        );
    }

    template <std::derived_from<ForceField> T, class... Args>
    T* makeForceField(Args&&... args)
    {
        return static_cast<T*>(
            forces_
                .emplace_back(std::make_unique<T>(std::forward<Args>(args)...))
                .get()
        );
    }

    void step(float dt)
    {
        cleanup();
        detectContacts();
        applyForces(dt);
        applyConstraints(dt);
        updateBodies(dt);
    }


    auto bodies()
    {
        return makeRange(bodies_.begin(), bodies_.end(), BypassSmartPointer{});
    }
    auto bodies() const
    {
        return makeRange(bodies_.begin(), bodies_.end(), BypassSmartPointer{});
    }

private:

    auto forceFields()
    {
        return makeRange(forces_.begin(), forces_.end(), BypassSmartPointer{});
    }
    auto forceFields() const
    {
        return makeRange(forces_.begin(), forces_.end(), BypassSmartPointer{});
    }

    auto constraints()
    {
        return makeRange(
            constraints_.begin(),
            constraints_.end(),
            BypassSmartPointer{}
        );
    }
    auto constraints() const
    {
        return makeRange(
            constraints_.begin(),
            constraints_.end(),
            BypassSmartPointer{}
        );
    }


    void applyForces(float dt)
    {
        for (ForceField& force : forceFields())
        {
            auto applyForce = [=, &force](BodyTree::iterator body) {
                force.apply(**body, dt);
            };

            if (force.domain().type == ForceField::DomainType::global)
            {
                for (PhysicsBody& body : bodies())
                    force.apply(body, dt);
            }
            else
            {
                bodies_.forEachIn(force.domain().region, applyForce);
            }
        }
    }

    void detectContacts()
    {
        for (auto it = bodies_.begin(); it != bodies_.end(); ++it)
        {
            auto registerContact = [=](BodyTree::iterator other) {
                if (it->get() != other->get())
                {
                    BodyPair bodies{it->get(), other->get()};
                    auto     contact = inContacts(bodies);
                    if (contact == contacts_.end())
                    {
                        auto constraint = makeConstraint<ContactConstraint>(
                            Bodies<2>{bodies.first, bodies.second}
                        );

                        contacts_[bodies] = ContactStatus{0, constraint};
                    }
                }
            };

            bodies_.forEachIn(it.bounds(), registerContact);
        }
    }

    void cleanup()
    {
        for (auto& c : contacts_)
        {
            ConstraintPtr& contactConstraint = c.second.existingContact;
            if (contactConstraint != nullptr && contactConstraint->isDead())
                contactConstraint = nullptr;
        }

        auto constraint = constraints_.begin();
        while (constraint != constraints_.end())
        {
            if ((*constraint)->isDead())
                constraint = constraints_.erase(constraint);
            else
                constraint++;
        }

        // TODO: Removing constraints must update the contacts.conflictingConstraint 
        //      count, those that hit 0 can be removed.

        auto force = forces_.begin();
        while (force != forces_.end())
        {
            if ((*force)->isDead())
                force = forces_.erase(force);
            else
                force++;
        }

        // modification during iteration is undefined for RTree.
        std::vector<BodyTree::iterator> deadBodies{};
        for (auto body = bodies_.begin(); body != bodies_.end(); ++body)
            if ((*body)->isDead())
                deadBodies.emplace_back(body);

        for (auto body : deadBodies)
            bodies_.erase(body);
    }

    void applyConstraints(float dt)
    {
        std::vector<ConstraintPtr> actives{};
        for (auto& constraint : constraints_)
        {
            if (constraint->isActive())
                actives.emplace_back(constraint.get());
        }

        for (auto constraint : actives)
            constraint->initSolve(dt);

        // TODO: settings.maxIter
        for (Uint32 iter = 0; iter < 10; ++iter)
        {
            for (auto constraint : actives)
                constraint->solve(dt);
        }

        for (auto& constraint : actives)
            constraint->commit();
    }

    void updateBodies(float dt)
    {
        std::vector<BodyTree::iterator> toUpdate{};
        for (auto it = bodies_.begin(); it != bodies_.end(); ++it)
        {
            (*it)->step(dt);
            toUpdate.emplace_back(it);
        }

        // TODO: batch updates of RTree ...
        // TODO: Modifying the tree while iterating is undefined.
        for (auto it : toUpdate)
            bodies_.update(it, (*it)->collider().boundingBox());
    }

    BodyTree                               bodies_{};
    std::list<std::unique_ptr<Constraint>> constraints_{};
    std::list<std::unique_ptr<ForceField>> forces_{};


    struct ContactStatus
    {
        Uint32        nConflictingConstraints = 0;
        ConstraintPtr existingContact         = nullptr;
    };

    std::unordered_map<std::pair<BodyPtr, BodyPtr>, ContactStatus> contacts_;
    std::unordered_map<std::pair<BodyPtr, BodyPtr>, ContactStatus>::iterator
    inContacts(std::pair<BodyPtr, BodyPtr> bodies)
    {
        auto asIs = contacts_.find(bodies);
        return (asIs != contacts_.end())
                   ? asIs
                   : contacts_.find(
                       std::pair<BodyPtr, BodyPtr>{bodies.second, bodies.first}
                   );
    }

    // TODO:
    ConstraintSolver solver_;
    // solver settings
};


} // namespace simu
