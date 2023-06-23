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

#include "Simu/physics/PhysicsWorld.hpp"
#include "Simu/physics/Constraint.hpp"

namespace simu
{

void PhysicsWorld::step(float dt)
{
    cleanup();
    detectContacts();
    applyForces(dt);
    applyConstraints(dt);
    updateBodies(dt);
}

void PhysicsWorld::declareContactConflict(const Bodies<2>& bodies)
{
    auto contact = inContacts(bodies);
    if (contact == contacts_.end())
    {
        contacts_[bodies] = ContactStatus{1, nullptr};
    }
    else
    {
        ++contact->second.nConflictingConstraints;
        if (contact->second.existingContact != nullptr)
            contact->second.existingContact->kill();
    }
}


void PhysicsWorld::removeContactConflict(const Bodies<2>& bodies)
{
    auto contact = inContacts(bodies);
    if (contact != contacts_.end())
    {
        if ((--contact->second.nConflictingConstraints <= 0)
            && (contact->second.existingContact == nullptr))
            contacts_.erase(contact);
    }
}


void PhysicsWorld::applyForces(float dt)
{
    for (ForceField& force : forceFields())
    {
        auto applyForce
            = [=, &force](BodyTree::iterator body) { force.apply(**body, dt); };

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


void PhysicsWorld::detectContacts()
{
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        auto registerContact = [=](BodyTree::iterator other) {
            if (it->get() != other->get())
            {
                Bodies<2> bodies{it->get(), other->get()};
                auto      contact = inContacts(bodies);
                if (contact == contacts_.end())
                {
                    auto constraint = makeConstraint<ContactConstraint>(bodies);

                    contacts_[bodies] = ContactStatus{0, constraint};
                }
            }
        };

        bodies_.forEachIn(it.bounds(), registerContact);
    }
}


struct PhysicsWorld::Cleaner
{
    typedef std::unordered_map<Bodies<2>, PhysicsWorld::ContactStatus>::iterator
        ContactIter;

    template <class Container>
    auto deadObjects(Container& container)
    {
        std::vector<typename Container::iterator> dead;
        for (auto it = container.begin(); it != container.end(); ++it)
            if (isDead(it))
                dead.emplace_back(it);

        return dead;
    }

    template <class DeadObjects>
    void onDestruction(PhysicsWorld& world, const DeadObjects& deadObjects)
    {
        for (auto it : deadObjects)
            notifyDestruction(it, world);
    }

    template <class Container, class DeadObjects>
    void erase(Container& container, const DeadObjects& deadObjects)
    {
        for (auto it : deadObjects)
            container.erase(it);
    }


private:

    template <class Iter>
    bool isDead(Iter it)
    {
        return access(it)->isDead();
    }

    template <class Iter>
    void notifyDestruction(Iter it, PhysicsWorld& world)
    {
        return access(it)->onDestruction(world);
    }

    template <class Iter>
    PhysicsObject* access(Iter it)
    {
        return it->get();
    }
};


template <>
PhysicsObject*
PhysicsWorld::Cleaner::access<typename PhysicsWorld::Cleaner::ContactIter>(
    ContactIter it
)
{
    return it->second.existingContact;
}

template <>
bool PhysicsWorld::Cleaner::isDead<typename PhysicsWorld::Cleaner::ContactIter>(
    ContactIter it
)
{
    return access(it) != nullptr && access(it)->isDead();
}


void PhysicsWorld::cleanup()
{
    Cleaner cleaner{};

    auto deadContacts    = cleaner.deadObjects(contacts_);
    auto deadConstraints = cleaner.deadObjects(constraints_);
    auto deadForces      = cleaner.deadObjects(forces_);
    auto deadBodies      = cleaner.deadObjects(bodies_);

    cleaner.onDestruction(*this, deadContacts);
    cleaner.onDestruction(*this, deadConstraints);
    cleaner.onDestruction(*this, deadForces);
    cleaner.onDestruction(*this, deadBodies);

    cleaner.erase(contacts_, deadContacts);
    cleaner.erase(constraints_, deadConstraints);
    cleaner.erase(forces_, deadForces);
    cleaner.erase(bodies_, deadBodies);
}

void PhysicsWorld::applyConstraints(float dt)
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
            constraint->solveVelocities(dt);
    }

    for (PhysicsBody& body : bodies())
        body.step(dt);

    // TODO: settings.maxPosIter
    for (Uint32 iter = 0; iter < 2; ++iter)
    {
        for (auto constraint : actives)
            constraint->solvePositions();
    }
}


void PhysicsWorld::updateBodies(float dt)
{
    std::vector<BodyTree::iterator> toUpdate{};
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        toUpdate.emplace_back(it);
    }

    // TODO: batch updates of RTree ...
    // TODO: Modifying the tree while iterating is undefined.
    for (auto it : toUpdate)
        bodies_.update(it, (*it)->collider().boundingBox());
}


} // namespace simu
