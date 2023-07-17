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

#include "Simu/physics/World.hpp"

#include "Simu/physics/Island.hpp"

namespace simu
{

World::World(ContactFactory makeContact) : makeContactConstraint_{makeContact}
{
}

void World::step(float dt)
{
    // TODO:
    // - cleanup
    // - detect contacts
    //
    // - create islands
    // - for each island
    //      - if any body is awake, wake everything
    //
    // - apply forcefields on awake bodies
    //
    // - for each island
    //      - apply constraints until nIter or epsilonLambda
    //
    // - integrate bodies
    //
    // - for each island
    //      - correct positions until nIter or epsilon
    //
    // - update body bounds on awake bodies.
    //


    cleanup();
    detectContacts();

    // TODO: with some additionnal management code,
    //  islands can be made to persist between steps, possibly reducing computation.
    Islands islands(bodies());
    for (Island& island : islands.islands())
        if (island.isAwake())
            for (Body* body : island.bodies())
                body->wake();

    applyForces(dt);

    for (Island& island : islands.islands())
        if (!settings().enableSleeping || island.isAwake())
            applyVelocityConstraints(island, dt);

    integrateBodies(dt);

    for (Island& island : islands.islands())
        if (!settings().enableSleeping || island.isAwake())
            applyPositionConstraints(island);

    updateBodies(dt);
}

void World::declareContactConflict(const Bodies<2>& bodies)
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


void World::removeContactConflict(const Bodies<2>& bodies)
{
    auto contact = inContacts(bodies);
    if (contact != contacts_.end())
    {
        contact->second.nConflictingConstraints
            = std::max(0, contact->second.nConflictingConstraints - 1);

        if ((contact->second.nConflictingConstraints == 0)
            && (contact->second.existingContact == nullptr))
            contacts_.erase(contact);
    }
}


void World::applyForces(float dt)
{
    struct ApplyForce
    {
        void operator()(BodyTree::iterator body) const { (*this)(**body); }

        void operator()(Body& body) const
        {
            if (!body.isAsleep() && !body.isStructural())
                force.apply(body, dt);
        }

        ForceField& force;
        float       dt;
    };


    for (ForceField& force : forceFields())
    {
        ApplyForce applyForce{force, dt};

        if (force.domain().type == ForceField::DomainType::global)
        {
            for (Body& body : bodies())
                applyForce(body);
        }
        else
        {
            bodies_.forEachIn(force.domain().region, applyForce);
        }
    }
}


void World::detectContacts()
{
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        auto registerContact = [=, this](BodyTree::iterator other) {
            if (it->get() != other->get())
            {
                Bodies<2> bodies{it->get(), other->get()};
                auto      contact = inContacts(bodies);
                if (contact == contacts_.end())
                {
                    auto constraint = makeContactConstraint(bodies);

                    contacts_[bodies] = ContactStatus{0, constraint};
                }
            }
        };

        bodies_.forEachIn(it.bounds(), registerContact);
    }
}


struct World::Cleaner
{
    typedef std::unordered_map<Bodies<2>, World::ContactStatus>::iterator ContactIter;

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
    void onDestruction(World& world, const DeadObjects& deadObjects)
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
    void notifyDestruction(Iter it, World& world)
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
World::Cleaner::access<typename World::Cleaner::ContactIter>(ContactIter it)
{
    return it->second.existingContact;
}

template <>
bool World::Cleaner::isDead<typename World::Cleaner::ContactIter>(ContactIter it)
{
    return access(it) != nullptr && access(it)->isDead();
}


void World::cleanup()
{
    Cleaner cleaner{};

    auto deadContacts    = cleaner.deadObjects(contacts_);
    auto deadConstraints = cleaner.deadObjects(constraints_);
    auto deadForces      = cleaner.deadObjects(forces_);
    auto deadBodies      = cleaner.deadObjects(bodies_);

    for (auto c : deadConstraints)
    {
        for (Body* body : (*c)->bodies())
        {
            body->constraints_.erase(std::find(
                body->constraints_.begin(),
                body->constraints_.end(),
                c->get()
            ));

            body->wake();
        }
    }

    cleaner.onDestruction(*this, deadContacts);
    cleaner.onDestruction(*this, deadConstraints);
    cleaner.onDestruction(*this, deadForces);
    cleaner.onDestruction(*this, deadBodies);

    cleaner.erase(contacts_, deadContacts);
    cleaner.erase(constraints_, deadConstraints);
    cleaner.erase(forces_, deadForces);
    cleaner.erase(bodies_, deadBodies);
}

void World::applyVelocityConstraints(Island& island, float dt)
{
    std::vector<ConstraintPtr> actives{};
    for (Constraint* constraint : island.constraints())
    {
        if (constraint->isActive())
            actives.emplace_back(constraint);
    }

    for (Constraint* constraint : actives)
        constraint->initSolve(dt);

    for (Uint32 iter = 0; iter < settings_.nVelocityIterations; ++iter)
    {
        for (Constraint* constraint : actives)
            constraint->solveVelocities(dt);
    }
}

void World::integrateBodies(float dt)
{
    for (Body& body : bodies())
        if (!body.isAsleep())
            body.step(dt);
}

void World::applyPositionConstraints(Island& island)
{
    for (Uint32 iter = 0; iter < settings_.nPositionIterations; ++iter)
    {
        for (auto constraint : island.constraints())
            if (constraint->isActive())
                constraint->solvePositions();
    }
}

void World::updateBodies(float dt)
{
    if (settings_.enableSleeping)
    {
        for (Body& body : bodies())
        {
            body.updateTimeImmobile(
                dt,
                settings_.velocitySleepThreshold,
                settings_.angularVelocitySleepThreshold
            );

            if (body.canSleep(settings_.inactivitySleepDelay))
                body.sleep();
        }
    }

    std::vector<BodyTree::iterator> toUpdate{};
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        if (!(*it)->isAsleep())
            toUpdate.emplace_back(it);
    }

    // TODO: batch updates of RTree ...
    // TODO: Modifying the tree while iterating is undefined.
    for (auto it : toUpdate)
        bodies_.update(it, (*it)->collider().boundingBox());
}


} // namespace simu
