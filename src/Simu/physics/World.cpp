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


    for (Body& b : bodies())
        b.preStep();
    for (Constraint& c : constraints())
        c.preStep();
    for (ForceField& f : forceFields())
        f.preStep();


    if (dt > 0.f)
    {
        Islands islands(bodies(), miscAlloc_);
        
        islands.wakeOrSleep();
        applyForces(dt);
        islands.solve(settings(), dt);
        
        updateBodies(dt);
    }

    for (Body& b : bodies())
        b.postStep();
    for (Constraint& c : constraints())
        c.postStep();
    for (ForceField& f : forceFields())
        f.postStep();
}

void World::declareContactConflict(const Bodies& bodies)
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


void World::removeContactConflict(const Bodies& bodies)
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

typename World::ContactFactory World::defaultContactFactory
    = [](Bodies b, typename World::ConstraintAlloc& alloc) {
          return alloc.makeUnique<ContactConstraint>(b);
      };

ContactConstraint* World::makeContactConstraint(Bodies bodies)
{
    auto c = makeContactConstraint_(bodies, cAlloc_);
    c->setAllocator(cAlloc_);
    c->onConstruction(*this);

    for (Body* body : c->bodies().bodies())
    {
        body->constraints_.emplace_back(c.get());
        body->wake();
    }

    return static_cast<ContactConstraint*>(
        constraints_.emplace_back(std::move(c)).get()
    );
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
            bodyTree_.forEachIn(force.domain().region, applyForce);
        }
    }
}

void World::detectContacts()
{
    for (auto it = bodyTree_.begin(); it != bodyTree_.end(); ++it)
    {
        auto registerContact = [=, this](BodyTree::iterator other) {
            Bodies bodies{*it, *other};
            auto      contact = inContacts(bodies);

            if (contact == contacts_.end())
                contacts_[bodies]
                    = ContactStatus{0, makeContactConstraint(bodies)};
        };

        bodyTree_.forEachOverlapping(it, registerContact);
    }
}


struct World::Cleaner
{
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


void World::cleanup()
{
    for (auto it = contacts_.begin(); it != contacts_.end();)
    {
        auto& contact = *it;
        if (contact.second.existingContact != nullptr)
        {
            if (!boundsOf(contact.first.bodies()[0]).overlaps(boundsOf(contact.first.bodies()[0])))
            {
                contact.second.existingContact->kill();
                it = contacts_.erase(it);
                continue;
            }
        }
        ++it;
    }

    Cleaner cleaner{};

    auto deadConstraints = cleaner.deadObjects(constraints_);
    auto deadForces      = cleaner.deadObjects(forces_);
    auto deadBodies      = cleaner.deadObjects(bodies_);

    for (auto c : deadConstraints)
    {
        for (Body* body : (*c)->bodies().bodies())
        {
            body->constraints_.erase(std::find(
                body->constraints_.begin(),
                body->constraints_.end(),
                c->get()
            ));

            body->wake();
        }
    }

    for (auto b : deadBodies)
        bodyTree_.erase((*b)->treeLocation_);

    cleaner.onDestruction(*this, deadConstraints);
    cleaner.onDestruction(*this, deadForces);
    cleaner.onDestruction(*this, deadBodies);

    cleaner.erase(constraints_, deadConstraints);
    cleaner.erase(forces_, deadForces);
    cleaner.erase(bodies_, deadBodies);
}

void World::applyVelocityConstraints(Island& island, float dt)
{
    std::vector<Constraint*, typename Alloc::rebind<Constraint*>::other> actives{
        miscAlloc_};

    for (Constraint* constraint : island.constraints())
    {
        if (constraint->isActive())
            actives.emplace_back(constraint);
    }

    for (Constraint* constraint : actives)
        constraint->initSolve();

    for (Constraint* constraint : actives)
        constraint->warmstart(dt);


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

    for (const auto& b : bodies_)
    {
        auto treeIt = b->treeLocation_;
        if (!b->isAsleep()
            && !treeIt.bounds().contains(b->collider().boundingBox()))
            bodyTree_.update(treeIt, boundsOf(b.get()));
    }
}


} // namespace simu
