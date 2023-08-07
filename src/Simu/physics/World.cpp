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

    for (Body& b : bodies())
        b.preStep();
    for (Constraint& c : constraints())
        c.preStep();
    for (auto& c : contacts_)
        if (c.second.existingContact != nullptr)
            static_cast<Constraint*>(c.second.existingContact.get())->preStep();
    for (ForceField& f : forceFields())
        f.preStep();


    if (dt > 0.f)
    {
        // TODO: While sleeping is not fully implemented,
        //  it currently requires that all islands were computed
        //  and all Bodies in an island have been waken if one is not sleeping.
        //
        // Instead forces can be applied and stored in the Body.
        //  if an island is sleeping, all its bodies' forces and velocities are set to 0
        applyForces(dt);

        solveIslands(bodies(), cAlloc_, settings(), dt);

        updateBodies(dt);
    }

    for (Body& b : bodies())
        b.postStep();
    for (Constraint& c : constraints())
        c.postStep();
    for (auto& c : contacts_)
        if (c.second.existingContact != nullptr)
            c.second.existingContact->postStep();
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
    = [](Bodies b, const typename World::ContactAlloc& alloc) {
          return makeUnique<ContactConstraint>(alloc, b);
      };

UniquePtr<ContactConstraint> World::makeContactConstraint(Bodies bodies)
{
    auto c = makeContactConstraint_(bodies, cAlloc_);
    c->setAllocator(cAlloc_);
    c->onConstruction(*this);

    for (Body* body : c->bodies())
    {
        body->contacts_.emplace_back(c.get());
        body->wake();
    }

    return c;
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
        ContactConstraint* contact = it->second.existingContact.get();
        if (contact != nullptr && contact->isDead())
        {
            for (Body* b : contact->bodies())
                std::erase(b->contacts_, contact);

            it = contacts_.erase(it);
        }
        else
        {
            ++it;
        }
    }


    Cleaner cleaner{};

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

    for (auto b : deadBodies)
        bodyTree_.erase((*b)->treeLocation_);

    cleaner.onDestruction(*this, deadConstraints);
    cleaner.onDestruction(*this, deadForces);
    cleaner.onDestruction(*this, deadBodies);

    cleaner.erase(constraints_, deadConstraints);
    cleaner.erase(forces_, deadForces);
    cleaner.erase(bodies_, deadBodies);
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

    typedef typename BodyTree::iterator           BodyIt;
    std::vector<BodyIt, ReboundTo<Alloc, BodyIt>> toUpdate{miscAlloc_};

    for (const auto& b : bodies_)
    {
        auto treeIt = b->treeLocation_;
        if (!b->isAsleep()
            && !treeIt.bounds().contains(b->collider().boundingBox()))
            toUpdate.emplace_back(treeIt);
    }


    // update bounds
    for (auto it : toUpdate)
        bodyTree_.update(it, boundsOf(*it));


    // check for new contacts
    auto registerContact = [=, this](BodyIt first, BodyIt second) {
        if (first == second)
            return;

        Bodies bodies{*first, *second};
        auto   contact = inContacts(bodies);

        if (contact == contacts_.end())
            contacts_[bodies] = ContactStatus{0, makeContactConstraint(bodies)};
    };

    bodyTree_.forEachOverlapping(
        makeView(toUpdate.data(), toUpdate.data() + toUpdate.size()),
        registerContact
    );


    // kill outdated contacts
    for (auto it : toUpdate)
    {
        Body* b = *it;

        for (ContactConstraint* c : b->contacts())
        {
            if (!c->isDead())
            {
                auto bodies = c->bodies();
                if (!boundsOf(bodies[0]).overlaps(boundsOf(bodies[1])))
                    c->kill();
            }
        }
    }
}


} // namespace simu
