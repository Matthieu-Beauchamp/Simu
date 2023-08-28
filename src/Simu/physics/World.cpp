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

#include "Simu/physics/Body.hpp"
#include "Simu/physics/ForceField.hpp"
#include "Simu/physics/Island.hpp"

namespace simu
{

World::World(ContactFactory makeContact) : makeContactConstraint_{makeContact}
{
}

void World::clear()
{
    bodies_.clear();
    forces_.clear();
    contacts_.clear();
    contactsTable_.clear();
    constraints_.clear();
    colliderTree_.clear();
}

void World::setContactFactory(ContactFactory makeContact)
{
    contacts_.clear();
    makeContactConstraint_ = makeContact;
}

Body* World::makeBody(const BodyDescriptor& descr)
{
    return makeBody<Body>(descr);
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
    for (ContactConstraint& c : contacts())
        c.preStep();
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

        {
            SIMU_PROFILE_ENTRY(profiler_.narrowPhaseCollision);

            for (ContactConstraint& c : contacts())
                c.updateContacts();
        }

        solveIslands(bodies(), cAlloc_, settings(), dt, profiler());

        updateBodies(dt);
    }

    for (Body& b : bodies())
        b.postStep();
    for (Constraint& c : constraints())
        c.postStep();
    for (ContactConstraint& c : contacts())
        c.postStep();
    for (ForceField& f : forceFields())
        f.postStep();
}

void World::updateSettings(const Settings& settings)
{
    if (!settings.enableSleeping && settings_.enableSleeping)
        for (Body& body : bodies())
            body.wake();

    settings_ = settings;
}

typename World::ContactFactory World::defaultContactFactory =
    [](Collider&                    first,
       Collider&                    second,
       CollisionCallback            callback,
       typename World::ContactList& contacts) {
        return contacts.emplace_back<ContactConstraint>(first, second, callback);
    };

typename World::ContactIterator
World::makeContactConstraint(Collider& first, Collider& second)
{
    auto c = makeContactConstraint_(
        first, second, shapeCollider_.getCallback(first.shape(), second.shape()), contacts_
    );

    c->onConstruction(*this);

    for (Body* body : c->bodies())
    {
        body->contacts_.emplace_back(std::addressof(*c));
        body->wake();
    }

    return c;
}

void World::applyForces(float dt)
{
    struct ApplyForce
    {
        void operator()(ColliderTree::iterator collider) const
        {
            (*this)(**collider);
        }

        void operator()(Collider& collider) const
        {
            Body& body = *collider.body();
            if (!body.isAsleep() && !body.isStructural())
                force.apply(collider, dt);
        }

        ForceField& force;
        float       dt;
    };


    for (ForceField& force : forceFields())
    {
        ApplyForce applyForce{force, dt};

        if (force.domain().type == ForceField::DomainType::global)
        {
            for (Collider* collider : colliderTree_)
                applyForce(*collider);
        }
        else
        {
            colliderTree_.forEachIn(force.domain().region, applyForce);
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
        return &*it;
    }
};


void World::addCollider(Collider* collider)
{
    collider->treeLocation_ = colliderTree_.emplace(
        collider->shape().boundingBox(), collider
    );
}

void World::removeCollider(Collider* collider)
{
    colliderTree_.erase(collider->treeLocation_);

    Body* b = collider->body();
    for (ContactConstraint* c : b->contacts_)
    {
        if (c->colliders()[0] == collider || c->colliders()[1] == collider)
            c->kill();
    }
}

void World::cleanup()
{
    for (auto it = contactsTable_.begin(); it != contactsTable_.end();)
    {
        auto contact = it->second.existingContact;
        if (contact->isDead())
        {
            for (Body* b : contact->bodies())
            {
                std::erase(b->contacts_, std::addressof(*contact));
                b->wake();
            }

            contacts_.erase(contact);
            it = contactsTable_.erase(it);
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
        for (Body* body : c->bodies())
        {
            std::erase(body->constraints_, &*c);
            body->wake();
        }
    }

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
                dt, settings_.velocitySleepThreshold, settings_.angularVelocitySleepThreshold
            );

            if (body.canSleep(settings_.inactivitySleepDelay))
                body.sleep();
        }
    }

    typedef typename ColliderTree::iterator ColliderIt;

    // check for new contacts
    auto registerContact = [=, this](ColliderIt first, ColliderIt second) {
        std::array<simu::Collider*, 2> colliders{*first, *second};

        const Body* b0 = colliders[0]->body();
        const Body* b1 = colliders[1]->body();
        if (b0 == b1)
            return;

        for (Constraint* c : b0->constraints())
        {
            if (c->disablesContacts())
            {
                const Bodies& bodies = c->bodies();
                if (bodies[0] == b1 || bodies[1] == b1)
                    return;
            }
        }

        auto contact = inContacts(colliders);

        if (contact == contactsTable_.end())
            contactsTable_[colliders] = ContactStatus{
                makeContactConstraint(**first, **second), true};
        else
            contact->second.hit = true;
    };

    {
        SIMU_PROFILE_ENTRY(profiler_.treeUpdateAndCollision);

        colliderTree_.updateAndCollide(
            [](ColliderIt it) { return (*it)->shape().boundingBox(); }, registerContact
        );
    }

    SIMU_PROFILE_TREE_HEIGHT(profiler_, colliderTree_);

    for (auto& contact : contactsTable_)
    {
        ContactStatus& cs = contact.second;
        if (!cs.hit)
        {
            cs.existingContact->kill();
        }

        cs.hit = false;
    }
}

typename World::ContactTable::iterator
World::inContacts(const std::array<simu::Collider*, 2>& colliders)
{
    auto asIs = contactsTable_.find(colliders);
    if (asIs != contactsTable_.end())
        return asIs;
    else
        return contactsTable_.find(std::array<simu::Collider*, 2>{
            colliders[1], colliders[0]});
}

} // namespace simu
