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
        contacts_[bodies] = ContactStatus{1, nullptr};
    else
        ++contact->second.nConflictingConstraints;
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


void PhysicsWorld::cleanup()
{
    for (auto c = contacts_.begin(); c != contacts_.end();)
    {
        ConstraintPtr contactConstraint = c->second.existingContact;
        if (contactConstraint != nullptr && contactConstraint->isDead())
            c = contacts_.erase(c);
        else
            ++c;
    }

    for (auto c = constraints_.begin(); c != constraints_.end();)
    {
        if ((*c)->isDead())
        {
            (*c)->onDestruction(*this);
            c = constraints_.erase(c);
        }
        else
            c++;
    }

    for (auto force = forces_.begin(); force != forces_.end();)
    {
        if ((*force)->isDead())
        {
            (*force)->onDestruction(*this);
            force = forces_.erase(force);
        }
        else
            force++;
    }

    // modification during iteration is undefined for RTree.
    std::vector<BodyTree::iterator> deadBodies{};
    for (auto body = bodies_.begin(); body != bodies_.end(); ++body)
        if ((*body)->isDead())
            deadBodies.emplace_back(body);

    for (auto body : deadBodies)
    {
        (*body)->onDestruction(*this);
        bodies_.erase(body);
    }
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
            constraint->solve(dt);
    }

    for (auto& constraint : actives)
        constraint->commit();
}


void PhysicsWorld::updateBodies(float dt)
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


} // namespace simu
