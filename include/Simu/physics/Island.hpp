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

#include <set>

#include "Simu/physics/Body.hpp"
#include "Simu/physics/Constraint.hpp"


namespace simu
{

class Island
{
public:

    typedef typename PhysicsObject::PhysicsAlloc Alloc;

    Island(const Alloc& alloc)
        : bodies_{alloc},
          constraints_{alloc},
          contacts_{alloc},
          positions_{alloc},
          velocities_{alloc}
    {
    }

    Island(const Island&) = delete;
    Island(Island&&)      = delete;

    void init(Body* root)
    {
        addBody(root);

        std::size_t current = 0;
        while (current != bodies_.size())
        {
            root = bodies_[current++];

            if (!root->interactsAsStructural())
            {
                for (Constraint* constraint : root->constraints())
                {
                    addConstraint(constraint, constraints_);
                }

                for (ContactConstraint* contact : root->contacts())
                {
                    addConstraint(contact, contacts_);
                }
            }
        }
    }

    void clear()
    {
        bodies_.clear();
        constraints_.clear();
        contacts_.clear();
        positions_.clear();
        velocities_.clear();
    }

    bool hasConstraints()
    {
        return !(constraints_.empty() && contacts_.empty());
    }

    bool isAwake() const { return isAwake_; }
    auto bodies() { return makeView(bodies_); }
    auto constraints() { return makeView(constraints_); }
    auto contacts() { return makeView(contacts_); }

    void refreshProxies()
    {
        // must do this after forces are applied
        for (Body* b : bodies_)
        {
            velocities_[b->proxyIndex] = b->velocity_;

            // in case of weird force field.
            positions_[b->proxyIndex] = b->position_;
        }

        for (Constraint* c : constraints_)
            c->bodies().startSolve(
                positions_.data(),
                velocities_.data()
            ); // refresh proxy pointers

        for (ContactConstraint* c : contacts_)
            c->bodies().startSolve(
                positions_.data(),
                velocities_.data()
            ); // refresh proxy pointers
    }

    void applyVelocityConstraints(Uint32 nIter, float dt)
    {
        for (Constraint* constraint : constraints_)
            constraint->initSolve(constraint->bodies().getProxies());
        for (ContactConstraint* constraint : contacts_)
            constraint->initSolve(constraint->bodies().getProxies());

        for (Constraint* constraint : constraints_)
            constraint->warmstart(constraint->bodies().getProxies(), dt);
        for (ContactConstraint* constraint : contacts_)
            constraint->warmstart(constraint->bodies().getProxies(), dt);

        for (Uint32 iter = 0; iter < nIter; ++iter)
        {
            for (Constraint* constraint : constraints_)
                constraint->solveVelocities(constraint->bodies().getProxies(), dt);
            for (ContactConstraint* constraint : contacts_)
                constraint->solveVelocities(constraint->bodies().getProxies(), dt);
        }
    }

    void integrateBodies(float dt)
    {
        for (std::size_t i = 0; i < positions_.size(); ++i)
            positions_[i].advance(
                velocities_[i].linear() * dt,
                velocities_[i].angular() * dt
            );
    }

    void applyPositionConstraints(Uint32 nIter)
    {
        for (Uint32 iter = 0; iter < nIter; ++iter)
        {
            for (Constraint* constraint : constraints_)
                constraint->solvePositions(constraint->bodies().getProxies());
            for (ContactConstraint* constraint : contacts_)
                constraint->solvePositions(constraint->bodies().getProxies());
        }
    }

    void endSolve()
    {
        for (Body* b : bodies_)
        {
            b->velocity_ = velocities_[b->proxyIndex];
            b->position_ = positions_[b->proxyIndex];
            b->update();
            b->proxyIndex = Body::NO_INDEX;
        }

        for (Constraint* c : constraints_)
            c->bodies().endSolve();
        for (ContactConstraint* c : contacts_)
            c->bodies().endSolve();
    }

private:

    bool addBody(Body* body)
    {
        bool isNew = body->proxyIndex == Body::NO_INDEX;

        if (isNew)
        {
            bodies_.emplace_back(body);
            isAwake_ = isAwake_ || !body->isAsleep();

            SIMU_ASSERT(
                positions_.size() < std::numeric_limits<Int32>::max(),
                "Too many bodies in island"
            );

            body->proxyIndex = static_cast<Int32>(positions_.size());
            positions_.emplace_back(body->position_);
            velocities_.emplace_back(body->velocity_);
        }

        return isNew;
    }

    template <class T, class C>
    bool addConstraint(T* constraint, C& container)
    {
        if (constraint->bodies().hasProxies())
            return false;

        container.emplace_back(constraint);

        auto& b      = constraint->bodies();
        Int32 index1 = b[0]->proxyIndex;
        Int32 index2 = b[1]->proxyIndex;
        bool  added1 = addBody(b[0]);
        bool  added2 = addBody(b[1]);

        constraint->bodies().startSolve(positions_.data(), velocities_.data());

        if (!constraint->isActive(constraint->bodies().getProxies()))
        {
            if (added2)
            {
                b[1]->proxyIndex = index2;
                bodies_.pop_back();
                positions_.pop_back();
                velocities_.pop_back();
            }

            if (added1)
            {
                b[0]->proxyIndex = index1;
                bodies_.pop_back();
                positions_.pop_back();
                velocities_.pop_back();
            }

            constraint->bodies().endSolve();
            container.pop_back();
            return false;
        }

        return true;
    }

    std::vector<Body*, ReboundTo<Alloc, Body*>>             bodies_;
    std::vector<Constraint*, ReboundTo<Alloc, Constraint*>> constraints_;
    std::vector<ContactConstraint*, ReboundTo<Alloc, ContactConstraint*>> contacts_;

    std::vector<Position, ReboundTo<Alloc, Position>> positions_;
    std::vector<Velocity, ReboundTo<Alloc, Velocity>> velocities_;

    bool isAwake_ = false;
};

template <class T>
concept BodyRange = std::ranges::range<T> && requires(T t) {
    // clang-format off
    { *t.begin() } -> std::convertible_to<Body&>;
    // clang-format on
};


template <BodyRange T, class Alloc>
void solveIslands(const T& bodies, const Alloc& alloc, World::Settings s, float dt)
{
    std::vector<Body*, ReboundTo<Alloc, Body*>> bodiesToProcess{alloc};
    for (Body& body : bodies)
        bodiesToProcess.emplace_back(&body);

    auto removeBody = [&](Body* body) { std::erase(bodiesToProcess, body); };

    Island island{alloc};
    while (!bodiesToProcess.empty())
    {
        island.init(bodiesToProcess.back());

        for (Body* body : island.bodies())
            removeBody(body);


        if (island.isAwake())
        {
            for (Body* body : island.bodies())
                body->wake();
        }
        else
        {
            // reset forces and velocities on all bodies
        }


        island.refreshProxies();

        if (island.hasConstraints())
        {
            island.applyVelocityConstraints(s.nVelocityIterations, dt);
            island.integrateBodies(dt);
            island.applyPositionConstraints(s.nPositionIterations);
        }
        else
        {
            island.integrateBodies(dt);
        }

        // structural bodies are shared across islands.
        // -> must not write back before all islands have solved (otherwise body has no index)
        // -> if the body has a velocity, it will be integrated multiple times...
        island.endSolve();

        island.clear();
    }
}

} // namespace simu
