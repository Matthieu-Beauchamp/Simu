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

    typedef typename PhysicsObject::PhysicsAlloc       Alloc;
    typedef typename Alloc::rebind<Body*>::other       BAlloc;
    typedef typename Alloc::rebind<Constraint*>::other CAlloc;


    Island(Body* root, const Alloc& alloc)
        : bodies_{alloc}, constraints_{alloc}, proxies_{alloc}
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
                    addConstraint(constraint);
                }
            }
        }
    }

    Island(const Island&) = delete;
    Island(Island&&)      = delete;

    void endSolve()
    {
        for (SolverProxy& s : proxies_)
            s.writeBack();

        for (Constraint* c : constraints_)
            c->bodies().endSolve();
    }

    bool isAwake() const { return isAwake_; }
    auto bodies() { return makeView(bodies_.begin(), bodies_.end()); }
    auto constraints()
    {
        return makeView(constraints_.begin(), constraints_.end());
    }

    void refreshProxies()
    {
        // must do this after forces are applied...
        for (SolverProxy& p : proxies_)
            p.refresh();

        for (Constraint* c : constraints_)
            c->bodies().startSolve(proxies_.data()); // refresh proxy pointers
    }

    void applyVelocityConstraints(Uint32 nIter, float dt)
    {
        for (Constraint* constraint : constraints_)
            constraint->initSolve();

        for (Constraint* constraint : constraints_)
            constraint->warmstart(dt);

        for (Uint32 iter = 0; iter < nIter; ++iter)
        {
            for (Constraint* constraint : constraints_)
                constraint->solveVelocities(dt);
        }
    }

    void integrateBodies(float dt)
    {
        for (SolverProxy& p : proxies_)
            p.setPosition(
                p.position() + p.velocity() * dt,
                p.orientation() + p.angularVelocity() * dt
            );
    }

    void applyPositionConstraints(Uint32 nIter)
    {
        for (Uint32 iter = 0; iter < nIter; ++iter)
        {
            for (Constraint* constraint : constraints_)
                constraint->solvePositions();
        }
    }

private:

    bool addBody(Body* body)
    {
        bool isNew = body->proxyIndex == Body::NO_INDEX;

        if (body->interactsAsStructural())
            isNew = isNew
                    || std::find(bodies_.begin(), bodies_.end(), body)
                           == bodies_.end();

        if (isNew)
        {
            bodies_.emplace_back(body);
            isAwake_ = isAwake_ || !body->isAsleep();

            body->proxyIndex = static_cast<Int32>(proxies_.size());
            proxies_.emplace_back(body);
        }

        return isNew;
    }

    bool addConstraint(Constraint* constraint)
    {
        if (constraint->bodies().isSolving())
            return false;

        constraints_.emplace_back(constraint);

        auto  b      = constraint->bodies().bodies();
        Int32 index1 = b[0]->proxyIndex;
        Int32 index2 = b[1]->proxyIndex;
        bool  added1 = addBody(b[0]);
        bool  added2 = addBody(b[1]);

        constraint->bodies().startSolve(proxies_.data());

        if (!constraint->isActive())
        {
            if (added2)
            {
                b[1]->proxyIndex = index2;
                bodies_.pop_back();
                proxies_.pop_back();
            }

            if (added1)
            {
                b[0]->proxyIndex = index1;
                bodies_.pop_back();
                proxies_.pop_back();
            }

            constraint->bodies().endSolve();
            constraints_.pop_back();
            return false;
        }

        return true;
    }

    std::vector<Body*, BAlloc>       bodies_;
    std::vector<Constraint*, CAlloc> constraints_;

    typedef typename Alloc::rebind<SolverProxy>::other ProxyAlloc;
    std::vector<SolverProxy, ProxyAlloc>               proxies_;

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
    std::vector<Body*> bodiesToProcess;
    for (Body& body : bodies)
        bodiesToProcess.emplace_back(&body);

    auto removeBody = [&](Body* body) {
        bodiesToProcess.erase(
            std::remove(bodiesToProcess.begin(), bodiesToProcess.end(), body),
            bodiesToProcess.end()
        );
    };

    while (!bodiesToProcess.empty())
    {
        Island island(bodiesToProcess.back(), alloc);

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
        island.applyVelocityConstraints(s.nVelocityIterations, dt);
        island.integrateBodies(dt);
        island.applyPositionConstraints(s.nPositionIterations);


        // structural bodies are shared across islands.
        // -> must not write back before all islands have solved (otherwise body has no index)
        // -> if the body has a velocity, it will be integrated multiple times...
        island.endSolve();
    }
}

} // namespace simu
