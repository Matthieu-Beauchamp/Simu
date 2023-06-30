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

#include <vector>

#include "Simu/config.hpp"
#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/PhysicsBody.hpp"

#include "Simu/physics/Sleepable.hpp"

namespace simu
{

class Island : public Sleepable
{
public:

    bool contains(PhysicsBody* body)
    {
        std::find(bodies_.begin(), bodies_.end(), body) == bodies_.end();
    }

    bool contains(Constraint* constraint)
    {
        std::find(constraints_.begin(), constraints_.end(), constraint)
            == constraints_.end();
    }

    auto bodies() { return makeView(bodies_.begin(), bodies_.end()); }
    auto bodies() const { return makeView(bodies_.begin(), bodies_.end()); }

    auto constraints()
    {
        return makeView(constraints_.begin(), constraints_.end());
    }
    auto constraints() const
    {
        return makeView(constraints_.begin(), constraints_.end());
    }


    ////////////////////////////////////////////////////////////
    // Sleeping
    ////////////////////////////////////////////////////////////

    bool canSleep(
        float velocityTreshold = simu::EPSILON,
        float angularTreshold  = simu::EPSILON
    ) const
    {
        bool immobile = true;
        for (const PhysicsBody* body : bodies_)
        {
            immobile
                = immobile
                  && (all(std::abs(body->velocity())
                          <= Vec2::filled(velocityTreshold))
                      && std::abs(body->angularVelocity()) <= angularTreshold);
        }
    }

    void sleep() override
    {
        for (PhysicsBody* body : bodies_)
            body->sleep();
        for (Constraint* constraint : constraints_)
            constraint->sleep();
    }

    void wake() override
    {
        for (PhysicsBody* body : bodies_)
            body->wake();
        for (Constraint* constraint : constraints_)
            constraint->wake();
    }

    bool isAsleep() const override
    {
        for (PhysicsBody* body : bodies_)
            if (!body->isAsleep())
                return false;
        for (Constraint* constraint : constraints_)
            if (!constraint->isAsleep())
                return false;

        return true;
    }

private:

    friend class Islands;

    Island(PhysicsBody* body) { bodies_.emplace_back(body); }

    void mergeWith(Island& other)
    {
        bodies_.insert(bodies_.end(), other.bodies_.begin(), other.bodies_.end());
        other.bodies_.clear();

        constraints_.insert(
            constraints_.end(),
            other.constraints_.begin(),
            other.constraints_.end()
        );
        other.constraints_.clear();
    }


    std::vector<Constraint*>  constraints_{};
    std::vector<PhysicsBody*> bodies_{};
};


class Islands
{
public:

    auto all()
    {
        return makeView(islands_.begin(), islands_.end(), BypassSmartPointer{});
    }

    auto all() const
    {
        return makeView(islands_.begin(), islands_.end(), BypassSmartPointer{});
    }

    void addBody(PhysicsBody* body)
    {
        islands_.emplace_back(std::make_unique<Island>(body));
    }

    // the bodies of the constraint must have been added previously
    void addConstraint(Constraint* constraint)
    {
        auto bodies = constraint->bodies();

        iterator island = islandOf(bodies.front());
        for (PhysicsBody* body : bodies)
            island = merge(island, islandOf(body));

        (*island)->constraints_.emplace_back(constraint);
    }

    // if a body is removed, then all constraints on that body must be removed too.
    void cleanup(
        const std::vector<Constraint*>&  constraintsToRemove,
        const std::vector<PhysicsBody*>& bodiesToRemove
    )
    {
        // TODO: There may be an easy to split the connected components directly,
        //  but since the constraints may have 1, 2, 3, ... bodies the graph representation is harder.

        std::vector<iterator> affected{};
        for (Constraint* constraint : constraintsToRemove)
        {
            iterator island = islandOf(constraint);
            if (std::find(affected.begin(), affected.end(), island)
                == affected.end())
                affected.emplace_back(island);
        }

        for (PhysicsBody* body : bodiesToRemove)
        {
            iterator island = islandOf(body);
            if (std::find(affected.begin(), affected.end(), island)
                == affected.end())
                affected.emplace_back(island);
        }

        std::vector<Constraint*>  keptConstraints;
        std::vector<PhysicsBody*> keptBodies;

        for (iterator island : affected)
        {
            for (PhysicsBody* body : (*island)->bodies())
                if (std::find(bodiesToRemove.begin(), bodiesToRemove.end(), body)
                    == bodiesToRemove.end())
                    keptBodies.emplace_back(body);

            for (Constraint* constraint : (*island)->constraints())
                if (std::find(
                        constraintsToRemove.begin(),
                        constraintsToRemove.end(),
                        constraint
                    )
                    == constraintsToRemove.end())
                    keptConstraints.emplace_back(constraint);
        }

        for (iterator island : affected)
            islands_.erase(island);

        for (PhysicsBody* body : keptBodies)
            addBody(body);

        for (Constraint* constraint : keptConstraints)
            addConstraint(constraint);
    }

private:

    typedef std::list<std::unique_ptr<Island>>::iterator       iterator;
    typedef std::list<std::unique_ptr<Island>>::const_iterator const_iterator;

    iterator islandOf(PhysicsBody* body)
    {
        iterator island = std::find_if(
            islands_.begin(),
            islands_.end(),
            [=](const std::unique_ptr<Island>& island) {
                return island->contains(body);
            }
        );

        SIMU_ASSERT(
            island != islands_.end(),
            "The body is expected to have been added to the islands"
        );

        return island;
    }

    iterator islandOf(Constraint* constraint)
    {
        iterator island = std::find_if(
            islands_.begin(),
            islands_.end(),
            [=](const std::unique_ptr<Island>& island) {
                return island->contains(constraint);
            }
        );

        SIMU_ASSERT(
            island != islands_.end(),
            "The constraint is expected to have been added to the islands"
        );

        return island;
    }

    iterator merge(iterator first, iterator second)
    {
        if (first != second)
        {
            (*first)->mergeWith(**second);
            islands_.erase(second);
        }

        return first;
    }

    std::list<std::unique_ptr<Island>> islands_;
};

} // namespace simu
