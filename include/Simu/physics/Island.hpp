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

#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/Constraint.hpp"

namespace simu
{

struct Island
{
public:

    Island(PhysicsBody* root)
    {
        addBody(root);
        expand(root);
    }

    void expand(PhysicsBody* root)
    {
        if (isTrueStructural(root))
            return;
            
        for (Constraint* constraint : root->constraints_)
        {
            if (constraints_.emplace(constraint).second)
            {
                for (PhysicsBody* body : constraint->bodies())
                {
                    if (!isTrueStructural(body))
                    {
                        addBody(body);
                        expand(body);
                    }
                }
            }
        }
    }

    bool isAwake() const { return isAwake_; }
    auto bodies() { return makeView(bodies_.begin(), bodies_.end()); }
    auto constraints()
    {
        return makeView(constraints_.begin(), constraints_.end());
    }

private:
    static bool isTrueStructural(PhysicsBody* body){
        if (body->constraints_.empty())
            return false;

        bool isStructural = true;
        for (Constraint* constraint : body->constraints_)
            isStructural = isStructural && constraint->isBodyStructural(body);

        return isStructural;
    }

    void addBody(PhysicsBody* body)
    {
        bodies_.emplace(body);
        isAwake_ = isAwake_ || !body->isAsleep();
    }

    std::set<PhysicsBody*> bodies_;
    std::set<Constraint*>  constraints_;
    bool                   isAwake_ = false;
};

template <class T>
concept BodyRange = std::ranges::range<T> && requires(T t) {
    // clang-format off
    { *t.begin() } -> std::convertible_to<PhysicsBody&>;
    // clang-format on
};

class Islands
{
public:

    template <BodyRange T>
    Islands(T&& bodies)
    {
        std::vector<PhysicsBody*> bodiesToProcess;
        for (PhysicsBody& body : bodies)
            bodiesToProcess.emplace_back(&body);

        auto removeBody = [&](PhysicsBody* body) {
            bodiesToProcess.erase(
                std::remove(bodiesToProcess.begin(), bodiesToProcess.end(), body),
                bodiesToProcess.end()
            );
        };

        while (!bodiesToProcess.empty())
        {
            // TODO: Skip structural bodies, what if one is structural,
            //  but never behaves like one according to constraints?

            Island island{bodiesToProcess.back()};
            for (PhysicsBody* body : island.bodies())
                removeBody(body);

            islands_.emplace_back(std::move(island));
        }

        for (auto it = islands_.begin(); it != islands_.end();)
        {
            if (it->constraints().empty())
                it = islands_.erase(it);
            else
                ++it;
        }
    }

    auto islands() { return makeView(islands_.begin(), islands_.end()); }

private:

    std::vector<Island> islands_;
};


} // namespace simu
