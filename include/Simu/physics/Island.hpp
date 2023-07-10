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

struct Island
{
public:

    Island(Body* root)
    {
        addBody(root);
        expand(root);
    }

    void expand(Body* root)
    {
        if (root->interactsAsStructural())
            return;

        for (Constraint* constraint : root->constraints())
        {
            if (constraints_.emplace(constraint).second)
            {
                for (Body* body : constraint->bodies())
                {
                    if (!body->interactsAsStructural())
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

    void addBody(Body* body)
    {
        bodies_.emplace(body);
        isAwake_ = isAwake_ || !body->isAsleep();
    }

    std::set<Body*> bodies_;
    std::set<Constraint*>  constraints_;
    bool                   isAwake_ = false;
};

template <class T>
concept BodyRange = std::ranges::range<T> && requires(T t) {
    // clang-format off
    { *t.begin() } -> std::convertible_to<Body&>;
    // clang-format on
};

class Islands
{
public:

    template <BodyRange T>
    Islands(T&& bodies)
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
            if (bodiesToProcess.back()->constraints().empty()
                || bodiesToProcess.back()->interactsAsStructural())
            {
                bodiesToProcess.pop_back();
            }
            else
            {
                Island island{bodiesToProcess.back()};
                for (Body* body : island.bodies())
                    removeBody(body);

                islands_.emplace_back(std::move(island));
            }
        }
    }

    auto islands() { return makeView(islands_.begin(), islands_.end()); }

private:

    std::vector<Island> islands_;
};


} // namespace simu
