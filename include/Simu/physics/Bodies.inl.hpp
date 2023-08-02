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

#include "Simu/physics/Bodies.hpp"
#include "Simu/physics/Body.hpp"

namespace simu
{


Bodies::Bodies(std::initializer_list<Body*> bodies, std::optional<Dominance> dominances)
{
    SIMU_ASSERT(
        bodies.size() == 2,
        "Incorrect number of bodies in initializer list."
    );

    Uint32 i = 0;
    for (auto body : bodies)
    {
        bodies_[i++] = body;
    }

    Dominance d = dominances.value_or(
        Vec2{bodies_[0]->dominance(), bodies_[1]->dominance()}
    );

    i              = 0;
    Uint32 nthBody = 0;
    for (const Body* body : bodies_)
    {
        float m = d[nthBody] * body->invMass();
        float I = d[nthBody++] * body->invInertia();

        invMassVec_[i++] = m;
        invMassVec_[i++] = m;
        invMassVec_[i++] = I;
    }
}


bool Bodies::isBodyStructural(const Body* body) const
{
    for (Uint32 i = 0; i < n; ++i)
        if (body == bodies_[i])
            return invMassVec_[3 * i] == 0.f;

    SIMU_ASSERT(false, "Body is not part of these bodies.");
    return true;
}


void Bodies::applyImpulse(const Impulse& impulse)
{
    assertHasProxies();
    Velocity dv = elementWiseMul(inverseMassVec(), impulse);

    Uint32 i = 0;
    for (SolverProxy* p : proxies_)
    {
        p->setVelocity(
            p->velocity() + Vec2{dv[i], dv[i + 1]},
            p->angularVelocity() + dv[i + 2]

        );

        i += 3;
    }
}


void Bodies::applyPositionCorrection(const State& correction)
{
    assertHasProxies();
    Uint32 i = 0;
    for (SolverProxy* p : proxies_)
    {
        p->advance(Vec2{correction[i], correction[i + 1]}, correction[i + 2]);
        i += 3;
    }
}


typename Bodies::Velocity Bodies::velocity() const
{
    assertHasProxies();

    Velocity v{};
    Uint32   i = 0;
    for (const SolverProxy* p : proxies_)
    {
        v[i++] = p->velocity()[0];
        v[i++] = p->velocity()[1];
        v[i++] = p->angularVelocity();
    }

    return v;
}


} // namespace simu
