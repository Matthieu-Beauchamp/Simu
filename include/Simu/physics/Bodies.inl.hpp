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

    invMasses_.m0 = d[0] * bodies_[0]->invMass();
    invMasses_.I0 = d[0] * bodies_[0]->invInertia();
    invMasses_.m1 = d[1] * bodies_[1]->invMass();
    invMasses_.I1 = d[1] * bodies_[1]->invInertia();
}


bool Bodies::isBodyStructural(const Body* body) const
{
    if (body == bodies_[0])
        return invMasses_.m0 == 0.f;

    if (body == bodies_[1])
        return invMasses_.m1 == 0.f;

    SIMU_ASSERT(false, "Body is not part of these bodies.");
    return true;
}


void Bodies::applyImpulse(const Impulse& P)
{
    assertHasProxies();
    const InvMasses& m = invMasses_;

    proxies_[0]->incrementVel(m.m0* Vec2{P[0], P[1]}, m.I0 * P[2]);
    proxies_[1]->incrementVel(m.m1* Vec2{P[3], P[4]}, m.I1 * P[5]);
}


void Bodies::applyPositionCorrection(const State& S)
{
    assertHasProxies();
    const InvMasses& m = invMasses_;

    proxies_[0]->advancePos(m.m0* Vec2{S[0], S[1]}, m.I0 * S[2]);
    proxies_[1]->advancePos(m.m1* Vec2{S[3], S[4]}, m.I1 * S[5]);
}


typename Bodies::VelocityVec Bodies::velocity() const
{
    assertHasProxies();

    VelocityVec v{};
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
