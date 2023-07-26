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

template <Uint32 n>
Bodies<n>::Bodies(
    std::initializer_list<Body*> bodies,
    std::optional<Dominance>     dominances
)
    : Base{}
{
    Uint32 i = 0;
    for (auto body : bodies)
    {
        if (i < n)
            (*this)[i++] = body;
        else
            break;
    }

    if (dominances.has_value())
    {
        dominances_ = dominances.value();
    }
    else
    {
        Dominance d{};
        Uint32    j = 0;
        for (Body* body : *this)
            d[j++] = body->dominance();

        dominances_ = d;
    }
}

template <Uint32 n>
bool Bodies<n>::isBodyStructural(const Body* body) const
{
    for (Uint32 i = 0; i < n; ++i)
        if (body == (*this)[i])
            return dominances_[i] == 0.f;

    SIMU_ASSERT(false, "Body is not part of these bodies.");
}


template <Uint32 n>
void Bodies<n>::applyImpulse(const Impulse& impulse)
{ 
    Velocity dv = elementWiseMul(inverseMassVec(), impulse);

    Uint32   i  = 0;
    for (Body* body : *this)
    {
        body->setVelocity(body->velocity() + Vec2{dv[i], dv[i + 1]});
        body->setAngularVelocity(body->angularVelocity() + dv[i + 2]);
        i += 3;
    }
}

template <Uint32 n>
void Bodies<n>::applyPositionCorrection(const State& correction)
{
    Uint32 i = 0;
    for (Body* body : *this)
    {
        body->position_ += Vec2{correction[i], correction[i + 1]};
        body->orientation_ += correction[i + 2];
        body->update();
        i += 3;
    }
}

template <Uint32 n>
typename Bodies<n>::Velocity Bodies<n>::velocity() const
{
    Velocity v{};
    Uint32   i = 0;
    for (const Body* body : *this)
    {
        v[i++] = body->velocity()[0];
        v[i++] = body->velocity()[1];
        v[i++] = body->angularVelocity();
    }

    return v;
}

template <Uint32 n>
typename Bodies<n>::MassVec Bodies<n>::inverseMassVec() const
{
    MassVec mv;

    Uint32 i       = 0;
    Uint32 nthBody = 0;
    for (const Body* body : *this)
    {
        float m = dominances_[nthBody] / body->mass();
        float I = dominances_[nthBody++] / body->inertia();

        mv[i++] = m;
        mv[i++] = m;
        mv[i++] = I;
    }

    return mv;
}

template <Uint32 n>
typename Bodies<n>::Mass Bodies<n>::inverseMass() const
{
    return Mass::diagonal(inverseMassVec());
}

} // namespace simu
