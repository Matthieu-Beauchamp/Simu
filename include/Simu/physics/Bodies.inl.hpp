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
#include "Simu/physics/PhysicsBody.hpp"

namespace simu
{

template <Uint32 n>
Bodies<n>::Bodies(
    std::initializer_list<PhysicsBody*> bodies,
    std::optional<Dominance>            dominances
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
        for (PhysicsBody* body : *this)
            d[j++] = body->dominance();

        dominances_ = d;
    }
}

template <Uint32 n>
bool Bodies<n>::isBodyStructural(const PhysicsBody* body) const
{
    for (Uint32 i = 0; i < n; ++i)
        if (body == (*this)[i])
            return dominances_[i] == 0.f;

    SIMU_ASSERT(false, "Body is not part of this constraint.");
}


template <Uint32 n>
void Bodies<n>::applyImpulse(const Impulse& impulse)
{
    Velocity dv = inverseMass() * impulse;
    Uint32   i  = 0;
    for (PhysicsBody* body : *this)
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
    for (PhysicsBody* body : *this)
    {
        body->position_ += Vec2{correction[i], correction[i + 1]};
        body->orientation_ += correction[i + 2];
        body->collider_.update(body->toWorldSpace());
        i += 3;
    }
}

template <Uint32 n>
typename Bodies<n>::Velocity Bodies<n>::velocity() const
{
    Velocity v{};
    Uint32   i = 0;
    for (const PhysicsBody* body : *this)
    {
        v[i++] = body->velocity()[0];
        v[i++] = body->velocity()[1];
        v[i++] = body->angularVelocity();
    }

    return v;
}

template <Uint32 n>
typename Bodies<n>::Mass Bodies<n>::inverseMass() const
{
    Vector<float, 3 * n> diagonal;

    Uint32 i       = 0;
    Uint32 nthBody = 0;
    for (const PhysicsBody* body : *this)
    {
        diagonal[i++] = dominances_[nthBody] / body->properties().mass;
        diagonal[i++] = dominances_[nthBody] / body->properties().mass;
        diagonal[i++] = dominances_[nthBody++] / body->properties().inertia;
    }

    return Mass::diagonal(diagonal);
}

} // namespace simu
