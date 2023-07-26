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

#include "Simu/config.hpp"
#include "Simu/physics/BoundingBox.hpp"
#include "Simu/physics/Body.hpp"

namespace simu
{

class ForceField : public PhysicsObject
{
public:

    enum class DomainType
    {
        global,
        region
    };

    struct Domain
    {
        DomainType  type;
        BoundingBox region;
    };

    static Domain global()
    {
        return Domain{
            DomainType::global,
            BoundingBox{Vec2{1, 0}, Vec2{0, 0}}
        };
    }

    static Domain region(Vec2 min, Vec2 max)
    {
        return Domain{
            DomainType::region,
            BoundingBox{min, max}
        };
    }

    ForceField(Domain domain) : domain_{domain} {}
    ~ForceField() override = default;

    Domain domain() const { return domain_; }

    virtual void apply(Body& body, float dt) const = 0;

protected:

    Domain domain_;
};


class LinearField : public ForceField
{
public:

    explicit LinearField(Vec2 force, const Domain& domain = ForceField::global())
        : ForceField{domain}, force_{force}
    {
    }

    void apply(Body& body, float dt) const override
    {
        body.applyForce(force_, dt, Vec2{0, 0});
    }

private:

    Vec2 force_;
};

class Gravity : public ForceField
{
public:

    explicit Gravity(Vec2 acceleration = Vec2{0, -9.81f})
        : ForceField{ForceField::global()}, acceleration_{acceleration}
    {
    }

    void apply(Body& body, float dt) const override
    {
        body.applyForce(acceleration_ * body.mass(), dt, Vec2{0, 0});
    }

private:

    Vec2 acceleration_;
};

} // namespace simu
