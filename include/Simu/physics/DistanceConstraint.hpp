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

#include "Simu/physics/Constraint.hpp"

namespace simu
{

class DistanceConstraintFunction : public EqualityConstraintFunctionBase<2, 1>
{
public:

    typedef EqualityConstraintFunctionBase<2, 1> Base;


    DistanceConstraintFunction(
        const Bodies<nBodies>& bodies,
        std::array<Vec2, 2>    fixedPoints,
        std::optional<float>   distance
    )
        : Base{},
          localFixedPoints_{
              bodies[0]->toLocalSpace() * fixedPoints[0],
              bodies[1]->toLocalSpace() * fixedPoints[1]},
          distanceSquared_{
              distance.has_value() ? distance.value() * distance.value()
                                   : distanceSquared(fixedPoints)}
    {
        SIMU_ASSERT(distanceSquared_ != 0.f, "Use an HingeConstraint instead");
    }

    Value eval(const Bodies<nBodies>& bodies) const
    {
        return Value{
            distanceSquared(worldSpaceFixedPoints(bodies)) - distanceSquared_};
    }

    Value bias(const Bodies<nBodies>& /* bodies */) const { return Value{}; }

    Jacobian jacobian(const Bodies<nBodies>& bodies) const
    {
        auto worldPoints = worldSpaceFixedPoints(bodies);
        Vec2 d           = worldPoints[0] - worldPoints[1];

        std::array<Vec2, 2> fromCentroid{
            worldPoints[0] - bodies[0]->properties().centroid,
            worldPoints[1] - bodies[1]->properties().centroid};

        return 2.f * Jacobian{
            d[0],
            d[1],
            cross(d, fromCentroid[0]),
            -d[0],
            -d[1],
            -cross(d, fromCentroid[1])};
    }

    std::array<Vec2, 2> worldSpaceFixedPoints(const Bodies<nBodies>& bodies) const
    {
        return {
            bodies[0]->toWorldSpace() * localFixedPoints_[0],
            bodies[1]->toWorldSpace() * localFixedPoints_[1]};
    }

private:

    float distanceSquared(std::array<Vec2, 2> worldPoints) const
    {
        return normSquared(worldPoints[0] - worldPoints[1]);
    }

    std::array<Vec2, 2> localFixedPoints_;
    float               distanceSquared_;
};

class DistanceConstraint
    : public ConstraintImplementation<DistanceConstraintFunction>
{
public:

    typedef ConstraintImplementation<DistanceConstraintFunction> Base;

    DistanceConstraint(
        const Bodies<2>&     bodies,
        std::array<Vec2, 2>  fixedPoints,
        std::optional<float> distance        = std::nullopt,
        bool                 disableContacts = true
    )
        : Base{
            bodies,
            DistanceConstraintFunction{bodies, fixedPoints, distance},
            disableContacts
    }
    {
    }
};

} // namespace simu
