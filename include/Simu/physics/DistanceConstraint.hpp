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

// TODO: Distance velocity constraint is degenerate if points are equal,
//  this is not handled.

class DistanceConstraintFunction : public EqualityConstraintFunctionBase<2, 1>
{
public:

    typedef EqualityConstraintFunctionBase<2, 1> Base;


    DistanceConstraintFunction(
        const Bodies<nBodies>& bodies,
        std::array<Vec2, 2>    fixedPoints
    )
        : Base{},
          localFixedPoints_{
              bodies[0]->toLocalSpace() * fixedPoints[0],
              bodies[1]->toLocalSpace() * fixedPoints[1]}
    {
    }

    Value eval(const Bodies<nBodies>& bodies) const
    {
        return Value{distanceSquared(worldSpaceFixedPoints(bodies))};
    }

    Value bias(const Bodies<nBodies>& /* bodies */) const { return Value{}; }

    Jacobian jacobian(const Bodies<nBodies>& bodies) const
    {
        auto worldPoints = worldSpaceFixedPoints(bodies);
        Vec2 d           = worldPoints[0] - worldPoints[1];

        std::array<Vec2, 2> fromCentroid{
            worldPoints[0] - bodies[0]->centroid(),
            worldPoints[1] - bodies[1]->centroid()};

        return 2.f
               * Jacobian{
                   d[0],
                   d[1],
                   cross(fromCentroid[0], d),
                   -d[0],
                   -d[1],
                   -cross(fromCentroid[1], d)};
    }

    std::array<Vec2, 2> worldSpaceFixedPoints(const Bodies<nBodies>& bodies) const
    {
        return {
            bodies[0]->toWorldSpace() * localFixedPoints_[0],
            bodies[1]->toWorldSpace() * localFixedPoints_[1]};
    }

    static float distanceSquared(std::array<Vec2, 2> worldPoints)
    {
        return normSquared(worldPoints[0] - worldPoints[1]);
    }

private:

    std::array<Vec2, 2> localFixedPoints_;
};

class DistanceConstraint : public ConstraintImplementation<
                               DistanceConstraintFunction,
                               LimitsSolver<DistanceConstraintFunction>>
{
public:

    typedef ConstraintImplementation<
        DistanceConstraintFunction,
        LimitsSolver<DistanceConstraintFunction>>
        Base;

    DistanceConstraint(
        const Bodies<2>&    bodies,
        std::array<Vec2, 2> fixedPoints,
        bool                disableContacts = true
    )
        : DistanceConstraint{
            bodies,
            fixedPoints,
            norm(fixedPoints[1] - fixedPoints[0]),
            disableContacts}
    {
    }

    DistanceConstraint(
        const Bodies<2>&    bodies,
        std::array<Vec2, 2> fixedPoints,
        float               distance,
        bool                disableContacts = true
    )
        : DistanceConstraint{bodies, fixedPoints, distance, distance, disableContacts}
    {
    }

    DistanceConstraint(
        const Bodies<2>&     bodies,
        std::array<Vec2, 2>  fixedPoints,
        std::optional<float> minDistance,
        std::optional<float> maxDistance,
        bool                 disableContacts = true
    )
        : Base{
            bodies,
            DistanceConstraintFunction{bodies, fixedPoints},
            disableContacts
    }
    {
        SIMU_ASSERT(
            minDistance.has_value() || maxDistance.has_value(),
            "This constraint has no effect."
        );

        SIMU_ASSERT(
            !minDistance.has_value() || minDistance.value() > 0.f,
            "Cannot enforce a minimal distance <= 0. Use an Hinge constraint "
            "for distance of 0."
        );

        SIMU_ASSERT(
            maxDistance > 0.f,
            "Cannot enforce a maximal distance <= 0. Use an Hinge constraint "
            "for a distance of 0."
        );

        if (minDistance.has_value())
            solver.setLowerLimit(Value{minDistance.value() * minDistance.value()}
            );

        if (maxDistance.has_value())
            solver.setUpperLimit(Value{maxDistance.value() * maxDistance.value()}
            );
    }
};

} // namespace simu
