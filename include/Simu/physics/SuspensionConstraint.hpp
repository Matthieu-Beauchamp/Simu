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
#include "Simu/physics/Constraint.hpp"

namespace simu
{

class SuspensionConstraintFunction
{
public:

    static constexpr Uint32 nBodies   = 2;
    static constexpr Uint32 dimension = 2;

    typedef Vector<float, dimension>    Value;
    typedef Matrix<float, dimension, 6> Jacobian;

    ////////////////////////////////////////////////////////////
    /// \param bodies bodies[0] is the chassis, bodies[1] is the wheel.
    /// \param springDir the direction of the spring from the chassis to the wheel (world space).
    /// \param springHeight The length of the spring at rest
    /// \param chassisPoint The world space point where the spring is fixed to the chassis.
    /// \param wheelPoint The world space point where the spring is fixed to the wheel (defaults to centroid).
    ////////////////////////////////////////////////////////////
    SuspensionConstraintFunction(
        const Bodies&       bodies,
        Vec2                springDir,
        float               springHeight,
        Vec2                chassisPoint,
        std::optional<Vec2> wheelPoint
    )
    {
        Transform toChassisLocal = bodies[0]->toLocalSpace();

        springDir_     = normalized(toChassisLocal.rotation() * springDir);
        perpSpringDir_ = perp(springDir_);

        springHeight_ = springHeight;
        SIMU_ASSERT(springHeight_ >= 0.f, "Should not be negative");

        chassisPoint_ = toChassisLocal * chassisPoint;
        wheelPoint_   = wheelPoint.has_value()
                            ? bodies[1]->toLocalSpace() * wheelPoint.value()
                            : bodies[1]->localCentroid();
    }

    Value eval(const Proxies& proxies) const
    {
        WorldSpace ws = worldSpaceInfo(proxies);

        Vec2 d = ws.wheelPoint - ws.chassisPoint;

        return Value{
            dot(d, ws.springDir) - springHeight_,
            dot(d, ws.perpSpringDir),
        };
    }

    Value bias(const Proxies&) const { return Value{}; }

    Jacobian jacobian(const Proxies& proxies) const
    {
        WorldSpace ws = worldSpaceInfo(proxies);

        Vec2 d = ws.wheelPoint - ws.chassisPoint;

        typedef Vector<float, Jacobian::nCols> Row;

        Row springRow{
            -ws.springDir[0],
            -ws.springDir[1],
            cross(ws.springDir, d) - cross(ws.chassisRadius, ws.springDir),

            ws.springDir[0],
            ws.springDir[1],
            cross(ws.wheelRadius, ws.springDir),
        };

        Row perpRow{
            -ws.perpSpringDir[0],
            -ws.perpSpringDir[1],
            cross(ws.perpSpringDir, d) - cross(ws.chassisRadius, ws.perpSpringDir),

            ws.perpSpringDir[0],
            ws.perpSpringDir[1],
            cross(ws.wheelRadius, ws.perpSpringDir),
        };

        return Jacobian::fromRows({springRow, perpRow});
    }

    Value clampLambda(Value lambda, float /* dt */) const { return lambda; }

    Value clampPositionLambda(Value lambda) const
    {
        lambda[0] = 0.f; // Baumgarte position correction for spring.
        return lambda;
    }


private:

    struct WorldSpace
    {
        Transform chassisToWorld;
        Transform wheelToWorld;

        Vec2 springDir;
        Vec2 perpSpringDir;

        Vec2 chassisPoint;
        Vec2 chassisRadius;

        Vec2 wheelPoint;
        Vec2 wheelRadius;
    };

    WorldSpace worldSpaceInfo(const Proxies& proxies) const
    {
        WorldSpace ws{proxies[0].toWorldSpace(), proxies[1].toWorldSpace()};

        ws.springDir     = ws.chassisToWorld.rotation() * springDir_;
        ws.perpSpringDir = ws.chassisToWorld.rotation() * perpSpringDir_;

        ws.chassisPoint  = ws.chassisToWorld * chassisPoint_;
        ws.chassisRadius = ws.chassisPoint - proxies[0].centroid();

        ws.wheelPoint  = ws.wheelToWorld * wheelPoint_;
        ws.wheelRadius = ws.wheelPoint - proxies[1].centroid();
        return ws;
    }

    Vec2  springDir_;
    Vec2  perpSpringDir_;
    float springHeight_;

    Vec2 chassisPoint_;
    Vec2 wheelPoint_;
};


class SuspensionConstraint
    : public ConstraintImplementation<SuspensionConstraintFunction>
{
    typedef ConstraintImplementation<SuspensionConstraintFunction> Base;

public:

    ////////////////////////////////////////////////////////////
    /// \param bodies bodies[0] is the chassis, bodies[1] is the wheel.
    /// \param springDir the direction of the spring from the chassis to the wheel (world space).
    /// \param springHeight The length of the spring at rest
    /// \param chassisPoint The world space point where the spring is fixed to the chassis.
    /// \param wheelPoint The world space point where the spring is fixed to the wheel (defaults to centroid).
    ////////////////////////////////////////////////////////////
    SuspensionConstraint(
        const Bodies&       bodies,
        Vec2                springDir,
        float               springHeight,
        Vec2                chassisPoint,
        std::optional<Vec2> wheelPoint      = std::nullopt,
        bool                disableContacts = true
    )
        : Base{
            bodies,
            SuspensionConstraintFunction{
                                         bodies, springDir,
                                         springHeight, chassisPoint,
                                         wheelPoint},
            disableContacts
    }
    {
        // Fully rigid
        setSpringDamping(0.f);
        setSpringRestitution(1.f);
    }

    float springDamping() const { return solver.damping()[0]; }
    void  setSpringDamping(float damping) { solver.damping()[0] = damping; }

    float springRestitution() const { return solver.restitution()[0]; }
    void  setSpringRestitution(float restitution)
    {
        solver.restitution()[0] = restitution;
    }
};


} // namespace simu
