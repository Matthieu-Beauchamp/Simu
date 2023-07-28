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

#include "Simu/utility/View.hpp"
#include "Simu/math/Matrix.hpp"

#include "Simu/physics/PhysicsObject.hpp"
#include "Simu/physics/Bodies.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief
///
/// Call sequence pseudo-code:
///
/// \code
/// let S be the set of active constraints;
/// foreach constraint c:
///     if c.isActive():
///         S <- S U c;
///
/// foreach constraint c in S:
///     c.initSolve();
///
/// foreach constraint c in S:
///     c.warmstart();
///
/// for i in range(world.nVelocityIterations)
///     foreach constraint c in S:
///         c.solveVelocities();
///
/// for i in range(world.nPositionIterations)
///     foreach constraint c:
///         c.solvePositions();
/// \endcode
///
///
////////////////////////////////////////////////////////////
class Constraint : public PhysicsObject
{
public:

    Constraint(const Bodies& bodies) : bodies_{bodies}
    {
        auto b = bodies_.bodies();

        bool tooManyStructural = false;
        if (b[0] == b[1] && bodies_.isBodyStructural(b[0]))
            tooManyStructural = true;
        else if (bodies_.isBodyStructural(b[0]) && bodies_.isBodyStructural(b[1]))
            tooManyStructural = true;

        SIMU_ASSERT(
            !tooManyStructural,
            "Cannot solve a constraint with multiple structural bodies."
        );

        SIMU_ASSERT(b[0] != nullptr && b[1] != nullptr, "Invalid body");
    }

    ~Constraint() override = default;

    bool shouldDie() const override
    {
        for (const Body* body : bodies().bodies())
            if (body->isDead())
                return true;

        return false;
    }

    virtual bool isActive() = 0;

    virtual void initSolve()         = 0;
    virtual void warmstart(float dt) = 0;

    virtual void solveVelocities(float dt) = 0;
    virtual void solvePositions()          = 0;


    Bodies&       bodies() { return bodies_; }
    const Bodies& bodies() const { return bodies_; }

private:

    Bodies bodies_;
};


template <class F>
concept ConstraintFunction = requires(
    F                 f,
    typename F::Value lambda,
    Bodies            bodies,
    float             dt
) {
    requires F::nBodies == 1 || F::nBodies == 2;

    // clang-format off
    typename F::Value;
    requires std::is_same_v<typename F::Value, Vector<float, F::dimension>>; 
    
    typename F::Jacobian;
    requires std::is_same_v<typename F::Jacobian, Matrix<float, F::dimension, 6>>; 

    { f.eval(bodies) } -> std::same_as<typename F::Value>;
    { f.bias(bodies) } -> std::same_as<typename F::Value>;

    { f.jacobian(bodies) } -> std::same_as<typename F::Jacobian>;
    
    { f.clampLambda(lambda, dt) } -> std::same_as<typename F::Value>;
    { f.clampPositionLambda(lambda) } -> std::same_as<typename F::Value>;
    // clang-format on
};


template <ConstraintFunction F>
class ConstraintSolverBase;

// clang-format off
template <class S>
concept ConstraintSolver
    = requires(S s, Bodies& bodies, typename S::F f, float dt) 
{
    typename S::F;
    requires ConstraintFunction<typename S::F>;

    requires std::derived_from<S, ConstraintSolverBase<typename S::F>>;

    requires std::constructible_from<S, Bodies, typename S::F>;


    { s.initSolve(bodies, f) };
    { s.warmstart(bodies, f, dt) };

    { s.solveVelocity(bodies, f, dt) };
    { s.solvePosition(bodies, f) };

};
// clang-format on


} // namespace simu
