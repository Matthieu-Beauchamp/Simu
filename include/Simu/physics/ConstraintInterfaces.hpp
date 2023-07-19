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


namespace simu
{

class Body;

typedef ViewType<Body**>       BodiesView;
typedef ViewType<Body const**> ConstBodiesView;

template <Uint32 n>
class Bodies;

class Constraint : public PhysicsObject
{
public:

    ~Constraint() override = default;

    virtual bool isActive()                = 0;
    virtual void initSolve(float dt)       = 0;
    virtual void solveVelocities(float dt) = 0;
    virtual void solvePositions()          = 0;


    // if a constraint has no bodies or changes its bodies during its lifetime,
    //  then behavior is undefined.
    virtual BodiesView      bodies()       = 0;
    virtual ConstBodiesView bodies() const = 0;

    virtual bool isBodyStructural(const Body* body) const = 0;
};


template <class F>
concept ConstraintFunction = requires(
    F                  f,
    typename F::Value  lambda,
    Bodies<F::nBodies> bodies,
    float              dt
) {
    // clang-format off
    typename F::Value;
    std::is_same_v<typename F::Value, Vector<float, F::dimension>>; 
    
    typename F::Jacobian;
    std::is_same_v<typename F::Jacobian, Matrix<float, F::dimension, 3*F::nBodies>>; 

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
    = requires(S s, Bodies<S::nBodies>& bodies, typename S::F f, float dt) 
{
    typename S::F;
    requires ConstraintFunction<typename S::F>;

    requires std::derived_from<S, ConstraintSolverBase<typename S::F>>;

    requires std::constructible_from<S, 
        Bodies<S::nBodies>, 
        typename S::F>;

    { s.initSolve(bodies, f, dt) };
    { s.solveVelocity(bodies, f, dt) };
    { s.solvePosition(bodies, f) };

};
// clang-format on


} // namespace simu
