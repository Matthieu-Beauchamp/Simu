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

#include <iostream>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/utility/Callable.hpp"

#include "Simu/physics/ConstraintInterfaces.hpp"
#include "Simu/physics/ConstraintSoftness.hpp"

namespace simu
{

// TODO: Position error should not be fully corrected in a single iteration,
//          see contact constraints.

namespace constraint
{

template <Uint32 dimension>
using Value = Vector<float, dimension>;

template <Uint32 dimension>
using Jacobian = Matrix<float, dimension, 6>;

template <Uint32 dimension>
using EffectiveMass = Matrix<float, dimension, dimension>;

typedef typename Proxies::Mass    Mass;
typedef typename Proxies::MassVec MassVec;

typedef typename Proxies::State       State;
typedef typename Proxies::VelocityVec VelocityVec;
typedef typename Proxies::Impulse     Impulse;

enum class Type
{
    hard,
    soft
};

template <Uint32 dimension_, Type t>
struct SolverData
{
    static constexpr Uint32 dimension = dimension_;
    static constexpr Type   type      = t;


    Value<dimension> lambda{};
    Value<dimension> prevLambda{};

    Jacobian<dimension> J{};

    Value<dimension> bias{};
};

template <Uint32 dimension_>
struct SolverData<dimension_, Type::soft>
    : public SolverData<dimension_, Type::hard>
{
    static constexpr Type type = Type::soft;

    std::array<ConstraintSoftness, dimension_> softness{};

    Value<dimension_> damping{};
};


template <class SolverData_, ConstraintFunction F>
EffectiveMass<SolverData_::dimension>
computeEffectiveMass(SolverData_& data, const Proxies& proxies, const F& f)
{
    data.J = f.jacobian(proxies);
    return data.J * proxies.inverseMass() * transpose(data.J);
}

template <class SolverData_, ConstraintFunction F>
EffectiveMass<SolverData_::dimension>
initSolve(SolverData_& data, const Proxies& proxies, const F& f, float dt)
{
    static_assert(SolverData_::dimension == F::dimension, "");

    data.prevLambda = data.lambda;
    data.lambda     = Value<F::dimension>{};

    EffectiveMass<F::dimension> effectiveMass = computeEffectiveMass(
        data, proxies, f
    );

    data.bias = f.bias(proxies);

    if constexpr (SolverData_::type == Type::soft)
    {
        Value<F::dimension> restitution;
        for (Uint32 i = 0; i < F::dimension; ++i)
        {
            ConstraintSoftness::Feedbacks feedback = data.softness[i].getFeedbacks(
                1.f / effectiveMass(i, i), dt
            );

            data.damping[i] = feedback.gamma / dt;
            restitution[i]  = feedback.beta / dt;
        }

        Value<F::dimension> C = f.eval(proxies);
        data.bias += elementWiseMul(restitution, C);
    }

    return effectiveMass;
}

template <class SolverData_>
EffectiveMass<SolverData_::dimension>
applyDampingMass(const SolverData_& data, const EffectiveMass<SolverData_::dimension>& effMass)
{
    if constexpr (SolverData_::type == Type::soft)
        return effMass
               + EffectiveMass<SolverData_::dimension>::diagonal(data.damping);
    else
        return effMass;
}

template <class SolverData_>
Value<SolverData_::dimension>
velocityError(const SolverData_& data, const Proxies& proxies)
{
    return data.J * proxies.velocity();
}

template <class SolverData_>
Value<SolverData_::dimension>
velocityBias(const SolverData_& data, bool withDamping)
{
    if constexpr (SolverData_::type == Type::soft)
    {
        if (withDamping)
            return data.bias + elementWiseMul(data.damping, data.lambda);
    }

    return data.bias;
}

template <class SolverData_>
void applyImpulse(const SolverData_& data, Proxies& proxies, Value<SolverData_::dimension> dLambda)
{
    Impulse P = transpose(data.J) * dLambda;
    proxies.applyImpulse(P);
}


template <class SolverData_>
void applyPositionCorrection(
    const SolverData_&            data,
    Proxies&                      proxies,
    Value<SolverData_::dimension> posLambda
)
{
    State dS = transpose(data.J) * posLambda;
    proxies.applyPositionCorrection(dS);
}

template <class SolverData_, ConstraintFunction F>
void incrementLambda(
    SolverData_&               data,
    Proxies&                   proxies,
    const F&                   f,
    float                      dt,
    const Value<F::dimension>& dLambda
)
{
    Value<F::dimension> nextLambda = f.clampLambda(data.lambda + dLambda, dt);
    setLambda(data, proxies, nextLambda);
}

// assumes properly clamped.
template <class SolverData_>
void setLambda(SolverData_& data, Proxies& proxies, const Value<SolverData_::dimension>& lambda)
{
    Value<SolverData_::dimension> oldLambda = data.lambda;
    data.lambda                             = lambda;

    applyImpulse(data, proxies, data.lambda - oldLambda);
}

template <class SolverData_>
void warmstart(SolverData_& data, Proxies& proxies)
{
    setLambda(data, proxies, data.prevLambda);
}

} // namespace constraint


template <ConstraintFunction F_, constraint::Type type = constraint::Type::hard>
class EqualitySolver
{
public:

    typedef F_ F;

    typedef constraint::SolverData<F::dimension, type> SolverData;

    typedef constraint::Value<F::dimension>         Value;
    typedef constraint::EffectiveMass<F::dimension> EffectiveMass;

    typedef Solver<float, F::dimension> MatrixSolver;


    EqualitySolver() = default;

    void initSolve(const Proxies& proxies, const F& f, float dt)
    {
        solver_ = initSolve(data_, proxies, f, dt);
    }

    void warmstart(Proxies& proxies) { warmstart(data_, proxies); }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        solveVelocity(solver_, data_, proxies, f, dt);
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        solvePosition(data_, proxies, f);
    }

    const SolverData& data() const { return data_; }
    SolverData&       data() { return data_; }


    ////////////////////////////////////////////////////////////
    // Static methods to share implementation
    ////////////////////////////////////////////////////////////

    static MatrixSolver
    initSolve(SolverData& data, const Proxies& proxies, const F& f, float dt)
    {
        EffectiveMass effMass = constraint::initSolve(data, proxies, f, dt);
        MatrixSolver  solver{constraint::applyDampingMass(data, effMass)};
        SIMU_ASSERT(solver.isValid(), "Constraint cannot be solved");
        return solver;
    }

    static void warmstart(SolverData& data, Proxies& proxies)
    {
        constraint::warmstart(data, proxies);
    }

    static void solveVelocity(
        const MatrixSolver& solver,
        SolverData&         data,
        Proxies&            proxies,
        const F&            f,
        float               dt
    )
    {
        auto err = constraint::velocityError(data, proxies)
                   + constraint::velocityBias(data, true);

        auto dLambda = solver.solve(-err);
        constraint::incrementLambda(data, proxies, f, dt, dLambda);
    }

    static void solvePosition(SolverData& data, Proxies& proxies, const F& f)
    {
        Value error = f.eval(proxies);

        MatrixSolver solver{constraint::computeEffectiveMass(data, proxies, f)};

        Value posLambda = solver.solve(-error);
        posLambda       = f.clampPositionLambda(posLambda);
        constraint::applyPositionCorrection(data, proxies, posLambda);
    }


private:

    MatrixSolver solver_{EffectiveMass{}};
    SolverData   data_{};
};


template <ConstraintFunction F_, constraint::Type type = constraint::Type::hard>
class InequalitySolver
{
public:

    typedef F_ F;

    typedef constraint::SolverData<F::dimension, type> SolverData;

    typedef constraint::Value<F::dimension>         Value;
    typedef constraint::EffectiveMass<F::dimension> EffectiveMass;


    InequalitySolver() = default;


    void initSolve(const Proxies& proxies, const F& f, float dt)
    {
        effectiveMass_ = initSolve(data_, proxies, f, dt);
    }

    void warmstart(Proxies& proxies) { warmstart(data_, proxies); }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        solveVelocity(effectiveMass_, data_, proxies, f, dt);
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        solvePosition(data_, proxies, f);
    }

    const SolverData& data() const { return data_; }
    SolverData&       data() { return data_; }


    ////////////////////////////////////////////////////////////
    // Static methods to share implementation
    ////////////////////////////////////////////////////////////

    static EffectiveMass
    initSolve(SolverData& data, const Proxies& proxies, const F& f, float dt)
    {
        return constraint::initSolve(data, proxies, f, dt);
    }

    static void warmstart(SolverData& data, Proxies& proxies)
    {
        constraint::warmstart(data, proxies);
    }

    static void solveVelocity(
        const EffectiveMass& effMass,
        SolverData&          data,
        Proxies&             proxies,
        const F&             f,
        float                dt
    )
    {
        Value error = constraint::velocityError(data, proxies)
                      + constraint::velocityBias(data, false);

        Value correctedError = effMass * data.lambda;

        Value lambda = solveInequalities(
            constraint::applyDampingMass(data, effMass),
            -error + correctedError,
            [&](Value lambda) { return f.clampLambda(lambda, dt); },
            data.lambda
        );

        constraint::setLambda(data, proxies, lambda);
    }

    static void solvePosition(SolverData& data, Proxies& proxies, const F& f)
    {
        Value error = f.eval(proxies);

        EffectiveMass effMass = constraint::computeEffectiveMass(data, proxies, f);

        Value posLambda = solveInequalities(effMass, -error, [&](Value lambda) {
            return f.clampPositionLambda(lambda);
        });

        constraint::applyPositionCorrection(data, proxies, posLambda);
    }


private:

    EffectiveMass effectiveMass_{};
    SolverData    data_{};
};


namespace details
{

template <class F>
struct WrappedFunc
{
    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename F::Value    Value;
    typedef typename F::Jacobian Jacobian;

    WrappedFunc(const F& f, float sign, Value limit)
        : f{f}, sign{sign}, limit{limit}
    {
    }

    Value eval(const Proxies& proxies) const
    {
        return sign * (f.eval(proxies) - limit);
    }

    Jacobian jacobian(const Proxies& proxies) const
    {
        return sign * f.jacobian(proxies);
    }

    Value clampLambda(const Value& lambda, float dt) const
    {
        return sign * f.clampLambda(sign * lambda, dt);
    }

    Value clampPositionLambda(const Value& lambda) const
    {
        return sign * f.clampPositionLambda(sign * lambda);
    }

    Value bias(const Proxies& proxies) const { return sign * f.bias(proxies); }

    const F& f;
    float    sign;
    Value    limit;
};

} // namespace details


////////////////////////////////////////////////////////////
/// \brief Adds upper and lower limits to a constraint
///
/// Given a constraint of the form C(s): f(s) - a = 0
/// where f(s) is a function depending on the state of the bodies and a is a constant,
///
/// defines the constraints:
///     lower limit: CL(s):  C(s) - L >= 0   (f(s) >= a + L)
///     upper limit: CU(s): -C(s) + U >= 0   (f(s) <= a + U)
/// with L and U constant offsets from a, L <= U.
///
/// If L == U, then the equality constraint CE(s): f(s) - a - L = 0 is applied
///
/// This is not typically used with constraints of more than one dimension.
///
////////////////////////////////////////////////////////////
template <ConstraintFunction F_, constraint::Type type = constraint::Type::hard>
class LimitsSolver
{
public:

    typedef F_                       F;
    typedef details::WrappedFunc<F_> WF;

    typedef constraint::SolverData<F::dimension, type> SolverData;

    typedef constraint::Value<F::dimension>         Value;
    typedef constraint::EffectiveMass<F::dimension> EffectiveMass;

    LimitsSolver() = default;

    void setLowerLimit(std::optional<Value> L)
    {
        L_ = L;
        assertLimits();
    }
    void setUpperLimit(std::optional<Value> U)
    {
        U_ = U;
        assertLimits();
    }

    bool isActive(const Proxies& proxies, const F& f) const
    {
        return nextFunc(proxies, f) != Func::off;
    }


    void initSolve(const Proxies& proxies, const F& f, float dt)
    {
        Func prev     = funcType_;
        funcType_     = nextFunc(proxies, f);
        canWarmstart_ = (funcType_ == prev) && (funcType_ != Func::off);

        WF wf{wrapFunc(f)};

        if (funcType_ == Func::equality)
            solver_ = Equality::initSolve(data_, proxies, wf, dt);
        else if (funcType_ == Func::lower || funcType_ == Func::upper)
            effectiveMass_ = Inequality::initSolve(data_, proxies, wf, dt);
    }

    void warmstart(Proxies& proxies)
    {
        if (canWarmstart_)
        {
            constraint::warmstart(data_, proxies);
        }
    }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        // TODO: Bias may be incorrect, both for function and solver's restitutino/damping.
        //      (currently untested)

        WF wf{wrapFunc(f)};

        if (funcType_ == Func::equality)
            Equality::solveVelocity(solver_, data_, proxies, wf, dt);
        else if (funcType_ == Func::lower || funcType_ == Func::upper)
            Inequality::solveVelocity(effectiveMass_, data_, proxies, wf, dt);
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        WF wf{wrapFunc(f)};

        if (funcType_ == Func::equality)
            Equality::solvePosition(data_, proxies, wf);
        else if (funcType_ == Func::lower || funcType_ == Func::upper)
            Inequality::solvePosition(data_, proxies, wf);
    }

    const SolverData& data() const { return data_; }
    SolverData&       data() { return data_; }


private:

    typedef EqualitySolver<WF, type>   Equality;
    typedef InequalitySolver<WF, type> Inequality;


    void assertLimits() const
    {
        if (L_.has_value() && U_.has_value())
            SIMU_ASSERT(all(L_.value() <= U_.value()), "Invalid limits");
    }

    enum class Func
    {
        lower,
        upper,
        equality,
        off
    };

    WF wrapFunc(const F& f) const
    {
        float sign{};
        Value limit{};

        switch (funcType_)
        {
            case Func::lower:
                sign  = 1.f;
                limit = L_.value();
                break;
            case Func::upper:
                sign  = -1.f;
                limit = U_.value();
                break;
            case Func::equality:
                sign  = 1.f;
                limit = L_.value_or(Value{});
                break;
            default: break;
        }

        return WF{f, sign, limit};
    }

    Func nextFunc(const Proxies& proxies, const F& f) const
    {
        if (!L_.has_value() && !U_.has_value())
            return Func::equality;

        if (L_.has_value() && U_.has_value() && all(L_.value() == U_.value()))
            return Func::equality;

        Value C = f.eval(proxies);
        if (L_.has_value() && all(C - L_.value() <= Value::filled(0.f)))
            return Func::lower;
        else if (U_.has_value() && all(-C + U_.value() <= Value::filled(0.f)))
            return Func::upper;

        return Func::off;
    }

    SolverData data_{};

    Func funcType_ = Func::off;

    std::optional<Value> L_ = std::nullopt;
    std::optional<Value> U_ = std::nullopt;

    union
    {
        EffectiveMass                   effectiveMass_{};
        typename Equality::MatrixSolver solver_;
    };

    bool canWarmstart_ = false;
};

} // namespace simu
