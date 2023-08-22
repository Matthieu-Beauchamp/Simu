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
using JacobianMatrix = Matrix<float, dimension, 6>;

template <Uint32 dimension>
using EffectiveMass = Matrix<float, dimension, dimension>;

typedef typename Proxies::Mass    Mass;
typedef typename Proxies::MassVec MassVec;

typedef typename Proxies::State       State;
typedef typename Proxies::VelocityVec VelocityVec;
typedef typename Proxies::Impulse     Impulse;

} // namespace constraint


template <Uint32 dimension>
class Lambda
{
public:

    typedef constraint::Value<dimension> Val;

    Lambda() = default;

    void newStep()
    {
        previous_    = accumulated_;
        accumulated_ = Val{};
    }

    Val getAccumulated() const { return accumulated_; }
    Val getPrevious() const { return previous_; }

    template <Callable<Val(const Val&)> Clamp>
    Val increment(const Val& dLambda, const Clamp& clampFunc)
    {
        return setAccumulated(clampFunc(accumulated_ + dLambda));
    }

    // assumes accumulated is clamped to valid range.
    // Returns the change in accumulated lambda.
    Val setAccumulated(const Val& accumulated)
    {
        Val dLambda  = accumulated - accumulated_;
        accumulated_ = accumulated;
        return dLambda;
    }

    /// \param damping The Gamma / dt vector
    Val forceFeedbackBias(const Val& damping)
    {
        return elementWiseMul(accumulated_, damping);
    }

private:

    Val accumulated_;
    Val previous_;
};


template <Uint32 dimension>
class Jacobian : public constraint::JacobianMatrix<dimension>
{
public:

    typedef constraint::JacobianMatrix<dimension> Base;

    typedef constraint::Value<dimension> Value;

    typedef constraint::EffectiveMass<dimension> EffectiveMass;

    typedef constraint::Mass    Mass;
    typedef constraint::MassVec MassVec;

    typedef constraint::VelocityVec Velocity;
    typedef constraint::Impulse     Impulse;


    Jacobian() = default;

    using Base::operator=;

    /// This is the non inverted J * M^-1 * J^T
    EffectiveMass computeEffectiveMass(const MassVec& invMassVec) const
    {
        return *this * Mass::diagonal(invMassVec) * transpose(*this);
    }

    /// This is the non inverted J * M^-1 * J^T + E*Gamma/dt
    /// \param damping The diagonal of E*Gamma / dt
    EffectiveMass
    computeEffectiveMassWithDamping(const MassVec& invMassVec, const Value& damping) const
    {
        return computeEffectiveMass(invMassVec) + EffectiveMass::diagonal(damping);
    }

    Value velocityError(const Velocity& velocity) const
    {
        return *this * velocity;
    }

    Impulse impulse(const Value& lambda) { return transpose(*this) * lambda; }
};


template <ConstraintFunction F_>
class ConstraintSolverBase
{
public:

    typedef F_ F;


    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef constraint::Value<dimension>         Value;
    typedef constraint::EffectiveMass<dimension> EffectiveMass;

    ConstraintSolverBase() = default;

    EffectiveMass initBase(const Proxies& proxies, const F& f, float dt)
    {
        lambda_.newStep();
        EffectiveMass effectiveMass = computeEffectiveMass(proxies, f);

        Value restitution;
        for (Uint32 i = 0; i < dimension; ++i)
        {
            ConstraintSoftness::Feedbacks feedback = softness_[i].getFeedbacks(
                1.f / effectiveMass(i, i), dt
            );

            damping_[i]    = feedback.gamma / dt;
            restitution[i] = feedback.beta / dt;
        }

        Value C = f.eval(proxies);

        bias_ = f.bias(proxies);
        bias_ += elementWiseMul(restitution, C);

        return effectiveMass;
    }

    void warmstartDefault(Proxies& proxies)
    {
        Value dLambda         = lambda_.setAccumulated(lambda_.getPrevious());
        constraint::Impulse P = J_.impulse(dLambda);
        proxies.applyImpulse(P);
    }

    /// Updates the Jacobian and returns J * M^-1 * J^T
    EffectiveMass computeEffectiveMass(const Proxies& proxies, const F& f)
    {
        J_ = f.jacobian(proxies);
        return J_.computeEffectiveMass(proxies.invMassVec());
    }

    EffectiveMass applyDampingMass(const EffectiveMass& effectiveMass)
    {
        return effectiveMass + EffectiveMass::diagonal(damping_);
    }

    Value computeVelocityError(const Proxies& proxies) const
    {
        return J_.velocityError(proxies.velocity());
    }

    Value bias(bool withForceFeedback = true) const
    {
        if (withForceFeedback)
            return bias_ + elementWiseMul(damping_, lambda_.getAccumulated());

        return bias_;
    }

    void applyImpulse(Proxies& proxies, Value dLambda)
    {
        proxies.applyImpulse(J_.impulse(dLambda));
    }

    void applyPositionCorrection(Proxies& proxies, Value posLambda)
    {
        proxies.applyPositionCorrection(J_.impulse(posLambda));
    }

    void
    incrementLambda(Proxies& proxies, const F& f, float dt, const Value& dLambda)
    {
        Value change = lambda_.increment(dLambda, [&](const Value& lambda) {
            return f.clampLambda(lambda, dt);
        });

        this->applyImpulse(proxies, change);
    }

    // assumes properly clamped.
    void setLambda(Proxies& proxies, const Value& lambda)
    {
        Value change = lambda_.setAccumulated(lambda);
        this->applyImpulse(proxies, change);
    }

    const Lambda<dimension>& lambda() const { return lambda_; }
    Lambda<dimension>&       lambda() { return lambda_; }

    const Jacobian<dimension>& jacobian() const { return J_; }

    const auto& softness() const { return softness_; }
    auto&       softness() { return softness_; }

private:

    Lambda<dimension>   lambda_{};
    Jacobian<dimension> J_{};
    Value               bias_{};

    std::array<ConstraintSoftness, dimension> softness_{};

    Value damping_{};
};


template <ConstraintFunction F>
class EqualitySolver : public ConstraintSolverBase<F>
{
public:

    typedef ConstraintSolverBase<F> Base;

    typedef constraint::Value<Base::dimension>         Value;
    typedef constraint::EffectiveMass<Base::dimension> EffectiveMass;
    typedef Solver<float, Base::dimension>             Solver_;


    EqualitySolver() = default;

    void initSolve(const Proxies& proxies, const F& f, float dt)
    {
        EffectiveMass effMass = this->initBase(proxies, f, dt);
        solver_               = Solver{this->applyDampingMass(effMass)};
        SIMU_ASSERT(solver_.isValid(), "Constraint cannot be solved");
    }

    void warmstart(Proxies& proxies) { this->warmstartDefault(proxies); }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        auto err     = this->computeVelocityError(proxies) + this->bias();
        auto dLambda = solver_.solve(-err);
        this->incrementLambda(proxies, f, dt, dLambda);
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        Value error = f.eval(proxies);

        solver_ = Solver_{this->computeEffectiveMass(proxies, f)};

        Value posLambda = f.clampPositionLambda(solver_.solve(-error));
        this->applyPositionCorrection(proxies, posLambda);
    }


private:

    Solver_ solver_{EffectiveMass{}};
};


template <ConstraintFunction F>
class InequalitySolver : public ConstraintSolverBase<F>
{
public:

    typedef ConstraintSolverBase<F> Base;

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    InequalitySolver() = default;

    void initSolve(const Proxies& proxies, const F& f, float dt)
    {
        effectiveMass_ = this->initBase(proxies, f, dt);
    }

    void warmstart(Proxies& proxies) { this->warmstartDefault(proxies); }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        Value accLambda = this->lambda().getAccumulated();
        Value error = this->computeVelocityError(proxies) + this->bias(false);
        Value correctedError = effectiveMass_ * accLambda;

        Value lambda = solveInequalities(
            this->applyDampingMass(effectiveMass_),
            -error + correctedError,
            [&](Value lambda) { return f.clampLambda(lambda, dt); },
            accLambda
        );

        this->setLambda(proxies, lambda);
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        Value error = f.eval(proxies);

        effectiveMass_ = this->computeEffectiveMass(proxies, f);

        Value posLambda = solveInequalities(effectiveMass_, -error, [&](Value lambda) {
            return f.clampPositionLambda(lambda);
        });

        this->applyPositionCorrection(posLambda);
    }


private:

    KMatrix effectiveMass_;
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
template <ConstraintFunction F_>
class LimitsSolver : public ConstraintSolverBase<details::WrappedFunc<F_>>
{
public:

    typedef details::WrappedFunc<F_> WF;
    typedef F_                       F; // for concept

    typedef ConstraintSolverBase<WF> Base;

    static constexpr Uint32 nBodies   = F_::nBodies;
    static constexpr Uint32 dimension = F_::dimension;

    typedef typename Base::Value Value;

    typedef typename Base::EffectiveMass   EffectiveMass;
    typedef Solver<float, Base::dimension> KSolver;

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

        EffectiveMass effMass = this->initBase(proxies, wrapFunc(f), dt);

        if (funcType_ == Func::equality)
            solver_ = KSolver{this->applyDampingMass(effMass)};
        else if (funcType_ == Func::lower || funcType_ == Func::upper)
            effectiveMass_ = effMass;
    }

    void warmstart(Proxies& proxies)
    {
        if (canWarmstart_)
        {
            this->warmstartDefault(proxies);
        }
    }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        // TODO: Bias may be incorrect, both for function and solver's restitutino/damping.
        //      (currently untested)

        WF wf{wrapFunc(f)};

        switch (funcType_)
        {
            case Func::equality:
            {
                Value err = -(this->computeVelocityError(proxies) + this->bias());
                this->incrementLambda(proxies, wf, dt, solver_.solve(err));
                break;
            }
            case Func::upper: [[fallthrough]];
            case Func::lower:
            {
                Value accLambda = this->lambda().getAccumulated();
                Value error     = this->computeVelocityError(proxies)
                              + this->bias(false);
                Value correctedError = effectiveMass_ * accLambda;

                Value lambda = solveInequalities(
                    this->applyDampingMass(effectiveMass_),
                    -error + correctedError,
                    [&](Value lambda) {
                        return std::max(
                            wf.clampLambda(lambda, dt), Value::filled(0.f)
                        );
                    },
                    accLambda
                );

                this->setLambda(proxies, lambda);
                break;
            }

            default: break;
        }
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        WF wf{wrapFunc(f)};

        Value C = wf.eval(proxies);

        EffectiveMass effMass = this->computeEffectiveMass(proxies, wf);

        Value posLambda{};
        switch (funcType_)
        {
            case Func::equality:
            {
                posLambda = wf.clampPositionLambda(solve(effMass, -C));
                break;
            }
            case Func::lower: [[fallthrough]];
            case Func::upper:
            {
                if (all(C >= Value::filled(0.f)))
                    return;

                posLambda = solveInequalities(effMass, -C, [&](Value lambda) {
                    return std::max(
                        wf.clampPositionLambda(lambda), Value::filled(0.f)
                    );
                });

                break;
            }

            default: break;
        }

        this->applyPositionCorrection(proxies, posLambda);
    }

private:

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

    Func funcType_ = Func::off;

    std::optional<Value> L_ = std::nullopt;
    std::optional<Value> U_ = std::nullopt;

    union
    {
        EffectiveMass effectiveMass_{};
        KSolver       solver_;
    };

    bool canWarmstart_ = false;
};

} // namespace simu
