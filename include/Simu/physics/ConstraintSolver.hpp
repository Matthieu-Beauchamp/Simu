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

#include "Simu/physics/ConstraintInterfaces.hpp"

namespace simu
{

class Body;


template <ConstraintFunction F_>
class ConstraintSolverBase
{
public:

    typedef F_ F;

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename F::Value    Value;
    typedef typename F::Jacobian Jacobian;

    typedef typename Proxies::State       State;
    typedef typename Proxies::VelocityVec VelocityVec;
    typedef typename Proxies::Impulse     Impulse;

    typedef Matrix<float, dimension, dimension> KMatrix;


    ConstraintSolverBase(Bodies /* bodies */, const F& /* f */) {}

    Value&       restitution() { return restitution_; }
    const Value& restitution() const { return restitution_; }

    Value&       damping() { return damping_; }
    const Value& damping() const { return damping_; }

    void warmstartDefault(Proxies& proxies, const F& f, float dt)
    {
        Value lambda = getAccumulatedLambda();
        setLambdaHint(Value::filled(0.f));

        updateLambda(proxies, f, dt, lambda);
    }

    KMatrix
    computeEffectiveMass(const Proxies& proxies, const F& f, bool addDamping)
    {
        J_                    = f.jacobian(proxies);
        KMatrix effectiveMass = J_ * proxies.inverseMass() * transpose(J_);

        if (addDamping)
            effectiveMass += KMatrix::diagonal(damping());

        return effectiveMass;
    }

    Value
    computeRhs(const Proxies& proxies, const F& f, float dt, bool addDamping) const
    {
        Value error                  = J_ * proxies.velocity();
        Value bias                   = f.bias(proxies);
        Value baumgarteStabilization = KMatrix::diagonal(restitution())
                                       * f.eval(proxies) / dt;

        Value previousDamping = addDamping
                                    ? KMatrix::diagonal(damping()) * lambda_
                                    : Value{};

        return -(error + bias + baumgarteStabilization + previousDamping);
    }

    void updateLambda(Proxies& proxies, const F& f, float dt, Value dLambda)
    {
        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        proxies.applyImpulse(impulse(J_, lambda_ - oldLambda));
    }

    static Impulse impulse(const Jacobian& J, const Value& lambda)
    {
        return transpose(J) * lambda;
    }

    Jacobian getJacobian() const { return J_; }
    Value    getAccumulatedLambda() const { return lambda_; }
    void     setLambdaHint(Value lambda) { lambda_ = lambda; }

private:

    Value    lambda_{};
    Jacobian J_{};

    Value restitution_{};
    Value damping_{};
};


template <ConstraintFunction F>
class EqualitySolver : public ConstraintSolverBase<F>
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


    EqualitySolver(const Bodies& bodies, const F& f)
        : Base{bodies, f}, solver_{KMatrix{}}
    {
    }

    void initSolve(const Proxies& proxies, const F& f)
    {
        solver_ = Solver{this->computeEffectiveMass(proxies, f, true)};
        SIMU_ASSERT(solver_.isValid(), "Constraint cannot be solved");
    }

    void warmstart(Proxies& proxies, const F& f, float dt)
    {
        this->warmstartDefault(proxies, f, dt);
    }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        auto err     = this->computeRhs(proxies, f, dt, true);
        auto dLambda = solver_.solve(err);
        this->updateLambda(proxies, f, dt, dLambda);
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        Value error = f.eval(proxies);

        Jacobian J = f.jacobian(proxies);
        solver_    = KSolver{this->computeEffectiveMass(proxies, f, false)};

        Value posLambda          = f.clampPositionLambda(solver_.solve(-error));
        State positionCorrection = transpose(J) * posLambda;

        proxies.applyPositionCorrection(positionCorrection);
    }


private:

    KSolver solver_;
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


    InequalitySolver(const Bodies& bodies, const F& f) : Base{bodies, f} {}

    void initSolve(const Proxies& proxies, const F& f)
    {
        effectiveMass_ = this->computeEffectiveMass(proxies, f, true);
    }

    void warmstart(Proxies& proxies, const F& f, float dt)
    {
        this->warmstartDefault(proxies, f, dt);
    }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        // TODO: Incorrect! must not use damping
        Value correctedError = effectiveMass_ * this->getAccumulatedLambda();

        Value lambda = solveInequalities(
            effectiveMass_,
            this->computeRhs(proxies, f, dt, false) + correctedError,
            [=, &f](Value lambda) { return f.clampLambda(lambda, dt); },
            this->getAccumulatedLambda()
        );

        this->updateLambda(proxies, f, dt, lambda - this->getAccumulatedLambda());
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        Value error = f.eval(proxies);

        Jacobian J     = f.jacobian(proxies);
        effectiveMass_ = J * proxies.inverseMass() * transpose(J);

        // TODO: Inequality solver should be using:
        // Value posLambda = solveInequalities(
        //     effectiveMass_,
        //     -error,
        //     [&](Value lambda) { return f.clampPositionLambda(lambda); }
        // );
        //
        // but it's too inaccurate for contacts

        Solver<float, F::dimension> s{effectiveMass_};
        if (!s.isValid())
            return; // Jacobians are parallel, TODO:

        Value posLambda = s.solve(-error);
        posLambda       = f.clampPositionLambda(posLambda);

        State positionCorrection = transpose(J) * posLambda;

        proxies.applyPositionCorrection(positionCorrection);
    }


private:

    KMatrix effectiveMass_;
};


////////////////////////////////////////////////////////////
/// \brief Adds upper and lower limits to a constraint
///
/// Given a constraint of the form C(s): f(s) - a = 0
/// where f(s) is a function depending on the state of the bodies and a is a constant,
///
/// defines the constraints:
///     lower limit: CL(s):  C(s) - L >= 0   (f(s) >= a + L)
///     upper limit: CU(s): -C(s) + U >= 0   (f(s) <= a + H)
/// with L and U constant offsets from a, L <= U.
///
/// If L == U, then the equality constraint CE(s): f(s) - a - L = 0 is applied
///
/// This is not typically used with constraints of more than one dimension.
///
////////////////////////////////////////////////////////////
template <ConstraintFunction F>
class LimitsSolver : public ConstraintSolverBase<F>
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

    LimitsSolver(Bodies bodies, const F& f) : Base{bodies, f} {}

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


    void initSolve(const Proxies& proxies, const F& f)
    {
        Func prev     = func_;
        func_         = nextFunc(proxies, f);
        canWarmstart_ = (func_ == prev) && (func_ != Func::off);

        KMatrix effMass = this->computeEffectiveMass(proxies, f, true);

        if (func_ == Func::equality)
            solver_ = KSolver{effMass};
        else if (func_ == Func::lower || func_ == Func::upper)
            effectiveMass_ = effMass;
    }

    void warmstart(Proxies& proxies, const F& f, float dt)
    {
        if (canWarmstart_)
        {
            if (func_ == Func::upper)
            {
                proxies.applyImpulse(this->impulse(
                    this->getJacobian(),
                    -this->getAccumulatedLambda()
                ));
            }
            else
            {
                this->warmstartDefault(proxies, f, dt);
            }
        }
        else
        {
            this->setLambdaHint(Value::filled(0.f));
        }
    }

    void solveVelocity(Proxies& proxies, const F& f, float dt)
    {
        Value err = rhs(proxies, f, dt);

        switch (func_)
        {
            case Func::equality:
            {
                this->updateLambda(proxies, f, dt, solver_.solve(err));
                break;
            }
            case Func::lower:
            {
                Value lambda = solveInequalities(
                    effectiveMass_,
                    err,
                    [=, &f](Value lambda) {
                        return std::max(
                            f.clampLambda(lambda, dt),
                            Value::filled(0.f)
                        );
                    },
                    this->getAccumulatedLambda()
                );

                this->updateLambda(
                    proxies,
                    f,
                    dt,
                    lambda - this->getAccumulatedLambda()
                );
                break;
            }
            case Func::upper:
            {
                Value lambda = solveInequalities(
                    effectiveMass_,
                    err,
                    [=, &f](Value lambda) {
                        return std::max(
                            -f.clampLambda(-lambda, dt),
                            Value::filled(0.f)
                        );
                    },
                    this->getAccumulatedLambda()
                );

                proxies.applyImpulse(this->impulse(
                    this->getJacobian(),
                    -(lambda - this->getAccumulatedLambda())
                ));
                this->setLambdaHint(lambda);
                break;
            }
            default: break;
        }
    }

    void solvePosition(Proxies& proxies, const F& f)
    {
        Value    C       = f.eval(proxies);
        KMatrix  effMass = this->computeEffectiveMass(proxies, f, false);
        Jacobian J       = this->getJacobian();

        Value posLambda{};
        switch (nextFunc(proxies, f))
        {
            case Func::equality:
            {
                posLambda = f.clampPositionLambda(
                    solve(effMass, -(C - L_.value()))
                );
                break;
            }
            case Func::lower:
            {
                posLambda = solveInequalities(
                    effMass,
                    -(C - L_.value()),
                    [&f](Value lambda) {
                        return std::max(
                            f.clampPositionLambda(lambda),
                            Value::filled(0.f)
                        );
                    }
                );

                break;
            }
            case Func::upper:
            {
                posLambda = -solveInequalities(
                    effMass,
                    C - U_.value(),
                    [&f](Value lambda) {
                        return std::max(
                            -f.clampPositionLambda(-lambda),
                            Value::filled(0.f)
                        );
                    }
                );
                break;
            }
            default: break;
        }

        State positionCorrection = transpose(J) * posLambda;

        proxies.applyPositionCorrection(positionCorrection);
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

    Value rhs(const Proxies& proxies, const F& f, float dt)
    {
        Jacobian J              = this->getJacobian();
        Value    error          = J * proxies.velocity();
        Value    bias           = f.bias(proxies);
        Value    baumgarteCoeff = KMatrix::diagonal(this->restitution()) / dt;

        Value C = f.eval(proxies);

        switch (func_)
        {
            case Func::equality:
            {
                if (L_.has_value())
                    C -= L_.value();

                Value previousDamping = KMatrix::diagonal(this->damping())
                                        * this->getAccumulatedLambda();
                return -(error + bias + baumgarteCoeff * C + previousDamping);
            }
            case Func::lower:
            {
                C -= L_.value();
                // TODO: Use effectiveMass.
                Value alreadyCorrected = J * proxies.inverseMass() * transpose(J)
                                         * this->getAccumulatedLambda();
                return -(error - alreadyCorrected + bias + baumgarteCoeff * C);
            }
            case Func::upper:
            {
                // TODO: Use effectiveMass.
                Value alreadyCorrected = J * proxies.inverseMass() * transpose(J)
                                         * -this->getAccumulatedLambda();
                return error - alreadyCorrected + bias
                       + baumgarteCoeff * (C - U_.value());
            }
            default: return Value{};
        }
    }

    Func func_ = Func::off;

    std::optional<Value> L_ = std::nullopt;
    std::optional<Value> U_ = std::nullopt;

    union
    {
        KMatrix effectiveMass_;
        KSolver solver_;
    };

    bool canWarmstart_ = false;
};

} // namespace simu
