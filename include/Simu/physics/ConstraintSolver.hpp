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

    typedef typename Bodies<nBodies>::State    State;
    typedef typename Bodies<nBodies>::Velocity Velocity;
    typedef typename Bodies<nBodies>::Impulse  Impulse;

    typedef Matrix<float, dimension, dimension> KMatrix;


    ConstraintSolverBase(Bodies<nBodies> /* bodies */, const F& /* f */) {}

    Value&       restitution() { return restitution_; }
    const Value& restitution() const { return restitution_; }

    Value&       damping() { return damping_; }
    const Value& damping() const { return damping_; }

    KMatrix
    computeEffectiveMass(const Bodies<nBodies>& bodies, const F& f, bool addDamping)
    {
        J_                    = f.jacobian(bodies);
        KMatrix effectiveMass = J_ * bodies.inverseMass() * transpose(J_);

        if (addDamping)
            effectiveMass += KMatrix::diagonal(damping());

        return effectiveMass;
    }

    Value
    computeRhs(const Bodies<nBodies>& bodies, const F& f, float dt, bool addDamping) const
    {
        Value error = J_ * bodies.velocity();
        Value bias  = f.bias(bodies);
        Value baumgarteStabilization
            = KMatrix::diagonal(restitution()) * f.eval(bodies) / dt;

        Value previousDamping
            = addDamping ? KMatrix::diagonal(damping()) * lambda_ : Value{};

        return -(error + bias + baumgarteStabilization + previousDamping);
    }

    void
    updateLambda(Bodies<nBodies>& bodies, const F& f, float dt, Value dLambda)
    {
        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        bodies.applyImpulse(impulse(J_, lambda_ - oldLambda));
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


    EqualitySolver(const Bodies<nBodies>& bodies, const F& f)
        : Base{bodies, f}, solver_{KMatrix{}}
    {
        initSolver(bodies, f);
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        this->updateLambda(
            bodies,
            f,
            dt,
            solver_.solve(this->computeRhs(bodies, f, dt, true))
        );
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J = f.jacobian(bodies);
        solver_    = KSolver{this->computeEffectiveMass(bodies, f, false)};

        Value posLambda = f.clampPositionLambda(solver_.solve(-error));
        State positionCorrection
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
    }


private:

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
    {
        solver_ = Solver{this->computeEffectiveMass(bodies, f, true)};
        SIMU_ASSERT(solver_.isValid(), "Constraint cannot be solved");
    }

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


    InequalitySolver(const Bodies<nBodies>& bodies, const F& f)
        : Base{bodies, f}
    {
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        // TODO: Incorrect! must not use damping
        Value correctedError = effectiveMass_ * this->getAccumulatedLambda();

        Value lambda = solveInequalities(
            effectiveMass_,
            this->computeRhs(bodies, f, dt, false) + correctedError,
            [=, &f](Value lambda) { return f.clampLambda(lambda, dt); },
            this->getAccumulatedLambda()
        );

        this->updateLambda(bodies, f, dt, lambda - this->getAccumulatedLambda());
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J     = f.jacobian(bodies);
        effectiveMass_ = J * bodies.inverseMass() * transpose(J);

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

        State positionCorrection
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
    }


private:

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
    {
        effectiveMass_ = this->computeEffectiveMass(bodies, f, true);
    }

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

    LimitsSolver(Bodies<nBodies> bodies, const F& f) : Base{bodies, f} {}

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

    bool isActive(const Bodies<nBodies>& bodies, const F& f) const
    {
        return nextFunc(bodies, f) != Func::off;
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        Func prev = func_;
        func_     = nextFunc(bodies, f);
        initSolver(bodies, f);

        if (func_ == prev && func_ != Func::off)
        {
            Value lambda = this->getAccumulatedLambda();

            if (func_ == Func::upper)
            {
                bodies.applyImpulse(this->impulse(this->getJacobian(), -lambda));
            }
            else
            {
                this->setLambdaHint(Value::filled(0.f));
                this->updateLambda(bodies, f, dt, lambda);
            }
        }
        else
        {
            this->setLambdaHint(Value::filled(0.f));
        }
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        Value err = rhs(bodies, f, dt);

        switch (func_)
        {
            case Func::equality:
            {
                this->updateLambda(bodies, f, dt, solver_.solve(err));
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
                    bodies,
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

                bodies.applyImpulse(this->impulse(
                    this->getJacobian(),
                    -(lambda - this->getAccumulatedLambda())
                ));
                this->setLambdaHint(lambda);
                break;
            }
            default: break;
        }
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value    C       = f.eval(bodies);
        Jacobian J       = f.jacobian(bodies);
        KMatrix  effMass = this->computeEffectiveMass(bodies, f, false);

        Value posLambda;
        switch (nextFunc(bodies, f))
        {
            case Func::equality:
            {
                posLambda
                    = f.clampPositionLambda(solve(effMass, -(C - L_.value())));
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

        State positionCorrection
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
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

    Func nextFunc(const Bodies<nBodies>& bodies, const F& f) const
    {
        if (!L_.has_value() && !U_.has_value())
            return Func::equality;

        if (L_.has_value() && U_.has_value() && all(L_.value() == U_.value()))
            return Func::equality;

        Value C = f.eval(bodies);
        if (L_.has_value() && all(C - L_.value() <= Value::filled(0.f)))
            return Func::lower;
        else if (U_.has_value() && all(-C + U_.value() <= Value::filled(0.f)))
            return Func::upper;

        return Func::off;
    }

    Value rhs(const Bodies<nBodies>& bodies, const F& f, float dt)
    {
        Jacobian J              = this->getJacobian();
        Value    error          = J * bodies.velocity();
        Value    bias           = f.bias(bodies);
        Value    baumgarteCoeff = KMatrix::diagonal(this->restitution()) / dt;

        Value C = f.eval(bodies);

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
                Value alreadyCorrected = J * bodies.inverseMass() * transpose(J)
                                         * this->getAccumulatedLambda();
                return -(error - alreadyCorrected + bias + baumgarteCoeff * C);
            }
            case Func::upper:
            {
                Value alreadyCorrected = J * bodies.inverseMass() * transpose(J)
                                         * -this->getAccumulatedLambda();
                return error - alreadyCorrected + bias
                       + baumgarteCoeff * (C - U_.value());
            }
            default: return Value{};
        }
    }

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
    {
        KMatrix effMass = this->computeEffectiveMass(bodies, f, true);

        if (func_ == Func::equality)
            solver_ = KSolver{effMass};
        else if (func_ == Func::lower || func_ == Func::upper)
            effectiveMass_ = effMass;
    }

    Func func_ = Func::off;

    std::optional<Value> L_ = std::nullopt;
    std::optional<Value> U_ = std::nullopt;

    union
    {
        KMatrix effectiveMass_;
        KSolver solver_;
    };
};


namespace priv
{

// Uses a special form for velocities:
// K( (lambda + dLambda - (1+e)lambda) / (1+e) ) >= -Jv
template <ConstraintFunction F>
class ContactSolver : public ConstraintSolverBase<F>
{
public:

    static_assert(
        std::is_same_v<F, SingleContactFunction>
            || std::is_same_v<F, DoubleContactFunction>,
        "This is reserved for contact constraints."
    );

    typedef ConstraintSolverBase<F> Base;

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    ContactSolver(const Bodies<nBodies>& bodies, const F& f)
        : Base{bodies, f},
          restitutionCoefficient_{
            CombinableProperty{bodies[0]->material().bounciness, bodies[1]->material().bounciness}
          .value},
          maxPen_{CombinableProperty{bodies[0]->material().penetration, bodies[1]->material().penetration}
          .value}
    {
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        // auto error = this->computeRhs(bodies, f, dt, false);
        // auto initialGuess
        //     = -(restitutionCoefficient_ / (1.f + restitutionCoefficient_))
        //       * this->getAccumulatedLambda();

        // Value x = solveInequalities(
        //     effectiveMass_,
        //     error,
        //     [this](Value x) { return this->clampX(x); },
        //     initialGuess
        // );

        // Value dLambda = (1.f + restitutionCoefficient_) * x
        //                 + restitutionCoefficient_ * this->getAccumulatedLambda();

        // this->updateLambda(bodies, f, dt, dLambda);


        // computing the bounce from the relative normal velocities when the solver is initialized
        //
        // Box stacking is unstable for bouncy bodies.
        // Box2d uses a restitution threshold:
        //  if the relative velocity is lower than the treshold, then apply
        //  restitution.
        // This avoids the jittery bounces, allow stable contacts (that can sleep)

        Jacobian J     = this->getJacobian();
        Value    error = J * bodies.velocity() + bounce_;
        Value    baumgarte
            = KMatrix::diagonal(this->restitution()) * f.eval(bodies) / dt;
        Value alreadyComputed = effectiveMass_ * this->getAccumulatedLambda();

        Value lambda = solveInequalities(
            effectiveMass_,
            -(error - alreadyComputed + baumgarte),
            [=, &f](Value lambda) { return f.clampLambda(lambda, dt); },
            this->getAccumulatedLambda()
        );

        Value dLambda = lambda - this->getAccumulatedLambda();

        this->updateLambda(bodies, f, dt, dLambda);
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value error = f.eval(bodies);

        float beta          = 0.2f;
        Value maxPen        = Value::filled(maxPen_);
        Value maxCorrection = Value::filled(0.2f);

        // should be done based on the maximum error found over all contacts...
        //  (according to box2D)
        if (all(error > -3.f * maxPen))
            return;


        error = clamp(beta * (error + maxPen), -maxCorrection, Value{});

        Jacobian J     = f.jacobian(bodies);
        effectiveMass_ = J * bodies.inverseMass() * transpose(J);

        Value posLambda
            = solveInequalities(effectiveMass_, -error, [&f](Value posLambda) {
                  return f.clampPositionLambda(posLambda);
              });

        State positionCorrection
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
    }


private:

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
    {
        effectiveMass_ = this->computeEffectiveMass(bodies, f, false);
        bounce_
            = restitutionCoefficient_ * this->getJacobian() * bodies.velocity();
        bounce_ = std::min(Value::filled(0.f), bounce_);
    }

    typename F::Value clampX(typename F::Value x)
    {
        return std::max(x, -this->getAccumulatedLambda());
    }

    Value   bounce_;
    KMatrix effectiveMass_;

    float restitutionCoefficient_;
    float maxPen_;
};

} // namespace priv

} // namespace simu
