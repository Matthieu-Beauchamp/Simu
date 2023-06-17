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

#include <array>

#include "Simu/config.hpp"
#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/ContactManifold.hpp"
#include "Simu/math/BarycentricCoordinates.hpp"
#include "Simu/math/Gjk.hpp"

namespace simu
{

template <class T>
T clamp(T val, T min, T max)
{
    return std::min(std::max(val, min), max);
}

class Constraint : public PhysicsObject
{
public:

    ~Constraint() override = default;

    virtual bool isActive()          = 0;
    virtual void initSolve(float dt) = 0;
    virtual void solve(float dt)     = 0;
    virtual void commit()            = 0;
};


template <Uint32 nConstraints>
using ConstraintValue = Vector<float, nConstraints>;

template <Uint32 nConstraints, Uint32 nBodies>
using Jacobian = Matrix<float, nConstraints, 3 * nBodies>;

template <Uint32 nBodies>
using VelocityVector = Vector<float, 3 * nBodies>;

template <Uint32 nBodies>
using MassMatrix = Matrix<float, 3 * nBodies, 3 * nBodies>;


class ConstraintSolver
{
public:

    template <Uint32 nBodies>
    static VelocityVector<nBodies> velocity(const ConstBodies<nBodies>& bodies)
    {
        VelocityVector<nBodies> v{};
        Uint32                  i = 0;
        for (const PhysicsBody* body : bodies)
        {
            v[i++] = body->velocity()[0];
            v[i++] = body->velocity()[1];
            v[i++] = body->angularVelocity();
        }

        return v;
    }

    template <Uint32 nBodies>
    static MassMatrix<nBodies> inverseMass(const ConstBodies<nBodies>& bodies)
    {
        // TODO: Body dominance ratios
        Vector<float, 3 * nBodies> diagonal;
        Uint32                     i = 0;
        for (const PhysicsBody* body : bodies)
        {
            diagonal[i++] = 1 / body->properties().mass;
            diagonal[i++] = 1 / body->properties().mass;
            diagonal[i++] = 1 / body->properties().inertia;
        }

        return MassMatrix<nBodies>::diagonal(diagonal);
    }

    template <Uint32 nConstraints, Uint32 nBodies>
    static ConstraintValue<nConstraints> solveLambda(
        const ConstBodies<nBodies>&     bodies,
        ConstraintValue<nConstraints>   C,
        Jacobian<nConstraints, nBodies> J,
        float                           dt,
        ConstraintValue<nConstraints>   bias,
        ConstraintValue<nConstraints>   beta,
        ConstraintValue<nConstraints>   gamma,
        ConstraintValue<nConstraints>   oldLambda
    )
    {
        typedef Matrix<float, nConstraints, nConstraints> MatType;

        MatType K
            = J * inverseMass(bodies) * transpose(J) + MatType::diagonal(gamma);

        ConstraintValue<nConstraints> rhs
            = -(J * velocity(bodies) + bias + MatType::diagonal(beta) * C / dt
                + MatType::diagonal(gamma) * oldLambda);

        return solve(K, rhs);
    }

    template <Uint32 nConstraints, Uint32 nBodies>
    static VelocityVector<nBodies> impulse(
        const Jacobian<nConstraints, nBodies>& J,
        const ConstraintValue<nConstraints>&   lambda
    )
    {
        return transpose(J) * lambda;
    }

    template <Uint32 nBodies>
    static void
    apply(const Bodies<nBodies>& bodies, const VelocityVector<nBodies>& impulse)
    {
        VelocityVector<nBodies> dv = inverseMass<nBodies>(bodies) * impulse;
        Uint32                  i  = 0;
        for (PhysicsBody* body : bodies)
        {
            body->velocity() += Vec2{dv[i], dv[i + 1]};
            body->angularVelocity() += dv[i + 2];
            i += 3;
        }
    }
};


template <class F>
concept ConstraintFunction = requires(
    F                       f,
    typename F::Value       val,
    ConstBodies<F::nBodies> bodies,
    float                   dt
) {
    // clang-format off
    typename F::Value;
    std::is_same_v<typename F::Value, Vector<float, F::dimension>>; 
    
    typename F::Jacobian;
    std::is_same_v<typename F::Jacobian, Matrix<float, F::dimension, 3*F::nBodies>>; 

    { f.eval(bodies) } -> std::same_as<typename F::Value>;

    { f.bias(bodies) } -> std::same_as<typename F::Value>;
    { f.jacobian(bodies) } -> std::same_as<typename F::Jacobian>;

    { f.isActive(val) }        -> std::same_as<bool>;
    { f.clampLambda(val, dt) } -> std::same_as<typename F::Value>;

    { f.restitution() } -> std::same_as<typename F::Value>;
    { f.damping() }     -> std::same_as<typename F::Value>;
    // clang-format on
};

template <Uint32 nBodies_, Uint32 dimension_>
class ConstraintBase
{
public:

    static constexpr Uint32 nBodies   = nBodies_;
    static constexpr Uint32 dimension = dimension_;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    struct Descriptor
    {
        Value restitution = Value::filled(0.2f);
        Value damping     = Value::filled(0.f);
    };

    ConstraintBase(const Descriptor& descriptor) : descriptor_{descriptor} {}

    Value restitution() const { return descriptor_.restitution; }
    Value damping() const { return descriptor_.damping; }

private:

    Descriptor descriptor_;
};


namespace details
{

template <ConstraintFunction... Fs>
struct Dimension : public std::integral_constant<Uint32, -1>
{
};

template <ConstraintFunction F, ConstraintFunction... Fs>
struct Dimension<F, Fs...>
    : public std::integral_constant<Uint32, F::dimension + Dimension<Fs...>::value>
{
};

template <>
struct Dimension<> : public std::integral_constant<Uint32, 0>
{
};


template <ConstraintFunction... Fs>
struct NBodies : public std::integral_constant<Uint32, -1>
{
};

template <ConstraintFunction F>
struct NBodies<F> : public std::integral_constant<Uint32, F::nBodies>
{
};

template <ConstraintFunction F1, ConstraintFunction F2, ConstraintFunction... Fs>
struct NBodies<F1, F2, Fs...> : public NBodies<F1>
{
    static_assert(
        NBodies<F1>::value == NBodies<F2, Fs...>::value,
        "Incompatible constraints"
    );
};


template <class Tuple, class Func, Uint32 i = 0>
    requires std::invocable<Func, std::tuple_element_t<i, Tuple>&>
constexpr void forEach(Tuple& tuple, Func& func)
{
    func(std::get<i>(tuple));
    if constexpr (i + 1 < std::tuple_size_v<Tuple>)
        forEach<Tuple, Func, i + 1>(tuple, func);
}

template <class Tuple, class Func, Uint32 i = 0>
    requires std::invocable<Func, const std::tuple_element_t<i, Tuple>&>
constexpr void forEach(const Tuple& tuple, Func& func)
{
    func(std::get<i>(tuple));
    if constexpr (i + 1 < std::tuple_size_v<Tuple>)
        forEach<Tuple, Func, i + 1>(tuple, func);
}

} // namespace details

template <ConstraintFunction... Fs>
class ConstraintFunctions
{
public:

    static constexpr Uint32 nBodies   = details::NBodies<Fs...>::value;
    static constexpr Uint32 dimension = details::Dimension<Fs...>::value;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    ConstraintFunctions(const Fs&... constraints) : constraints{constraints...}
    {
    }

    Value eval(const ConstBodies<nBodies>& bodies) const
    {
        Eval e{bodies};
        details::forEach(constraints, e);
        return static_cast<Value>(e.manip);
    }

    Jacobian jacobian(const ConstBodies<nBodies>& bodies) const
    {
        JacobianBuilder j{bodies};
        details::forEach(constraints, j);
        return static_cast<Jacobian>(j.manip);
    }

    bool isActive(const Value& evaluatedTo) const
    {
        IsActive isActive{evaluatedTo};
        details::forEach(constraints, isActive);
        return isActive.isActive;
    }

    Value clampLambda(const Value& lambda, float dt) const
    {
        LambdaClamper l{lambda, dt};
        details::forEach(constraints, l);
        return static_cast<Value>(l.manip);
    }

    Value bias(const ConstBodies<nBodies>& bodies) const
    {
        Bias b{bodies};
        details::forEach(constraints, b);
        return static_cast<Value>(b.manip);
    }

    Value restitution() const
    {
        Restitution r{};
        details::forEach(constraints, r);
        return static_cast<Value>(r.manip);
    }

    Value damping() const
    {
        Damping d{};
        details::forEach(constraints, d);
        return static_cast<Value>(d.manip);
    }

    std::tuple<Fs...> constraints;

private:

    template <class MatrixT>
    class MatrixRowManipulator
    {
        // TODO: Some free functions for matrix construction could help with type deduction

    public:

        MatrixRowManipulator() : rows_{} {}

        MatrixRowManipulator(const MatrixT& m) : rows_{m.asRows()} {}
        operator MatrixT() { return MatrixT::fromRows(rows_); }

        template <class SubMatrixT>
        void assign(Uint32 begin, const SubMatrixT& subMat)
        {
            for (Row row : subMat.asRows())
                rows_[begin++] = row;
        }

        template <Uint32 dim>
        auto extract(Uint32 begin)
        {
            Vector<Row, dim> rows;
            Uint32           end = begin + dim;

            for (Uint32 i = 0; begin + i < end; ++i)
                rows[i] = rows_[begin + i];

            return Matrix<typename Row::value_type, dim, Row::mRows * Row::nCols>::fromRows(
                rows
            );
        }

    private:

        typedef decltype(std::declval<MatrixT>().asRows()) MatrixRows;
        typedef typename MatrixRows::value_type            Row;

        MatrixRows rows_;
    };

    struct Bias
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.bias(bodies));
            i += details::Dimension<F>::value;
        }

        ConstBodies<nBodies>  bodies;
        MatrixRowManipulator<Value> manip;
        Uint32                      i = 0;
    };

    struct LambdaClamper
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            constexpr Uint32 offset = details::Dimension<F>::value;
            manip.assign(i, f.clampLambda(manip.extract<offset>(i), dt));
            i += offset;
        }

        MatrixRowManipulator<Value> manip;
        float                       dt;
        Uint32                      i = 0;
    };

    struct IsActive
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            isActive = isActive || f.isActive(manip.extract<F::dimension>(i));
            i += details::Dimension<F>::value;
        }

        MatrixRowManipulator<Value> manip;
        Uint32                      i        = 0;
        bool                        isActive = false;
    };

    struct JacobianBuilder
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.jacobian(bodies));
            i += details::Dimension<F>::value;
        }

        ConstBodies<nBodies>     bodies;
        MatrixRowManipulator<Jacobian> manip;
        Uint32                         i = 0;
    };

    struct Eval
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.eval(bodies));
            i += details::Dimension<F>::value;
        }

        ConstBodies<nBodies>  bodies;
        MatrixRowManipulator<Value> manip;
        Uint32                      i = 0;
    };

    struct Restitution
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.restitution());
            i += details::Dimension<F>::value;
        }

        MatrixRowManipulator<Value> manip;
        Uint32                      i = 0;
    };

    struct Damping
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.damping());
            i += details::Dimension<F>::value;
        }

        MatrixRowManipulator<Value> manip;
        Uint32                      i = 0;
    };
};


template <ConstraintFunction F>
class ConstraintImplementation : public Constraint
{
public:

    typedef F::Value Value;

    ConstraintImplementation(const Bodies<F::nBodies>& bodies, const F& f)
        : f{f}, bodies_{bodies}
    {
    }

    bool shouldDie() const override
    {
        for (const PhysicsBody* body : bodies_)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        eval_ = f.eval(bodies_);
        return f.isActive(eval_);
    }

    void initSolve(float dt) override
    {
        // TODO: Apply guess based on previous lambda
        // if (warmStarting)
        //      solver.applyImpulse(oldJacobian, oldLambda*dt/oldDt)
        // ConstraintSolver::apply(
        //     bodies_,
        //     ConstraintSolver::impulse<F::dimension, F::nBodies>(jacobian_, lambda_)
        // );
        //
        // Careful! motor constraints should not apply twice!

        jacobian_ = f.jacobian(bodies_);
        bias_     = f.bias(bodies_);
        lambda_   = Value{};
    }

    void solve(float dt) override
    {
        Value dLambda = ConstraintSolver::solveLambda<F::dimension, F::nBodies>(
            bodies_,
            eval_,
            jacobian_,
            dt,
            bias_, // TODO: This may need recomputing (depends on velocity?)
            f.restitution(),
            f.damping(),
            lambda_
        );

        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        VelocityVector<F::nBodies> impulse
            = ConstraintSolver::impulse<F::dimension, F::nBodies>(
                jacobian_,
                lambda_ - oldLambda
            );
        ConstraintSolver::apply(bodies_, impulse);
    }

    void commit() override
    {
        // TODO: Do position correction with split impulses here?
    }

    F f;

private:

    Bodies<F::nBodies> bodies_;

    F::Jacobian jacobian_{};
    F::Value    eval_{};
    F::Value    bias_{};
    F::Value    lambda_{};
};


////////////////////////////////////////////////////////////
// Predefined constraint functions
////////////////////////////////////////////////////////////

// TODO: Equality, Motor and inequality constraint Base

template <Uint32 nBodies, Uint32 dimension>
class EqualityConstraintFunctionBase : public ConstraintBase<nBodies, dimension>
{
public:

    typedef ConstraintBase<nBodies, dimension> Base;
    typedef typename Base::Value               Value;

    typedef const ConstBodies<nBodies>& CBodies;

    using Base::Base;

    bool  isActive(Base::Value) const { return true; }
    Value bias(CBodies) const { return Value{}; }
    Value clampLambda(Value lambda, float) const { return lambda; }
};


class RotationConstraintFunction : public EqualityConstraintFunctionBase<2, 1>
{
public:

    typedef EqualityConstraintFunctionBase<2, 1> Base;

    typedef const ConstBodies<nBodies>& CBodies;

    RotationConstraintFunction(Descriptor descr, const ConstBodies<nBodies>& bodies)
        : Base{descr}, initialAngle_{angleDiff(bodies)}
    {
    }

    Value eval(CBodies bodies) const
    {
        return angleDiff(bodies) - initialAngle_;
    }

    Jacobian jacobian(CBodies) const { return Jacobian{0, 0, -1, 0, 0, 1}; }

private:

    Value angleDiff(CBodies bodies) const
    {
        return Value{bodies[1]->orientation() - bodies[0]->orientation()};
    }

    Value initialAngle_;
};


class HingeConstraintFunction : public EqualityConstraintFunctionBase<2, 2>
{
public:

    typedef EqualityConstraintFunctionBase<2, 2> Base;

    typedef const ConstBodies<nBodies>& CBodies;

    HingeConstraintFunction(
        Descriptor                  descr,
        const ConstBodies<nBodies>& bodies,
        Vec2                        worldSpaceSharedPoint
    )
        : Base{descr},
          localSpaceSharedPoint_{
              bodies[0]->toLocalSpace() * worldSpaceSharedPoint,
              bodies[1]->toLocalSpace() * worldSpaceSharedPoint}
    {
    }

    Value eval(CBodies bodies) const
    {
        return bodies[1]->toWorldSpace() * localSpaceSharedPoint_[1]
               - bodies[0]->toWorldSpace() * localSpaceSharedPoint_[0];
    }

    Jacobian jacobian(CBodies bodies) const
    {
        std::array<Mat3, nBodies> toWorldSpace{
            bodies[0]->toWorldSpace(),
            bodies[1]->toWorldSpace()};

        std::array<Vec2, nBodies> centroidToSharedPoint{
            toWorldSpace[0] * localSpaceSharedPoint_[0]
                - toWorldSpace[0] * bodies[0]->localProperties().centroid,
            toWorldSpace[1] * localSpaceSharedPoint_[1]
                - toWorldSpace[1] * bodies[1]->localProperties().centroid,
        };

        std::array<Vec2, nBodies> instantRotation{
            perp(centroidToSharedPoint[0], false),
            perp(centroidToSharedPoint[1], false)};

        // clang-format off
        return Jacobian{
            -1,  0, -instantRotation[0][0], 1, 0, instantRotation[1][0],
             0, -1, -instantRotation[0][1], 0, 1, instantRotation[1][1]
        };
        // clang-format on
    }

private:

    std::array<Vec2, nBodies> localSpaceSharedPoint_;
};


////////////////////////////////////////////////////////////
// Motor Constraints
////////////////////////////////////////////////////////////

template <Uint32 nBodies_, Uint32 dimension_>
class MotorConstraintBase
{
public:

    static constexpr Uint32 nBodies   = nBodies_;
    static constexpr Uint32 dimension = dimension_;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    typedef const ConstBodies<nBodies>& CBodies;

    MotorConstraintBase(float maxVelocity, float maxForce)
        : maxVelocity_{std::abs(maxVelocity)}, maxForce_{std::abs(maxForce)}
    {
    }

    // no position restitution nor damping on motors
    Value eval(CBodies) const { return Value{}; }
    Value restitution() const { return Value{}; }
    Value damping() const { return Value{}; }

    bool isActive(Value) const { return true; }

    Value bias(CBodies) const { return -maxVelocity_ * direction_; }

    Value clampLambda(Value lambda, float dt) const
    {
        Value maxImpulse = direction_ * throttle_ * maxForce_ * dt;

        Uint32 i = 0;
        for (auto& l : lambda)
        {
            auto bound = maxImpulse[i++];
            l = (bound < 0) ? clamp(l, bound, 0.f) : clamp(l, 0.f, bound);
        }

        return lambda;
    }

    float throttle() const { return throttle_; }
    void  throttle(float throttle) { throttle_ = clamp(throttle, 0.f, 1.f); }

    Value direction() const { return direction_; }
    void  direction(Value direction)
    {
        if (normSquared(direction) == 0.f)
            direction_ = direction;
        else
            direction_ = normalized(direction);
    }

private:

    float throttle_ = 1.f;
    Value direction_{};

    float maxVelocity_;
    float maxForce_;
};


class RotationMotorConstraintFunction : public MotorConstraintBase<1, 1>
{
public:

    typedef MotorConstraintBase<1, 1> Base;

    typedef const ConstBodies<nBodies>& CBodies;

    RotationMotorConstraintFunction(
        CBodies bodies,
        float   maxAngularVelocity,
        float   maxAccel
    )
        : Base{maxAngularVelocity, maxAccel * bodies[0]->properties().inertia}
    {
    }

    Jacobian jacobian(CBodies) const { return Jacobian{0, 0, 1}; }
};


class TranslationMotorConstraintFunction : public MotorConstraintBase<1, 2>
{
public:

    typedef MotorConstraintBase<1, 2>   Base;
    typedef const ConstBodies<nBodies>& CBodies;

    TranslationMotorConstraintFunction(CBodies bodies, float maxVelocity, float maxAccel)
        : Base{maxVelocity, maxAccel * bodies[0]->properties().mass}
    {
    }

    Jacobian jacobian(CBodies) const
    {
        // clang-format off
        return Jacobian{1, 0, 0,
                        0, 1, 0};
        // clang-format on
    }
};


////////////////////////////////////////////////////////////
// Predefined constraints
////////////////////////////////////////////////////////////

class RotationConstraint
    : public ConstraintImplementation<RotationConstraintFunction>
{
public:

    typedef ConstraintImplementation<RotationConstraintFunction> Base;
    typedef RotationConstraintFunction                           F;

    struct Descriptor : public F::Descriptor
    {
        Descriptor(float restitution = 0.2f, float damping = 0.f)
            : F::Descriptor{
                Vector<float, 1>{restitution},
                Vector<float, 1>{damping}}
        {
        }
    };

    RotationConstraint(const Bodies<2>& bodies, Descriptor descr = Descriptor{}) 
        : Base{bodies, F{descr, bodies}}
    {
    }
};

class HingeConstraint : public ConstraintImplementation<HingeConstraintFunction>
{
public:

    typedef ConstraintImplementation<HingeConstraintFunction> Base;
    typedef HingeConstraintFunction                           F;

    struct Descriptor : public F::Descriptor
    {
        Descriptor(float restitution = 0.2f, float damping = 0.f)
            : F::Descriptor{Vec2::filled(restitution), Vec2::filled(damping)}
        {
        }
    };

    HingeConstraint(
        const Bodies<2>& bodies,
        Vec2             worldSpaceSharedPoint,
        Descriptor    descr = Descriptor{}
    ) : Base{bodies, F{descr, bodies, worldSpaceSharedPoint}}
    {
    }
};


typedef ConstraintFunctions<HingeConstraintFunction, RotationConstraintFunction>
    WeldConstraintFunction;

class WeldConstraint : public ConstraintImplementation<WeldConstraintFunction>
{
public:

    typedef HingeConstraintFunction    HingeF;
    typedef RotationConstraintFunction RotF;
    typedef WeldConstraintFunction     F;

    typedef ConstraintImplementation<F> Base;

    struct Descriptor : public HingeF::Descriptor, RotF::Descriptor
    {
        Descriptor(
            float restitution        = 0.2f,
            float angularRestitution = 0.2f,
            float damping            = 0.f,
            float angularDamping     = 0.f
        )
            : HingeF::Descriptor{Vec2::filled(restitution), Vec2::filled(damping)},
              RotF::Descriptor{
                  Vector<float, 1>{angularRestitution},
                  Vector<float, 1>{angularDamping}}
        {
        }
    };

    WeldConstraint(const Bodies<2>& bodies, Descriptor descr = Descriptor{})
        : Base{bodies, makeWeldFunction(bodies, descr)}
    {
    }

private:

    static Vec2 centroidMidPoint(const Bodies<2>& bodies)
    {
        return (bodies[0]->properties().centroid
                + bodies[1]->properties().centroid)
               / 2;
    }

    static HingeF makeHingeFunction(const Bodies<2>& bodies, Descriptor descr)
    {
        return HingeF{
            static_cast<HingeF::Descriptor>(descr),
            bodies,
            centroidMidPoint(bodies)};
    }

    static RotF makeRotFunction(const Bodies<2>& bodies, Descriptor descr)
    {
        return RotF{static_cast<RotF::Descriptor>(descr), bodies};
    }

    static F makeWeldFunction(const Bodies<2>& bodies, Descriptor descr)
    {
        return F{makeHingeFunction(bodies, descr), makeRotFunction(bodies, descr)};
    }
};


class RotationMotorConstraint
    : public ConstraintImplementation<RotationMotorConstraintFunction>
{
public:

    typedef RotationMotorConstraintFunction F;
    typedef ConstraintImplementation<F>     Base;

    RotationMotorConstraint(
        const Bodies<1>& bodies,
        float            maxAngularVelocity,
        float            maxTorque
    )
        : Base{
            bodies,
            F{bodies, maxAngularVelocity, maxTorque}
    }
    {
    }

    float throttle() const { return f.throttle(); }
    void  throttle(float throttle) { f.throttle(throttle); }

    Value direction() const { return f.direction(); }
    void  direction(Value direction) { f.direction(direction); }
};


class TranslationMotorConstraint
    : public ConstraintImplementation<TranslationMotorConstraintFunction>
{
public:

    typedef TranslationMotorConstraintFunction F;
    typedef ConstraintImplementation<F>        Base;

    TranslationMotorConstraint(const Bodies<1>& bodies, float maxVelocity, float maxForce)
        : Base{
            bodies,
            F{bodies, maxVelocity, maxForce}
    }
    {
    }

    float throttle() const { return f.throttle(); }
    void  throttle(float throttle) { f.throttle(throttle); }

    Value direction() const { return f.direction(); }
    void  direction(Value direction) { f.direction(direction); }
};


////////////////////////////////////////////////////////////
// Contacts
////////////////////////////////////////////////////////////

class NonPenetrationConstraintFunction
{
public:

    static constexpr Uint32 nBodies   = 2;
    static constexpr Uint32 dimension = 1;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    typedef const ConstBodies<nBodies>& CBodies;
    typedef ContactManifold<Collider>   Manifold;

    NonPenetrationConstraintFunction(
        CBodies         bodies,
        const Manifold& manifold,
        Uint32          contactIndex
    )
        : normal_{normalized(manifold.contactNormal)},
          reference_{manifold.referenceIndex()},
          incident_{manifold.incidentIndex()},
          restitutionCoefficient_{CombinableProperty{bodies[0]->material().bounciness, 
                                                     bodies[1]->material().bounciness}.value}
    {
        localSpaceContacts_[incident_] = bodies[incident_]->toLocalSpace()
                                         * manifold.contacts[contactIndex];

        // TODO: The manifold should compute this on his own
        auto refEdge = manifold.contactEdges[reference_];
        Vec2 worldSpaceRefContact
            = LineBarycentric{*refEdge.from, *refEdge.to, manifold.contacts[contactIndex]}
                  .closestPoint;

        localSpaceContacts_[reference_]
            = bodies[reference_]->toLocalSpace() * worldSpaceRefContact;
    }

    Value eval(CBodies bodies) const
    {
        auto contacts = worldSpaceContacts(bodies);
        Vec2 relPos   = contacts[incident_] - contacts[reference_];
        return Value{dot(relPos, normal_)};
    }

    bool isActive(Value evaluatedTo) const
    {
        // TODO: Allow some slop, given in bodies' material
        return evaluatedTo[0] < 0;
    }

    // TODO: Constraint implementation will need to recompute the bias...
    Value bias(CBodies bodies) const
    {
        // TODO: Restitution coefficients will be held in the bodies' material,
        //      may not need to save this as member.
        return restitutionCoefficient_ * jacobian(bodies)
               * ConstraintSolver::velocity(bodies);
    }

    Jacobian jacobian(CBodies bodies) const
    {
        auto     contacts = worldSpaceContacts(bodies);
        Jacobian J;

        J[3 * incident_]     = normal_[0];
        J[3 * incident_ + 1] = normal_[1];
        J[3 * incident_ + 2] = cross(
            contacts[incident_] - bodies[incident_]->properties().centroid,
            normal_
        );

        J[3 * reference_]     = -normal_[0];
        J[3 * reference_ + 1] = -normal_[1];
        J[3 * reference_ + 2] = -cross(
            contacts[reference_] - bodies[reference_]->properties().centroid,
            normal_
        );

        return J;
    }

    Value clampLambda(Value lambda, float dt) const
    {
        return std::max(Value{0}, lambda);
    }

    Value restitution() const
    {
        return Value{0.f}; // TODO: just use constraint base
    }

    Value damping() const
    {
        return Value{0.f}; // TODO:
    }


private:

    std::array<Vec2, 2> worldSpaceContacts(CBodies bodies) const
    {
        return std::array<Vec2, 2>{
            bodies[0]->toWorldSpace() * localSpaceContacts_[0],
            bodies[1]->toWorldSpace() * localSpaceContacts_[1]};
    }

    // TODO: Careful that these are updated between timesteps
    std::array<Vec2, 2> localSpaceContacts_;
    Vec2                normal_;
    Uint32              reference_;
    Uint32              incident_;

    float restitutionCoefficient_;
};


class FrictionConstraintFunction
{
public:

    static constexpr Uint32 nBodies   = 2;
    static constexpr Uint32 dimension = 1;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    typedef const ConstBodies<nBodies>& CBodies;
    typedef ContactManifold<Collider>   Manifold;

    FrictionConstraintFunction(
        CBodies         bodies,
        const Manifold& manifold
    )
        : tangent_{normalized(perp(manifold.contactNormal))},
          frictionCoefficient_{CombinableProperty{bodies[0]->material().friction, 
                                                  bodies[1]->material().friction}.value}
    {
        localSpaceContacts_[0]
            = bodies[0]->toLocalSpace() * manifold.contacts[0];

        // TODO: The manifold should compute this on his own
        auto refEdge = manifold.contactEdges[manifold.referenceIndex()];
        Vec2 worldSpaceRefContact
            = LineBarycentric{*refEdge.from, *refEdge.to, manifold.contacts[0]}
                  .closestPoint;

        localSpaceContacts_[1]
            = bodies[1]->toLocalSpace() * worldSpaceRefContact;
    }

    Value eval(CBodies bodies) const { return Value{}; }

    bool isActive(Value evaluatedTo) const { return true; }

    Value bias(CBodies bodies) const { return Value{}; }

    Jacobian jacobian(CBodies bodies) const
    {
        auto contacts = worldSpaceContacts(bodies);
        return Jacobian{
            tangent_[0],
            tangent_[1],
            cross(contacts[0] - bodies[0]->properties().centroid, tangent_),
            -tangent_[0],
            -tangent_[1],
            -cross(contacts[1] - bodies[1]->properties().centroid, tangent_)};
    }

    Value clampLambda(Value lambda, float dt) const
    {
        SIMU_ASSERT(
            false,
            "must be bounded by normal force. Call the other overloads"
        );
    }

    Value
    clampLambda(Value lambda, float dt, ConstraintValue<1> lambdaNormal) const
    {
        float absBound = frictionCoefficient_ * std::abs(lambdaNormal[0]);
        return Value{clamp(lambda[0], -absBound, absBound)};
    }

    Value
    clampLambda(Value lambda, float dt, ConstraintValue<2> lambdaNormal) const
    {
        return clampLambda(
            lambda,
            dt,
            ConstraintValue<1>{lambdaNormal[0] + lambdaNormal[1]}
        );
    }


    Value restitution() const
    {
        return Value{0.f}; // TODO: just use constraint base
    }

    Value damping() const
    {
        return Value{0.f}; // TODO:
    }


private:

    std::array<Vec2, 2> worldSpaceContacts(CBodies bodies) const
    {
        return std::array<Vec2, 2>{
            bodies[0]->toWorldSpace() * localSpaceContacts_[0],
            bodies[1]->toWorldSpace() * localSpaceContacts_[1]};
    }

    std::array<Vec2, 2> localSpaceContacts_;
    Vec2                tangent_;

    float frictionCoefficient_; // TODO: Dynamic vs static
};

using SingleContactFunction
    = ConstraintFunctions<NonPenetrationConstraintFunction, FrictionConstraintFunction>;

using DoubleContactFunction = ConstraintFunctions<
    NonPenetrationConstraintFunction,
    NonPenetrationConstraintFunction,
    FrictionConstraintFunction>;

template <>
SingleContactFunction::Value
SingleContactFunction::clampLambda(const Value& lambda, float dt) const;

template <>
DoubleContactFunction::Value
DoubleContactFunction::clampLambda(const Value& lambda, float dt) const;


class SingleContactConstraint
    : public ConstraintImplementation<SingleContactFunction>
{
public:

    typedef ConstraintImplementation<SingleContactFunction> Base;

    typedef ContactManifold<Collider> Manifold;

    SingleContactConstraint(const Bodies<2>& bodies, const Manifold& manifold)
        : Base{bodies, makeFunction(bodies, manifold)}
    {
    }

private:

    static SingleContactFunction
    makeFunction(const ConstBodies<2>& bodies, const Manifold& manifold)
    {
        return SingleContactFunction{
            NonPenetrationConstraintFunction{bodies, manifold, 0},
            FrictionConstraintFunction{bodies, manifold}
        };
    }
};

class DoubleContactConstraint
    : public ConstraintImplementation<DoubleContactFunction>
{
public:

    typedef ConstraintImplementation<DoubleContactFunction> Base;

    typedef ContactManifold<Collider> Manifold;

    DoubleContactConstraint(const Bodies<2>& bodies, const Manifold& manifold)
        : Base{bodies, makeFunction(bodies, manifold)}
    {
    }

private:

    static DoubleContactFunction
    makeFunction(const ConstBodies<2>& bodies, const Manifold& manifold)
    {
        return DoubleContactFunction{
            NonPenetrationConstraintFunction{bodies, manifold, 0},
            NonPenetrationConstraintFunction{bodies, manifold, 1},
            FrictionConstraintFunction{bodies, manifold}
        };
    }
};

struct Contact
{
    Contact(const Bodies<2>& bodies) : bodies{bodies} {}

    Gjk<Collider> makeGjk() const
    {
        return Gjk<Collider>{bodies[0]->collider(), bodies[1]->collider()};
    }

    // gjk must return a non null penetration to make a manifold
    ContactManifold<Collider> makeManifold() const
    {
        return makeManifold(makeGjk());
    }
    ContactManifold<Collider> makeManifold(const Gjk<Collider>& gjk) const
    {
        return ContactManifold<Collider>{
            PointerArray<Collider, 2, true>{
                                            &bodies[0]->collider(),
                                            &bodies[1]->collider()},
            gjk.penetration()
        };
    }

    Bodies<2> bodies;
};

class ContactConstraint : public Constraint
{
public:

    ContactConstraint(const Bodies<2>& bodies) : contact_{bodies} {}

    bool isActive() override
    {
        auto gjk = contact_.makeGjk();
        if (all(gjk.penetration() == Vec2{}))
            return false;

        auto manifold = contact_.makeManifold(gjk);
        makeContactConstraint(manifold);

        return contactConstraint_->isActive();
    }

    void initSolve(float dt) override { contactConstraint_->initSolve(dt); }
    void solve(float dt) override { contactConstraint_->solve(dt); }
    void commit() override { contactConstraint_->commit(); }

private:


    Contact                     contact_;
    std::unique_ptr<Constraint> contactConstraint_ = nullptr;

    void makeContactConstraint(const ContactManifold<Collider>& manifold)
    {
        if (manifold.nContacts == 1)
            contactConstraint_ = std::make_unique<SingleContactConstraint>(
                contact_.bodies,
                manifold
            );
        else
            contactConstraint_ = std::make_unique<DoubleContactConstraint>(
                contact_.bodies,
                manifold
            );
    }
};


} // namespace simu
