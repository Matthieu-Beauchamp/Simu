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
#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/ContactManifold.hpp"
#include "Simu/physics/ConstraintSolver.hpp"

namespace simu
{


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

    Value clampLambda(const Value& lambda, float dt) const
    {
        LambdaClamper l{lambda, dt};
        details::forEach(constraints, l);
        return static_cast<Value>(l.manip);
    }

    Value clampPositionLambda(const Value& lambda) const
    {
        PosLambdaClamper l{lambda};
        details::forEach(constraints, l);
        return static_cast<Value>(l.manip);
    }

    Value bias(const ConstBodies<nBodies>& bodies) const
    {
        Bias b{bodies};
        details::forEach(constraints, b);
        return static_cast<Value>(b.manip);
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

        ConstBodies<nBodies>        bodies;
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

    struct PosLambdaClamper
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            constexpr Uint32 offset = details::Dimension<F>::value;
            manip.assign(i, f.clampPositionLambda(manip.extract<offset>(i)));
            i += offset;
        }

        MatrixRowManipulator<Value> manip;
        Uint32                      i = 0;
    };

    struct JacobianBuilder
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.jacobian(bodies));
            i += details::Dimension<F>::value;
        }

        ConstBodies<nBodies>           bodies;
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

        ConstBodies<nBodies>        bodies;
        MatrixRowManipulator<Value> manip;
        Uint32                      i = 0;
    };
};


template <Uint32 nBodies_, Uint32 dimension_>
class EqualityConstraintFunctionBase
{
public:

    static constexpr Uint32 nBodies   = nBodies_;
    static constexpr Uint32 dimension = dimension_;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    typedef const ConstBodies<nBodies>& CBodies;

    EqualityConstraintFunctionBase() = default;

    Value clampLambda(Value lambda, float /* dt */) const { return lambda; }
    Value clampPositionLambda(Value lambda) const { return lambda; }
};

template <Uint32 nBodies_, Uint32 dimension_>
class InequalityConstraintFunctionBase
{
public:

    static constexpr Uint32 nBodies   = nBodies_;
    static constexpr Uint32 dimension = dimension_;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    typedef const ConstBodies<nBodies>& CBodies;

    InequalityConstraintFunctionBase() = default;

    Value clampLambda(Value lambda, float /* dt */) const
    {
        return std::max(lambda, Value::filled(0.f));
    }

    Value clampPositionLambda(Value lambda) const
    {
        return std::max(lambda, Value::filled(0.f));
    }
};

////////////////////////////////////////////////////////////
// Predefined constraint functions
////////////////////////////////////////////////////////////

class RotationConstraintFunction : public EqualityConstraintFunctionBase<2, 1>
{
public:

    typedef EqualityConstraintFunctionBase<2, 1> Base;


    RotationConstraintFunction(CBodies bodies)
        : Base{}, initialAngle_{angleDiff(bodies)}
    {
    }

    Value eval(CBodies bodies) const
    {
        return angleDiff(bodies) - initialAngle_;
    }

    Value bias(CBodies /* bodies */) const { return Value{}; }

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

    HingeConstraintFunction(CBodies bodies, Vec2 worldSpaceSharedPoint)
        : Base{},
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

    Value bias(CBodies /* bodies */) const { return Value{}; }

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
// Contacts
////////////////////////////////////////////////////////////

class NonPenetrationConstraintFunction
    : public InequalityConstraintFunctionBase<2, 1>
{
public:

    typedef InequalityConstraintFunctionBase<2, 1> Base;

    typedef ContactManifold<Collider> Manifold;

    NonPenetrationConstraintFunction(
        CBodies         bodies,
        const Manifold& manifold,
        Uint32          contactIndex
    )
        : Base{}, 
          normal_{(manifold.incidentIndex() == 0 ? 1.f:-1.f) * normalized(manifold.contactNormal)},
          restitutionCoefficient_{CombinableProperty{bodies[0]->material().bounciness, 
                                                     bodies[1]->material().bounciness}.value}
    {
        localSpaceContacts_[0]
            = bodies[0]->toLocalSpace() * manifold.contacts[0][contactIndex];

        localSpaceContacts_[1]
            = bodies[1]->toLocalSpace() * manifold.contacts[1][contactIndex];
    }

    Vec2 contactDistance(CBodies bodies) const
    {
        auto contacts = worldSpaceContacts(bodies);
        return contacts[0] - contacts[1];
    }

    Value eval(CBodies bodies) const
    {
        return Value{dot(contactDistance(bodies), normal_)};
    }

    Value bias(CBodies bodies) const
    {
        auto penetratingRelVelocity
            = jacobian(bodies)
              * ConstraintSolver<NonPenetrationConstraintFunction>::velocity(bodies
              );

        return restitutionCoefficient_
               * (penetratingRelVelocity - accumulatedRelVel);
    }

    Jacobian jacobian(CBodies bodies) const
    {
        auto contacts = worldSpaceContacts(bodies);

        Jacobian J{
            normal_[0],
            normal_[1],
            cross(contacts[0] - bodies[0]->properties().centroid, normal_),
            -normal_[0],
            -normal_[1],
            -cross(contacts[1] - bodies[1]->properties().centroid, normal_)};


        // This must be added to J[3*reference_ +2] for the complete equation
        //  cross(normal_, contacts[incident_] - contacts[reference_])

        return J;
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

    float restitutionCoefficient_;

public:

    Value accumulatedRelVel{};
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
            = bodies[0]->toLocalSpace() * manifold.contacts[0][0];

        localSpaceContacts_[1]
            = bodies[1]->toLocalSpace() * manifold.contacts[1][0];
    }

    Value eval(CBodies /* bodies */) const { return Value{}; }
    Value bias(CBodies /* bodies */) const { return Value{}; }

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

    Value clampLambda(Value /* lambda */, float /* dt */) const
    {
        SIMU_ASSERT(
            false,
            "must be bounded by normal force. Call the other overloads"
        );
    }

    Value
    clampLambda(Value lambda, float /* dt */, Vector<float, 1> lambdaNormal) const
    {
        float absBound = frictionCoefficient_ * std::abs(lambdaNormal[0]);
        return Value{clamp(lambda[0], -absBound, absBound)};
    }

    Value clampLambda(Value lambda, float dt, Vec2 lambdaNormal) const
    {
        return clampLambda(lambda, dt, Value{lambdaNormal[0] + lambdaNormal[1]});
    }

    Value clampPositionLambda(Value /* lambda */) const { return Value{}; }

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


} // namespace simu

#include "Simu/physics/ConstraintFunction.inl.hpp"
