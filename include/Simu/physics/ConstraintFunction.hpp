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
#include "Simu/physics/Body.hpp"
#include "Simu/physics/Bodies.hpp"
#include "Simu/physics/ContactManifold.hpp"
#include "Simu/physics/ConstraintInterfaces.hpp"

namespace simu
{


namespace details
{

template <ConstraintFunction... Fs>
struct Dimension : public std::integral_constant<Uint32, 0>
{
};

template <ConstraintFunction F, ConstraintFunction... Fs>
struct Dimension<F, Fs...>
    : public std::integral_constant<Uint32, F::dimension + Dimension<Fs...>::value>
{
};

template <ConstraintFunction... Fs>
struct NProxies : public std::integral_constant<Uint32, 0>
{
};

template <ConstraintFunction F>
struct NProxies<F> : public std::integral_constant<Uint32, F::nBodies>
{
};

template <ConstraintFunction F1, ConstraintFunction F2, ConstraintFunction... Fs>
struct NProxies<F1, F2, Fs...> : public NProxies<F1>
{
    static_assert(
        NProxies<F1>::value == NProxies<F2, Fs...>::value,
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

    static constexpr Uint32 nBodies   = details::NProxies<Fs...>::value;
    static constexpr Uint32 dimension = details::Dimension<Fs...>::value;

    typedef Vector<float, dimension>    Value;
    typedef Matrix<float, dimension, 6> Jacobian;

    ConstraintFunctions(const Fs&... constraints) : constraints{constraints...}
    {
    }

    Value eval(const Proxies& proxies) const
    {
        Eval e{proxies};
        details::forEach(constraints, e);
        return static_cast<Value>(e.manip);
    }

    Jacobian jacobian(const Proxies& proxies) const
    {
        JacobianBuilder j{proxies};
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

    Value bias(const Proxies& proxies) const
    {
        Bias b{proxies};
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
            manip.assign(i, f.bias(proxies));
            i += details::Dimension<F>::value;
        }

        const Proxies&              proxies;
        MatrixRowManipulator<Value> manip{};
        Uint32                      i = 0;
    };

    struct LambdaClamper
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            constexpr Uint32 offset = details::Dimension<F>::value;
            manip.assign(i, f.clampLambda(manip.template extract<offset>(i), dt));
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
            manip.assign(
                i,
                f.clampPositionLambda(manip.template extract<offset>(i))
            );
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
            manip.assign(i, f.jacobian(proxies));
            i += details::Dimension<F>::value;
        }

        const Proxies&                 proxies;
        MatrixRowManipulator<Jacobian> manip{};
        Uint32                         i = 0;
    };

    struct Eval
    {
        template <ConstraintFunction F>
        void operator()(F f)
        {
            manip.assign(i, f.eval(proxies));
            i += details::Dimension<F>::value;
        }

        const Proxies&              proxies;
        MatrixRowManipulator<Value> manip{};
        Uint32                      i = 0;
    };
};


template <Uint32 nBodies_, Uint32 dimension_>
class EqualityConstraintFunctionBase
{
public:

    static constexpr Uint32 nBodies   = nBodies_;
    static constexpr Uint32 dimension = dimension_;

    typedef Vector<float, dimension>    Value;
    typedef Matrix<float, dimension, 6> Jacobian;

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

    typedef Vector<float, dimension>    Value;
    typedef Matrix<float, dimension, 6> Jacobian;

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


    RotationConstraintFunction(const Bodies& bodies)
        : Base{},
          initialAngle_{bodies[1]->orientation() - bodies[0]->orientation()}
    {
    }

    Value eval(const Proxies& proxies) const
    {
        return Value{proxies[1].orientation() - proxies[0].orientation()}
               - initialAngle_;
    }

    Value bias(const Proxies& /* proxies */) const { return Value{}; }

    Jacobian jacobian(const Proxies& /* proxies */) const
    {
        return Jacobian{0, 0, -1, 0, 0, 1};
    }

private:

    Value initialAngle_;
};


class HingeConstraintFunction : public EqualityConstraintFunctionBase<2, 2>
{
public:

    typedef EqualityConstraintFunctionBase<2, 2> Base;

    HingeConstraintFunction(const Bodies& bodies, Vec2 worldSpaceSharedPoint)
        : Base{},
          localSpaceSharedPoint_{
              bodies[0]->toLocalSpace() * worldSpaceSharedPoint,
              bodies[1]->toLocalSpace() * worldSpaceSharedPoint}
    {
    }

    Value eval(const Proxies& proxies) const
    {
        auto worldPoints = worldSpacePoints(proxies);
        return worldPoints[1] - worldPoints[0];
    }

    Value bias(const Proxies& /* proxies */) const { return Value{}; }

    Jacobian jacobian(const Proxies& proxies) const
    {
        auto worldPoints = worldSpacePoints(proxies);

        std::array<Vec2, nBodies> centroidToSharedPoint{
            worldPoints[0] - proxies[0].centroid(),
            worldPoints[1] - proxies[1].centroid(),
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

    std::array<Vec2, nBodies> worldSpacePoints(const Proxies& proxies) const
    {
        return {
            proxies[0].toWorldSpace() * localSpaceSharedPoint_[0],
            proxies[1].toWorldSpace() * localSpaceSharedPoint_[1],
        };
    }

    std::array<Vec2, nBodies> localSpaceSharedPoint_;
};


} // namespace simu
