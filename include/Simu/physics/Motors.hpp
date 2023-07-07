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


template <Uint32 nBodies_, Uint32 dimension_>
class MotorFunctionBase
{
public:

    static constexpr Uint32 nBodies   = nBodies_;
    static constexpr Uint32 dimension = dimension_;

    typedef Vector<float, dimension>              Value;
    typedef Matrix<float, dimension, 3 * nBodies> Jacobian;

    MotorFunctionBase(float maxVelocity, float maxForce)
        : maxVelocity_{std::abs(maxVelocity)}, maxForce_{std::abs(maxForce)}
    {
    }

    Value eval(const Bodies<nBodies> /* bodies */) const { return Value{}; }
    Value bias(const Bodies<nBodies> /* bodies */) const
    {
        return -maxVelocity_ * direction_;
    }

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

    Value clampPositionLambda(Value /* lambda */) const { return Value{}; }

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


class RotationMotorFunction : public MotorFunctionBase<1, 1>
{
public:

    typedef MotorFunctionBase<1, 1> Base;

    RotationMotorFunction(float maxAngularVelocity, float maxTorque)
        : Base{maxAngularVelocity, maxTorque}
    {
    }

    Jacobian jacobian(const Bodies<1>& /* bodies */) const
    {
        return Jacobian{0, 0, 1};
    }
};


class TranslationMotorFunction : public MotorFunctionBase<1, 2>
{
public:

    typedef MotorFunctionBase<1, 2> Base;

    TranslationMotorFunction(float maxVelocity, float maxForce)
        : Base{maxVelocity, maxForce}
    {
    }

    Jacobian jacobian(const Bodies<1>& /* bodies */) const
    {
        // clang-format off
        return Jacobian{1, 0, 0,
                        0, 1, 0};
        // clang-format on
    }
};


class RotationMotor : public ConstraintImplementation<RotationMotorFunction>
{
public:

    typedef RotationMotorFunction       F;
    typedef ConstraintImplementation<F> Base;

    class Specs
    {
    public:

        static Specs fromTorque(float maxAngularVelocity, float maxTorque)
        {
            Specs s{};
            s.maxAngularVelocity_ = maxAngularVelocity;
            s.maxTorque_          = maxTorque;
            return s;
        }

        static Specs fromAccel(
            float                maxAngularVelocity,
            float                maxAngularAccel,
            std::optional<float> targetInertia = std::nullopt
        )
        {
            if (targetInertia.has_value())
                return fromTorque(
                    maxAngularVelocity,
                    maxAngularAccel * targetInertia.value()
                );

            Specs s{};
            s.maxAngularVelocity_ = maxAngularVelocity;
            s.maxAccel_           = maxAngularAccel;
            return s;
        }

        static Specs fromTargetForce(
            float maxAngularVelocity,
            float radius,
            float targetForceAtCentroid
        )
        {
            return fromTorque(maxAngularVelocity, radius * targetForceAtCentroid);
        }

        float maxAngularVelocity() const { return maxAngularVelocity_; }
        float maxTorque(const Bodies<1>& body) const
        {
            if (maxTorque_.has_value())
                return maxTorque_.value();

            return maxAccel_.value() * body[0]->properties().inertia;
        }

    private:

        float                maxAngularVelocity_{};
        std::optional<float> maxAccel_{};
        std::optional<float> maxTorque_{};
    };

    RotationMotor(const Bodies<1>& bodies, Specs specs)
        : Base{
            bodies,
            F{specs.maxAngularVelocity(), specs.maxTorque(bodies)},
            false
    }
    {
    }

    float throttle() const { return f.throttle(); }
    void  throttle(float throttle)
    {
        f.throttle(throttle);
        for (Body* body : bodies())
            body->wake();
    }

    Value direction() const { return f.direction(); }
    void  direction(Value direction)
    {
        f.direction(direction);
        for (Body* body : bodies())
            body->wake();
    }
};


class TranslationMotor
    : public ConstraintImplementation<TranslationMotorFunction>
{
public:

    typedef TranslationMotorFunction    F;
    typedef ConstraintImplementation<F> Base;

    class Specs
    {
    public:

        static Specs fromForce(float maxVelocity, float maxForce)
        {
            Specs s{};
            s.maxVelocity_ = maxVelocity;
            s.maxForce_    = maxForce;
            return s;
        }

        static Specs fromAccel(
            float                maxVelocity,
            float                maxAccel,
            std::optional<float> targetMass = std::nullopt
        )
        {
            if (targetMass.has_value())
                return fromForce(maxVelocity, maxAccel * targetMass.value());

            Specs s{};
            s.maxVelocity_ = maxVelocity;
            s.maxAccel_    = maxAccel;
            return s;
        }

        float maxVelocity() const { return maxVelocity_; }
        float maxForce(const Bodies<1>& body) const
        {
            if (maxForce_.has_value())
                return maxForce_.value();

            return maxAccel_.value() * body[0]->properties().mass;
        }

    private:

        float                maxVelocity_{};
        std::optional<float> maxAccel_{};
        std::optional<float> maxForce_{};
    };

    TranslationMotor(const Bodies<1>& bodies, Specs specs)
        : Base{
            bodies,
            F{specs.maxVelocity(), specs.maxForce(bodies)},
            false
    }
    {
    }

    float throttle() const { return f.throttle(); }
    void  throttle(float throttle)
    {
        f.throttle(throttle);
        for (Body* body : bodies())
            body->wake();
    }

    Value direction() const { return f.direction(); }
    void  direction(Value direction)
    {
        f.direction(direction);
        for (Body* body : bodies())
            body->wake();
    }
};


} // namespace simu
