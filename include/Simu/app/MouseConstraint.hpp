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

#include "Simu/physics/Constraint.hpp"

namespace simu
{

class MouseConstraintFunction : public EqualityConstraintFunctionBase<1, 2>
{
public:

    typedef EqualityConstraintFunctionBase<1, 2> Base;

    MouseConstraintFunction() : Base{} {}

    Value eval(Bodies<1> body)
    {
        return body[0]->toWorldSpace() * localBodyPos_ - mousePos_;
    }

    Value bias(Bodies<1> /* body */) { return Value{}; }


    Jacobian jacobian(Bodies<1> body)
    {
        Vec2 r = body[0]->toWorldSpace() * localBodyPos_
                 - body[0]->properties().centroid;

        // clang-format off
        return Jacobian{
            1.f, 0.f, -r[1],
            0.f, 1.f,  r[0],
        };
        // clang-format on
    }

    void updateMousePos(Vec2 pos) { mousePos_ = pos; }
    void updateBody(Body* body, Vec2 worldPos)
    {
        localBodyPos_ = body->toLocalSpace() * worldPos;
    }

private:

    Vec2 mousePos_;
    Vec2 localBodyPos_;
};


class MouseConstraint : public Constraint
{
public:

    MouseConstraint() = default;

    void turnOff() { updateBody(nullptr, Vec2{}); }
    void updateMousePos(Vec2 pos) { f_.updateMousePos(pos); }
    void updateBody(Body* body, Vec2 worldPos)
    {
        if (body == nullptr)
        {
            solver_ = std::nullopt;
        }
        else
        {
            f_.updateBody(body, worldPos);
            solver_ = S{Bodies<1>{body}, f_};
        }

        body_[0] = body;
    }

    bool shouldDie() override
    {
        if (body_[0] != nullptr && body_[0]->isDead())
            body_[0] = nullptr;

        return false;
    }

    bool isActive() override { return body_[0] != nullptr; }

    void initSolve(float dt) override
    {
        solver_.value().restitution() = Vec2::filled(restitution_);
        solver_.value().damping()     = Vec2::filled(damping_);

        solver_.value().initSolve(body_, f_, dt);
    }

    void solveVelocities(float dt) override
    {
        solver_.value().solveVelocity(body_, f_, dt);
    }

    void solvePositions() override { solver_.value().solvePosition(body_, f_); }

    BodiesView bodies() override
    {
        Body** b = &body_[0];
        return (body_[0] == nullptr) ? makeView(b, b) : body_.view();
    }
    ConstBodiesView bodies() const override
    {
        Body const** b = const_cast<Body const**>(&body_[0]);
        return (body_[0] == nullptr) ? makeView(b, b) : body_.view();
    }

    bool isBodyStructural(const Body* body) const override
    {
        return body_.isBodyStructural(body);
    }

    // TODO: Naming..
    float&       restitution() { return restitution_; }
    const float& restitution() const { return restitution_; }

    float&       damping() { return damping_; }
    const float& damping() const { return damping_; }

private:

    typedef EqualitySolver<MouseConstraintFunction> S;

    MouseConstraintFunction f_{};
    std::optional<S>        solver_ = std::nullopt;
    Bodies<1>               body_   = {nullptr};

    // TODO: Don't use optional<S>, -> solver must not require valid params in constructor.
    //  this way we can make these settings persist directly in solver
    float restitution_ = 0.2f;
    float damping_     = 0.1f;
};

} // namespace simu
