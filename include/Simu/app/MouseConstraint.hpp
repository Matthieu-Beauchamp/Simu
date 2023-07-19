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

    MouseConstraintFunction(Body* body, Vec2 pos) : Base{}
    {
        localBodyPos_ = body->toLocalSpace() * pos;
        mousePos_ = pos;
    }

    Value eval(Bodies<1> body) const
    {
        return body[0]->toWorldSpace() * localBodyPos_ - mousePos_;
    }

    Value bias(Bodies<1> /* body */) const { return Value{}; }


    Jacobian jacobian(Bodies<1> body) const
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

    Vec2 mousePos() const { return mousePos_; }
    Vec2 localBodyPos() const { return localBodyPos_; }

    void updateMousePos(Vec2 pos) { mousePos_ = pos; }


private:

    Vec2 mousePos_;
    Vec2 localBodyPos_;
};


class MouseConstraint : public ConstraintImplementation<MouseConstraintFunction>
{
    typedef ConstraintImplementation<MouseConstraintFunction> Base;

public:

    MouseConstraint(Body* body, Vec2 pos)
        : Base{
            Bodies<1>{body},
            MouseConstraintFunction{body, pos},
            false
    }
    {
        solver.restitution() = Value::filled(0.1f);
        solver.damping()     = Value::filled(1.f);
    }

    Vec2 bodyPos() const { return getBodies()[0]->toWorldSpace() * f.localBodyPos(); }
    Vec2 mousePos() const { return f.mousePos(); }

    void updateMousePos(Vec2 pos) { f.updateMousePos(pos); }

    void solvePositions() override {}
};

} // namespace simu
