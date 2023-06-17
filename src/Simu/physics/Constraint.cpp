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

#include "Simu/physics/Constraint.hpp"

namespace simu
{

template <>
SingleContactFunction::Value
SingleContactFunction::clampLambda(const Value& lambda, float dt) const
{
    ConstraintValue<1> lambdaNormal
        = std::get<0>(constraints).clampLambda(ConstraintValue<1>{lambda[0]}, dt);
    return Value{
        lambdaNormal[0],
        std::get<1>(constraints)
            .clampLambda(ConstraintValue<1>{lambda[1]}, dt, lambdaNormal)[0]};
}

template <>
DoubleContactFunction::Value
DoubleContactFunction::clampLambda(const Value& lambda, float dt) const
{
    ConstraintValue<2> lambdaNormal{
        std::get<0>(constraints).clampLambda(ConstraintValue<1>{lambda[0]}, dt)[0],
        std::get<1>(constraints).clampLambda(ConstraintValue<1>{lambda[1]}, dt)[0]};

    return Value{
        lambdaNormal[0],
        lambdaNormal[1],
        std::get<2>(constraints)
            .clampLambda(ConstraintValue<1>{lambda[2]}, dt, lambdaNormal)[0]};
}

} // namepace simu 
