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

namespace simu
{


////////////////////////////////////////////////////////////
/// \brief A property that can be combined according to some Mode
///
/// Some values are typically given for a system (collision restitution and friction coefficients),
///     this class allows defining them per object and then combining them to
///     obtain a value for the system.
///
/// Conception taken from https://docs.unity3d.com/Manual/class-PhysicMaterial.html
///
////////////////////////////////////////////////////////////
struct CombinableProperty
{
    ////////////////////////////////////////////////////////////
    /// \brief The Mode defines how the property is combined.
    ///
    /// When the Mode of two CombinableProperty is different, 
    ///     the one with the highest value (priority) is taken.
    ////////////////////////////////////////////////////////////
    enum Mode
    {
        average  = 0,
        minimum  = 1,
        multiply = 2,
        maximum  = 3
    };

    explicit CombinableProperty(float value, Mode mode = average)
        : mode{mode}, value{value}
    {
    }

    CombinableProperty(CombinableProperty first, CombinableProperty second)
        : mode{static_cast<Mode>(std::max(first.mode, second.mode))},
          value{combine(mode, first.value, second.value)}
    {
    }

    static float combine(Mode mode, float first, float second)
    {
        switch (mode)
        {
            case average: return (first + second) / 2;
            case minimum: return std::min(first, second);
            case multiply: return first * second;
            case maximum: return std::max(first, second);
            default: return 0.f;
        }
    }

    Mode  mode;
    float value;
};


} // namespace simu
