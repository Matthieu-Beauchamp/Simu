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
#include "Simu/physics/CombinableProperty.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief Defines the material of a Collider
///
////////////////////////////////////////////////////////////
struct Material
{
    ////////////////////////////////////////////////////////////
    /// The weight per unit of area, must be greater than 0
    ////////////////////////////////////////////////////////////
    float density = 1.f;

    ////////////////////////////////////////////////////////////
    /// How much the body will bounce when colliding with another.
    ///
    /// The value should be in [0, 1].
    /// A value of 0 means the object does not bounce at all (all energy is
    /// lost) A value of 1 means the object fully bounces (no energy is lost)
    ///
    /// behavior is undefined if e <= -1
    ///
    /// This value is combined to obtain a coefficient of restitution:
    /// https://en.wikipedia.org/wiki/Coefficient_of_restitution
    ////////////////////////////////////////////////////////////
    CombinableProperty bounciness{0.f, CombinableProperty::average};

    ////////////////////////////////////////////////////////////
    /// How much the body resists to sliding against another.
    ///
    /// The value will typically be in [0, 1].
    /// A value of 0 implies a very slippery object (ice)
    ///
    /// This value is combined to obtain a coefficient of friction:
    /// https://simple.wikipedia.org/wiki/Coefficient_of_friction
    ////////////////////////////////////////////////////////////
    CombinableProperty friction{0.f, CombinableProperty::average};

    ////////////////////////////////////////////////////////////
    /// How much penetration the body tolerates during collisions.
    /// 
    /// This value should be small enough that there is no visible overlapping,
    ///     while being big enough relative to a Body's geometry to compute correct ContactManifolds.
    /// 
    /// It is also used to determine if a ContactManifold is still valid. 
    ///     if the contact points between two bodies are closer than penetration,
    ///     the contact is considered valid.
    /// 
    /// This means that the Body's geometry's vertices should have a distance of more  
    ///     than twice the Material's penetration between them.
    /// 
    ////////////////////////////////////////////////////////////
    CombinableProperty penetration{0.005f, CombinableProperty::average};
};


} // namespace simu
