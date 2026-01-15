////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2025 Matthieu Beauchamp-Boulay
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
#include "Simu/math/Matrix.hpp"

namespace simu
{
struct Settings
{
    /// Time step of the simulation
    float dt = 1.f / 60.f;

    /// Gravity to apply to the simulation
    Vec2 gravity = Vec2{0.f, -10.f};

    /// Number of velocity solver iterations
    Uint32 n_velocity_iterations = 8;

    /// Number of position solver iterations
    Uint32 n_position_iterations = 2;

    /// How much of the position error is corrected at each iteration
    /// using position projection
    float position_correction_factor = 0.1f;

    /// The allowed penetration for a contact point.
    float penetration_slop = 1e-4f;

    /// The constraint force mixing (CFM) parameter.
    /// A value of 0 is a hard constraint, increase it to soften contacts
    float contact_softness = 1e-5f;

    /// The error reduction parameter (ERP).
    /// This converts a part of the penetration to corrective velocity.
    /// This is disabled by default, and the position projection is used instead.
    ///
    /// Set this to a value > 0 and set position iterations to 0 to use Baumgarte stabilization
    float contact_error_reduction = 0.f;

    /// Number of physics steps until non-touching collisions are removed from the cache
    Uint8 n_steps_without_contacts = 10;

    /// Enable constraints to guess impulse based on the previous step.
    bool enable_warm_starting = true;
};

} // namespace simu