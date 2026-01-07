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

namespace simu
{

/// Mass properties of a Body
class Mass
{
    static constexpr float inf = std::numeric_limits<float>::infinity();

    static constexpr float safe_inv(float denum) {
        return denum == 0.f ? inf : 1.f / denum;
    }

public:

    /// @return a mass object describing an object that cannot be moved
    static Mass structural() { return Mass{inf, inf}; }

    /// @param mass the mass of the object (>= 0)
    /// @param inertia the inertia of the object (>= 0)
    Mass(float mass, float inertia)
        : _invMass{safe_inv(mass)}, _invInertia{safe_inv(inertia)} {
        SIMU_ASSERT(mass >= 0, "Invalid mass");
        SIMU_ASSERT(inertia >= 0, "Invalid inertia");
    }

    /// @return The inverse mass of the object
    float invMass() const { return _invMass; }

    /// @return The inverse inertia of the object
    float invInertia() const { return _invInertia; }

    /// @return The mass of the object
    float mass() const { return safe_inv(invMass()); }

    /// @return The inertia of the object
    float inertia() const { return safe_inv(invInertia()); }

private:

    float _invMass;
    float _invInertia;
};

} // namespace simu
