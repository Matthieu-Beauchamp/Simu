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
#include "Settings.hpp"
#include "Simu/entities/Entities.hpp"
#include "components/Mass.hpp"
#include "components/Position.hpp"
#include "components/Velocity.hpp"

namespace simu
{

class Simulation
{
public:

    using EntitiesType = Entities<Position, Velocity, Mass>;

    /// Construct an empty world
    Simulation() = default;

    Simulation(const Simulation& other) = delete;
    Simulation(Simulation&& other)      = delete;

    const EntitiesType& entities() const { return _entities; }
    EntitiesType&       entities() { return _entities; }

    /// Makes the simulation progress in time.
    /// \param dt How much to advance the simulation (seconds)
    void step(float dt);

    /// Updates the world's settings
    void updateSettings(const Settings& settings) { _settings = settings; }

    /// Read the world's settings
    const Settings& settings() const { return _settings; }

private:

    Settings     _settings;
    EntitiesType _entities;
};


} // namespace simu
