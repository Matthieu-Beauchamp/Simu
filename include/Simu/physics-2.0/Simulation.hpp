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
#include "EntitiesType.hpp"
#include "collision/broadphase/BoundingVolumeHierarchy.hpp"

namespace simu
{
struct CollisionData;
class CollisionPair;

class Simulation
{
public:

    /// Construct an empty world
    Simulation() = default;

    Simulation(const Simulation& other) = delete;
    Simulation(Simulation&& other)      = delete;

    [[nodiscard]] const EntitiesType& entities() const { return _entities; }
    EntitiesType&                     entities() { return _entities; }

    /// Makes the simulation progress in time.
    /// \param dt How much to advance the simulation (seconds)
    void step(float dt);

    /// Updates the world's settings
    void updateSettings(const Settings& settings) { _settings = settings; }

    /// Read the world's settings
    [[nodiscard]] const Settings& settings() const { return _settings; }

private:

    void process_collisions(
        const EntitiesType& entities,
        float               epsilon,
        std::uint_fast8_t   max_steps_since_contacts
    ) noexcept;

    void
    process_collision(const EntitiesType& entities, CollisionPair pair, float epsilon) noexcept;

    Settings         _settings;
    EntitiesType     _entities; // TODO: Huge object

    std::unordered_map<CollisionPair, CollisionData> collision_pairs;

    BoundingVolumeHierarchy static_objects;
    BoundingVolumeHierarchy dynamic_objects;
};

} // namespace simu
