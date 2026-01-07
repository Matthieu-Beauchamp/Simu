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
#include "PhysicsObjects/ColliderPool.hpp"
#include "Settings.hpp"
#include "PhysicsObjects/ObjectPool.hpp"
#include "PhysicsObjects/DynamicPhysicsObject.hpp"
#include "PhysicsObjects/StaticPhysicsObject.hpp"
#include "collision/CollisionData.hpp"
#include "collision/CollisionPair.hpp"
#include "collision/broadphase/BoundingVolumeHierarchy.hpp"

#include <variant>

namespace simu
{

class Simulation
{
public:

    /// Construct an empty world
    Simulation() = default;

    Simulation(const Simulation& other) = delete;
    Simulation(Simulation&& other)      = delete;

    /// Makes the simulation progress in time.
    /// \param dt How much to advance the simulation (seconds)
    void step(float dt);

    /// Updates the world's settings
    void updateSettings(const Settings& settings) { _settings = settings; }

    /// Read the world's settings
    [[nodiscard]] const Settings& settings() const { return _settings; }

private:

    ObjectId get_collider_id(ObjectId object_id) const noexcept;

    void process_collisions() noexcept;

    void
    process_collision(CollisionPair pair) noexcept;

    Settings _settings;
    ObjectPool<DynamicPhysicsObject, ObjectId::DynamicPhysicsObject> dynamic_objects;
    ObjectPool<StaticPhysicsObject, ObjectId::StaticPhysicsObject> static_objects;

    ColliderPool colliders;

    std::unordered_map<CollisionPair, CollisionData> collision_pairs;

    BoundingVolumeHierarchy static_bvh;
    BoundingVolumeHierarchy dynamic_bvh;
};

} // namespace simu
