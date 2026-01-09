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
#include "ObjectBuilder.hpp"
#include "PhysicsObjects/ColliderPool.hpp"
#include "Settings.hpp"
#include "PhysicsObjects/ObjectPool.hpp"
#include "PhysicsObjects/DynamicPhysicsObject.hpp"
#include "PhysicsObjects/StaticPhysicsObject.hpp"
#include "Simu/physics-2.0/collision.hpp"
#include "constraint/contact.hpp"

#include <complex.h>

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
    void step();

    /// Updates the world's settings
    void update_settings(const Settings& settings) { _settings = settings; }

    /// Read the world's settings
    [[nodiscard]] const Settings& settings() const { return _settings; }

    /// Create a new object
    ObjectId create_object(ObjectBuilder builder);

private:

    [[nodiscard]] ObjectId get_collider_id(ObjectId object_id) const SIMU_NO_EXCEPT;

    void process_collisions() noexcept;

    void process_collision(CollisionPair pair) noexcept;

    void       solve_contacts() noexcept;
    [[nodiscard]] ObjectData get_object_data(CollisionPair pair) const noexcept;
    void       write_back(CollisionPair pair, const ObjectData&) noexcept;

    Settings _settings;
    ObjectPool<DynamicPhysicsObject, ObjectId::DynamicPhysicsObject> dynamic_objects{};
    ObjectPool<StaticPhysicsObject, ObjectId::StaticPhysicsObject> static_objects{};

    ColliderPool colliders;

    // TODO: Could store in sorted array using id = a * 2^32 + b
    // This could provide a better performance even if lookup is log(n)
    // Otherwise consider using a probing hashmap instead of std::
    std::unordered_map<CollisionPair, ContactConstraint2> collision_pairs;

    BoundingVolumeHierarchy static_bvh;
    BoundingVolumeHierarchy dynamic_bvh;
};

} // namespace simu
