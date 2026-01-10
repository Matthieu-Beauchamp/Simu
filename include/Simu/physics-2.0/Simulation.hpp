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

namespace simu
{

class Simulation
{
    using DynamicObjects = ObjectPool<DynamicPhysicsObject, ObjectId::DynamicPhysicsObject>;
    using StaticObjects = ObjectPool<StaticPhysicsObject, ObjectId::StaticPhysicsObject>;
    using Collisions = std::unordered_map<CollisionPair, ContactConstraint2>;
    using ContactPointer = decltype(std::declval<Collisions>().begin());

public:

    /// Construct an empty world
    Simulation() = default;

    // TODO: no reason to not be at least movable
    Simulation(const Simulation& other) = delete;
    Simulation(Simulation&& other)      = delete;

    /// Makes the simulation progress in time.
    void step() SIMU_NO_EXCEPT;

    /// The simulation's settings
    [[nodiscard]] Settings&       settings() { return _settings; }
    [[nodiscard]] const Settings& settings() const { return _settings; }

    /// Reset the simulation
    void clear() {
        _dynamic_objects = DynamicObjects{};
        _static_objects  = StaticObjects{};
        _colliders       = ColliderPool{};
        _collision_pairs.clear();
        _dynamic_bvh = BoundingVolumeHierarchy{};
        _static_bvh  = BoundingVolumeHierarchy{};
    }

    //////////////////////////////////////////////////
    // Objects

    /// Create a new object
    ObjectId create_object(ObjectBuilder builder);

    /// Destroy an object
    void destroy_object(ObjectId) { NOT_IMPLEMENTED; }

    auto dynamic_objects() SIMU_NO_EXCEPT { return _dynamic_objects.objects(); }
    auto dynamic_objects() const SIMU_NO_EXCEPT {
        return _dynamic_objects.objects();
    }

    auto static_objects() SIMU_NO_EXCEPT { return _static_objects.objects(); }
    auto static_objects() const SIMU_NO_EXCEPT {
        return _static_objects.objects();
    }

    DynamicPhysicsObject& get_dynamic_object(ObjectId id) SIMU_NO_EXCEPT {
        return _dynamic_objects[id];
    }
    const DynamicPhysicsObject& get_dynamic_object(ObjectId id) const SIMU_NO_EXCEPT {
        return _dynamic_objects[id];
    }

    StaticPhysicsObject& get_static_object(ObjectId id) SIMU_NO_EXCEPT {
        return _static_objects[id];
    }
    const StaticPhysicsObject& get_static_object(ObjectId id) const SIMU_NO_EXCEPT {
        return _static_objects[id];
    }

    //////////////////////////////////////////////////
    // Colliders

    [[nodiscard]] ColliderType collider_type(ObjectId id) const SIMU_NO_EXCEPT {
        SIMU_ASSERT(id.type() == ObjectId::Collider, "Invalid collider id");
        return _colliders.get_type(id);
    }

    Circle& get_circle(ObjectId id) SIMU_NO_EXCEPT {
        return _colliders.circle(id);
    }
    const Circle& get_circle(ObjectId id) const SIMU_NO_EXCEPT {
        return _colliders.circle(id);
    }

    Capsule& get_capsule(ObjectId id) SIMU_NO_EXCEPT {
        return _colliders.capsule(id);
    }
    const Capsule& get_capsule(ObjectId id) const SIMU_NO_EXCEPT {
        return _colliders.capsule(id);
    }

    Polygon& get_polygon(ObjectId id) SIMU_NO_EXCEPT {
        return _colliders.polygon(id);
    }
    const Polygon& get_polygon(ObjectId id) const SIMU_NO_EXCEPT {
        return _colliders.polygon(id);
    }

    //////////////////////////////////////////////////
    // Constraints

    auto contact_constraints() SIMU_NO_EXCEPT {
        return std::ranges::subrange{_collision_pairs.begin(), _collision_pairs.end()};
    }
    auto contact_constraints() const SIMU_NO_EXCEPT {
        return std::ranges::subrange{_collision_pairs.begin(), _collision_pairs.end()};
    }

private:

    [[nodiscard]] Position get_position(ObjectId object_id) const;
    [[nodiscard]] ObjectId get_collider_id(ObjectId object_id) const SIMU_NO_EXCEPT;

    void process_collisions() SIMU_NO_EXCEPT;

    void process_collision(CollisionPair pair) SIMU_NO_EXCEPT;

    void solve_contacts(const std::vector<ContactPointer>& contacts) noexcept;
    void solve_contact_positions(std::vector<ContactPointer>& contacts) noexcept;

    [[nodiscard]] ObjectData get_object_data(CollisionPair pair) const noexcept;
    void write_back_velocities(CollisionPair pair, const ObjectData&) noexcept;
    void write_back_positions(CollisionPair pair, const ObjectData&) noexcept;

    Settings       _settings;
    DynamicObjects _dynamic_objects{};
    StaticObjects  _static_objects{};

    ColliderPool _colliders;

    // TODO: Could store in sorted array using id = a * 2^32 + b
    // This could provide a better performance even if lookup is log(n)
    // Otherwise consider using a probing hashmap instead of std::
    Collisions _collision_pairs;

    BoundingVolumeHierarchy _static_bvh;
    BoundingVolumeHierarchy _dynamic_bvh;
};

} // namespace simu
