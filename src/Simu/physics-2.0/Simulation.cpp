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

#include "Simu/physics-2.0/Simulation.hpp"

#include "Simu/physics-2.0/components/Velocity.hpp"
#include "../../../include/Simu/physics-2.0/PhysicsObjects/ColliderOperations.hpp"
#include "physics/Collider.hpp"
#include "Simu/physics-2.0/collision.hpp"

#include <utility>

namespace simu
{

namespace
{

[[nodiscard]] Contacts<2> map_contacts(const Contacts<1>& contacts) {
    return Contacts<2>{
        .normal     = contacts.normal,
        .contacts_a = {contacts.contacts_a[0]},
        .contacts_b = {contacts.contacts_b[0]},
        .n_contacts = contacts.n_contacts
    };
}

[[nodiscard]] Contacts<2> inverse_contacts(const Contacts<2>& contacts) {
    return Contacts<2>{
        .normal     = -contacts.normal,
        .contacts_a = contacts.contacts_b,
        .contacts_b = contacts.contacts_a,
        .n_contacts = contacts.n_contacts
    };
}

/**
 * @param a The first collider id
 * @param b The second collider id
 * @param colliders the collider pool to use
 * @param epsilon tolerance to determine if two points are different contacts
 * @return The resulting contacts
 */
[[nodiscard]] Contacts<2>
collide(ObjectId a, ObjectId b, const ColliderPool& colliders, float epsilon = CONTACT_EPSILON) SIMU_NO_EXCEPT {
    ColliderType type_a = colliders.get_type(a);
    ColliderType type_b = colliders.get_type(b);

    switch (type_a) {
        case ColliderType::Circle:
        {
            const Circle& a_collider = colliders.circle(a);
            switch (type_b) {
                case ColliderType::Circle:
                {
                    const auto& b_collider = colliders.circle(b);
                    return map_contacts(collide(a_collider, b_collider));
                }
                case ColliderType::Capsule:
                {
                    const auto& b_collider = colliders.capsule(b);
                    return map_contacts(collide(a_collider, b_collider));
                }
                case ColliderType::Polygon:
                {
                    const auto& b_collider = colliders.polygon(b);
                    return map_contacts(collide(a_collider, b_collider));
                }
            }
        }
        case ColliderType::Capsule:
        {
            const Capsule& a_collider = colliders.capsule(a);
            switch (type_b) {
                case ColliderType::Circle:
                {
                    const auto& b_collider = colliders.circle(b);
                    return inverse_contacts(map_contacts(collide(b_collider, a_collider)));
                }
                case ColliderType::Capsule:
                {
                    const auto& b_collider = colliders.capsule(b);
                    return collide(a_collider, b_collider, epsilon);
                }
                case ColliderType::Polygon:
                {
                    const auto& b_collider = colliders.polygon(b);
                    return collide(a_collider, b_collider, epsilon);
                }
            }
        }
        case ColliderType::Polygon:
        {
            const Polygon& a_collider = colliders.polygon(a);
            switch (type_b) {
                case ColliderType::Circle:
                {
                    const auto& b_collider = colliders.circle(b);
                    return inverse_contacts(map_contacts(collide(b_collider, a_collider)));
                }
                case ColliderType::Capsule:
                {
                    const auto& b_collider = colliders.capsule(b);
                    return inverse_contacts(collide(b_collider, a_collider, epsilon));
                }
                case ColliderType::Polygon:
                {
                    const auto& b_collider = colliders.polygon(b);
                    return collide(a_collider, b_collider, epsilon);
                }
            }
        }
    }

    SIMU_ASSERT(false, "Missing collision combination");
}

} // namespace

void Simulation::step() {
    // TODO: In a separate thread, create improved tree to be used in the next timestep
    //      instead of waiting on it for the current step.
    //      Use old tree for current step.

    float dt = _settings.dt;

    {
        std::vector<ObjectId>    dynamic_objects_ids;
        std::vector<BoundingBox> dynamic_objects_boxes;
        for (DynamicPhysicsObject& object : dynamic_objects.objects()) {
            dynamic_objects_ids.push_back(object.id);
            dynamic_objects_boxes.emplace_back(
                bounding_box(object.collider_id, colliders, object.position)
            );
        }

        dynamic_bvh = BoundingVolumeHierarchy::mean_centroid_split(
            dynamic_objects_ids, dynamic_objects_boxes
        );
    }

    {
        // TODO: Compute in init and persist unless modified

        std::vector<ObjectId>    static_objects_ids;
        std::vector<BoundingBox> static_objects_boxes;
        for (StaticPhysicsObject& object : static_objects.objects()) {
            static_objects_ids.push_back(object.id);
            static_objects_boxes.emplace_back(bounding_box(object.collider_id, colliders));
        }

        static_bvh = BoundingVolumeHierarchy::mean_centroid_split(
            static_objects_ids, static_objects_boxes
        );
    }

    // Step velocity according to gravity
    Vec2 gravity = _settings.gravity * dt;
    for (DynamicPhysicsObject& object : dynamic_objects.objects()) {
        object.velocity.linear += gravity;
    }

    process_collisions();

    solve_contacts();

    // TODO: Solve other constraints

    // Step position according to resolved velocities
    for (DynamicPhysicsObject& object : dynamic_objects.objects()) {
        object.position.advance(object.velocity.linear * dt, object.velocity.angular * dt);
        if (object.collider_id) {}
    }

    // TODO: In bounding volume hierarchy, update and mark as dirty
    //      Can iterate over tree nodes and map to the object quickly with its
    //      id, but can't walk back up the tree.
    //      => Can do a postorder traversal update
}

ObjectId Simulation::get_collider_id(ObjectId object_id) const SIMU_NO_EXCEPT {
    switch (object_id.type()) {
        case ObjectId::DynamicPhysicsObject:
            return dynamic_objects[object_id].collider_id;
        case ObjectId::StaticPhysicsObject:
            return static_objects[object_id].collider_id;
    }

    SIMU_ASSERT(false, "Unexpected object type");
}

void Simulation::process_collisions() noexcept {
    {
        // TODO: Keep for polygons where the normal gives the separating axis.
        std::vector<CollisionPair> outdated;
        for (auto it = collision_pairs.begin(); it != collision_pairs.end(); it++) {
            if (it->second.steps_since_contact++ > _settings.n_steps_without_contacts) {
                outdated.push_back(it->first);
            }
        }

        for (CollisionPair& pair : outdated) {
            collision_pairs.erase(pair);
        }
    }

    dynamic_bvh.collide(dynamic_bvh, [this](ObjectId a, ObjectId b) noexcept {
        // When colliding with the same tree, collisions are detected twice.
        // Also ignore collision with self
        if (a.as_index() >= b.as_index()) {
            return;
        }

        process_collision(CollisionPair(a, b));
    });

    dynamic_bvh.collide(static_bvh, [this](ObjectId a, ObjectId b) noexcept {
        process_collision(CollisionPair(a, b));
    });
}

void Simulation::process_collision(CollisionPair pair) noexcept {
    Contacts<2> contacts = collide(
        get_collider_id(pair.a), get_collider_id(pair.b), colliders
    );

    if (contacts.n_contacts == 0) {
        return;
    }

    auto collision = collision_pairs.find(pair);

    if (collision != collision_pairs.end()) {
        collision->second.steps_since_contact = 0;
    } else {
        collision_pairs.emplace(pair, ContactConstraint2{.contacts = contacts});
    }

    // TODO: Update disjoint set for islands
}

void Simulation::solve_contacts() noexcept {
    using ContactPointer = decltype(collision_pairs.begin());
    std::vector<ContactPointer> constraints;
    constraints.reserve(collision_pairs.size());

    // TODO: Process into islands

    for (auto it = collision_pairs.begin(); it != collision_pairs.end(); it++) {
        if (it->second.contacts.n_contacts > 0) {
            constraints.push_back(it);
        }
    }

    for (const auto& it : constraints) {
        ContactConstraint2& constraint = it->second;
        ObjectData          data       = get_object_data(it->first);

        // TODO: add materials to get restitution and friction coeff
        init_contact_constraint(constraint, data, 0, _settings.enable_warm_starting);
        write_back(it->first, data);
    }

    // TODO: Add stop when stable
    // TODO: Don't apply impulses below some threshold
    for (std::uint32_t i = 0; i < _settings.n_velocity_iterations; i++) {
        for (const auto& it : constraints) {
            ContactConstraint2& constraint = it->second;
            ObjectData          data       = get_object_data(it->first);
            solve_contact_constraint(constraint, data);
            write_back(it->first, data);
        }
    }
}

ObjectData Simulation::get_object_data(CollisionPair pair) const noexcept {
    ObjectData data;
    if (pair.a.type() == ObjectId::DynamicPhysicsObject) {
        data.position_a = dynamic_objects[pair.a].position;
        data.velocity_a = dynamic_objects[pair.a].velocity;
        data.mass_a     = dynamic_objects[pair.a].mass;
    } else {
        data.position_a = static_objects[pair.a].position;
        data.velocity_a = Velocity{};
        data.mass_a     = Mass::structural();
    }

    if (pair.b.type() == ObjectId::DynamicPhysicsObject) {
        data.position_b = dynamic_objects[pair.b].position;
        data.velocity_b = dynamic_objects[pair.b].velocity;
        data.mass_b     = dynamic_objects[pair.b].mass;
    } else {
        data.position_b = static_objects[pair.b].position;
        data.velocity_b = Velocity{};
        data.mass_b     = Mass::structural();
    }

    return data;
}

void Simulation::write_back(CollisionPair pair, const ObjectData& data) noexcept {
    if (pair.a.type() == ObjectId::DynamicPhysicsObject) {
        dynamic_objects[pair.a].velocity = data.velocity_a;
    }

    if (pair.b.type() == ObjectId::DynamicPhysicsObject) {
        dynamic_objects[pair.b].velocity = data.velocity_b;
    }
}

ObjectId Simulation::create_object(ObjectBuilder builder) {
    SIMU_ASSERT(builder.has_collider_, "Object has no collider");

    Mass mass = builder.mass_;
    if (builder.compute_mass_from_geometry) {
        float density = builder.density_;
        switch (builder.collider_type_) {
            case ColliderType::Circle:
            {
                float r_squared = builder.circle_.radius() * builder.circle_.radius();
                float m       = std::numbers::pi_v<float> * r_squared * density;
                float inertia = 0.5f * m * r_squared;
                mass          = Mass{m, inertia};
                break;
            }
            case ColliderType::Capsule:
            {
                float radius = builder.capsule_.radius();
                float length = norm(builder.capsule_.top() - builder.capsule_.bottom());
                float pi = std::numbers::pi_v<float>;

                float m_r = density * 2.f * length * radius;
                float m_c = density * pi * radius * radius;

                float rect_inertia = m_r * (4 * radius * radius + length * length) / 12;
                float circ_inertia = m_c * 0.5f * radius * radius
                                     + m_c * (length * 0.5f) * (length * 0.5f);
                mass = Mass{m_r + m_c, rect_inertia + circ_inertia};
                break;
            }
            case ColliderType::Polygon:
            {
                auto properties = GeometricProperties(builder.polygon_);
                mass = Mass{properties.area * density, properties.momentOfArea * density};
                break;
            }
        }
    }

    ObjectId collider_id = colliders.allocate(builder.collider_type_);
    if (builder.is_static_) {
        // Store colliders in world space for static objects
        switch (builder.collider_type_) {
            case ColliderType::Circle:
                colliders.circle(collider_id) = builder.position_.toWorldSpace()
                                                * builder.circle_;
                break;
            case ColliderType::Capsule:
                colliders.capsule(collider_id) = builder.position_.toWorldSpace()
                                                 * builder.capsule_;
                break;
            case ColliderType::Polygon:
                colliders.polygon(collider_id) = builder.position_.toWorldSpace()
                                                 * builder.polygon_;
                break;
        }

        ObjectId id        = static_objects.allocate();
        static_objects[id] = StaticPhysicsObject{
            .id          = id,
            .position    = builder.position_,
            .collider_id = collider_id,
        };

        return id;
    } else {
        switch (builder.collider_type_) {
            case ColliderType::Circle:
                colliders.circle(collider_id) = builder.circle_;
                break;
            case ColliderType::Capsule:
                colliders.capsule(collider_id) = builder.capsule_;
                break;
            case ColliderType::Polygon:
                colliders.polygon(collider_id) = builder.polygon_;
                break;
        }

        ObjectId id         = dynamic_objects.allocate();
        dynamic_objects[id] = DynamicPhysicsObject{
            .id          = id,
            .position    = builder.position_,
            .velocity    = builder.velocity_,
            .mass        = mass,
            .collider_id = collider_id,
        };

        return id;
    }
}

} // namespace simu
