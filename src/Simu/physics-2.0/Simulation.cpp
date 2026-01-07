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
#include "physics-2.0/collision/CollisionData.hpp"
#include "physics-2.0/collision/CollisionPair.hpp"
#include "physics-2.0/collision/colliders/collisions.hpp"

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
 * @param a The first entity
 * @param b The second entity
 * @param entities The components for all entities
 * @param epsilon tolerance to determine if two points are different contacts
 * @return The resulting contacts
 */
[[nodiscard]] Contacts<2>
collide(Entity a, Entity b, const EntitiesType& entities, float epsilon) noexcept {
    SIMU_ASSERT(
        entities.get_component<ColliderType>().has_entity(a), "Entity should have a collider"
    );
    SIMU_ASSERT(
        entities.get_component<ColliderType>().has_entity(b), "Entity should have a collider"
    );

    ColliderType type_a = entities.get_component<ColliderType>().get_data(a);
    ColliderType type_b = entities.get_component<ColliderType>().get_data(b);

    switch (type_a) {
        case ColliderType::Circle:
        {
            const Circle& a_collider = entities.get_component<Circle>().get_data(a);
            switch (type_b) {
                case ColliderType::Circle:
                {
                    const auto& b_collider = entities.get_component<Circle>().get_data(b);
                    return map_contacts(collide(a_collider, b_collider));
                }
                case ColliderType::Capsule:
                {
                    const auto& b_collider = entities.get_component<Capsule>().get_data(b);
                    return map_contacts(collide(a_collider, b_collider));
                }
                case ColliderType::Polygon:
                {
                    const auto& b_collider = entities.get_component<Polygon>().get_data(b);
                    return map_contacts(collide(a_collider, b_collider));
                }
            }
        }
        case ColliderType::Capsule:
        {
            const Capsule& a_collider = entities.get_component<Capsule>().get_data(a);
            switch (type_b) {
                case ColliderType::Circle:
                {
                    const auto& b_collider = entities.get_component<Circle>().get_data(b);
                    return inverse_contacts(map_contacts(collide(b_collider, a_collider)));
                }
                case ColliderType::Capsule:
                {
                    const auto& b_collider = entities.get_component<Capsule>().get_data(b);
                    return collide(a_collider, b_collider, epsilon);
                }
                case ColliderType::Polygon:
                {
                    const auto& b_collider = entities.get_component<Polygon>().get_data(b);
                    return collide(a_collider, b_collider, epsilon);
                }
            }
        }
        case ColliderType::Polygon:
        {
            const Polygon& a_collider = entities.get_component<Polygon>().get_data(a);
            switch (type_b) {
                case ColliderType::Circle:
                {
                    const auto& b_collider = entities.get_component<Circle>().get_data(b);
                    return inverse_contacts(map_contacts(collide(b_collider, a_collider)));
                }
                case ColliderType::Capsule:
                {
                    const auto& b_collider = entities.get_component<Capsule>().get_data(b);
                    return inverse_contacts(collide(b_collider, a_collider, epsilon));
                }
                case ColliderType::Polygon:
                {
                    const auto& b_collider = entities.get_component<Polygon>().get_data(b);
                    return collide(a_collider, b_collider, epsilon);
                }
            }
        }
    }

    SIMU_ASSERT(false, "Missing collision combination");
}

} // namespace

void Simulation::process_collisions(
    const EntitiesType& entities,
    float               epsilon,
    std::uint_fast8_t   max_steps_since_contacts
) noexcept {
    {
        // TODO: Keep for polygons where the normal gives the separating axis.
        std::vector<CollisionPair> outdated;
        for (auto it = collision_pairs.begin(); it != collision_pairs.end(); it++) {
            if (it->second.steps_since_contact++ > max_steps_since_contacts) {
                outdated.push_back(it->first);
            }
        }

        for (CollisionPair& pair : outdated) {
            collision_pairs.erase(pair);
        }
    }

    dynamic_objects.collide(dynamic_objects, [&entities, this, epsilon](Entity a, Entity b) noexcept {
        // When colliding with the same tree, collisions are detected twice.
        // Also ignore collision with self
        if (a.id() >= b.id()) {
            return;
        }

        process_collision(entities, CollisionPair(a, b), epsilon);
    });

    dynamic_objects.collide(static_objects, [&entities, this, epsilon](Entity a, Entity b) noexcept {
        process_collision(entities, CollisionPair(a, b), epsilon);
    });
}

void Simulation::process_collision(
    const EntitiesType& entities,
    CollisionPair       pair,
    float               epsilon
) noexcept {
    // TODO: Transform?
    Contacts<2> contacts = collide(pair.a, pair.b, entities, epsilon);

    if (contacts.n_contacts == 0) {
        return;
    }

    auto collision = collision_pairs.find(pair);

    if (collision != collision_pairs.end()) {
        collision->second.steps_since_contact = 0;
    } else {
        collision_pairs.emplace(
            pair, CollisionData{.contacts = contacts, .steps_since_contact = 0}
        );
    }

    // TODO: Create constraint
    // Update disjoint set for islands
}

void Simulation::step(float dt) {
    // TODO: In a separate thread, create improved tree to be used in the next timestep
    //      instead of waiting on it for the current step.
    //      Use old tree for current step.

    std::vector<Entity> dynamic_entities;
    _entities.query<Position, Circle>().each([](Entity entity, const Position&, const Circle& circle) {});


    // Step velocity according to gravity
    Vec2 gravity = _settings.gravity * dt;
    _entities.query<Velocity>().each([gravity](Velocity& vel) {
        vel.linear += gravity;
    });

    // TODO: Apply constraints

    // Step position according to resolved velocities
    _entities.query<Position, Velocity>().each(
        [dt](Position& pos, const Velocity& vel) {
            pos.advance(vel.linear * dt, vel.angular * dt);
        }
    );

    // TODO: In bounding volume hierarchy, update and mark as dirty
}

} // namespace simu
