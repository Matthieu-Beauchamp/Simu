////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
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
#include "Simu/physics-2.0/collision.hpp"
#include "Simu/physics-2.0/components/Mass.hpp"
#include "Simu/physics-2.0/components/Position.hpp"
#include "Simu/physics-2.0/components/Velocity.hpp"

namespace simu
{

using Vec6 = Vector<float, 6>;

struct ObjectData
{
    Position position_a;
    Position position_b;
    Velocity velocity_a;
    Velocity velocity_b;
    Mass     mass_a;
    Mass     mass_b;
};

struct ContactConstraint2
{
    Contacts<2> contacts;
    Vec2        normal_impulses{};
    float       tangent_impulse{};
    Vec2        bias_velocities{};

    std::uint_fast8_t steps_since_contact = 0;
};

inline float relative_normal_velocity_at_contact(
    const ContactConstraint2& contact_constraint,
    const ObjectData&         object_data,
    std::uint32_t             contact_index
) {
    // (v_pb - v_pa)^T n = Jv
    Vec2 to_pa = contact_constraint.contacts.contacts_a[contact_index]
                 - object_data.position_a.position();
    Vec2 velocity_pa = object_data.velocity_a.linear
                       + object_data.velocity_a.angular * Vec2(-to_pa[1], to_pa[0]);

    Vec2 to_pb = contact_constraint.contacts.contacts_b[contact_index]
                 - object_data.position_b.position();
    Vec2 velocity_pb = object_data.velocity_b.linear
                       + object_data.velocity_b.angular * Vec2(-to_pb[1], to_pb[0]);

    return dot(velocity_pb - velocity_pa, contact_constraint.contacts.normal);
}

inline Vec6 contact_constraint_jacobian(
    const ContactConstraint2& contact_constraint,
    const ObjectData&         object_data,
    std::uint32_t             contact_index
) {
    Vec2 to_pa = contact_constraint.contacts.contacts_a[contact_index]
                 - object_data.position_a.position();

    Vec2 to_pb = contact_constraint.contacts.contacts_b[contact_index]
                 - object_data.position_b.position();

    Vec2 n = contact_constraint.contacts.normal;

    return Vec6{-n[0], -n[1], -cross(to_pa, n), n[0], n[1], cross(to_pb, n)};
}

inline float relative_tangent_velocity_at_contact(
    const ContactConstraint2& contact_constraint,
    const ObjectData&         object_data,
    std::uint32_t             contact_index
) {
    // (v_pb - v_pa)^T n = Jv
    Vec2 to_pa = contact_constraint.contacts.contacts_a[contact_index]
                 - object_data.position_a.position();
    Vec2 velocity_pa = object_data.velocity_a.linear
                       + object_data.velocity_a.angular * Vec2(-to_pa[1], to_pa[0]);

    Vec2 to_pb = contact_constraint.contacts.contacts_b[contact_index]
                 - object_data.position_b.position();
    Vec2 velocity_pb = object_data.velocity_b.linear
                       + object_data.velocity_b.angular * Vec2(-to_pb[1], to_pb[0]);

    return dot(velocity_pb - velocity_pa, perp(contact_constraint.contacts.normal));
}

inline Vec6 friction_constraint_jacobian(
    const ContactConstraint2& contact_constraint,
    const ObjectData&         object_data,
    std::uint32_t             contact_index
) {
    Vec2 to_pa = contact_constraint.contacts.contacts_a[contact_index]
                 - object_data.position_a.position();

    Vec2 to_pb = contact_constraint.contacts.contacts_b[contact_index]
                 - object_data.position_b.position();

    Vec2 t = perp(contact_constraint.contacts.normal);

    return Vec6{-t[0], -t[1], -cross(to_pa, t), t[0], t[1], cross(to_pb, t)};
}

inline Vec6 inverse_mass(const ObjectData& object_data) {
    float ma = object_data.mass_a.invMass();
    float mb = object_data.mass_b.invMass();
    float Ia = object_data.mass_a.invInertia();
    float Ib = object_data.mass_b.invInertia();
    return Vec6{ma, ma, Ia, mb, mb, Ib};
}

inline void init_contact_constraint(
    ContactConstraint2& contact_constraint,
    ObjectData&         object_data,
    float               collision_restitution,
    bool                warmstart
) {
    for (std::uint32_t i = 0; i < contact_constraint.contacts.n_contacts; ++i) {
        float relative_velocity = relative_normal_velocity_at_contact(
            contact_constraint, object_data, i
        );

        contact_constraint.bias_velocities[i] = collision_restitution * relative_velocity;
    }

    if (warmstart) {
        // TODO: Don't apply friction on warmstart, can't pull back on friction?
        auto n_contacts = contact_constraint.contacts.n_contacts;

        Vec6 J0 = n_contacts > 0
                      ? contact_constraint_jacobian(contact_constraint, object_data, 0)
                      : Vec6::filled(0.f);
        Vec6 J1 = n_contacts > 1
                      ? contact_constraint_jacobian(contact_constraint, object_data, 1)
                      : Vec6::filled(0.f);
        auto J  = Matrix<float, 2, 6>::fromRows({J0, J1});

        Vec6 inv_mass = inverse_mass(object_data);

        Vec6 friction_jacobian = n_contacts > 0
                                     ? friction_constraint_jacobian(
                                           contact_constraint, object_data, 0
                                       )
                                     : Vec6::filled(0.f);

        // Apply diff_impulses to objects
        Vec6 impulse = transpose(J) * contact_constraint.normal_impulses
                       + friction_jacobian * contact_constraint.tangent_impulse;
        Vec6 velocity_change = elementWiseMul(inv_mass, impulse);
        object_data.velocity_a.linear += Vec2(velocity_change[0], velocity_change[1]);
        object_data.velocity_a.angular += velocity_change[2];
        object_data.velocity_b.linear += Vec2(velocity_change[3], velocity_change[4]);
        object_data.velocity_b.angular += velocity_change[5];
    } else {
        contact_constraint.normal_impulses = Vec2(0, 0);
        contact_constraint.tangent_impulse = 0;
    }
}

inline void
solve_contact_constraint(ContactConstraint2& contact_constraint, ObjectData& object_data) {
    if (contact_constraint.contacts.n_contacts == 0)
        return;

    SIMU_ASSERT(
        is_approx(norm(contact_constraint.contacts.normal), 1.f, EPSILON),
        "contact normal should have unit length"
    );

    // TODO: Should store contact position relative to object?
    //      Having contact point in world space may cause problems for position correction

    if (contact_constraint.contacts.n_contacts == 1) {
        float rel_velocity = relative_normal_velocity_at_contact(
            contact_constraint, object_data, 0
        );

        // J M^-1 J^T
        Vec6 J = contact_constraint_jacobian(contact_constraint, object_data, 0);

        Vec6 inv_mass = inverse_mass(object_data);

        Vec6  impulse_direction = elementWiseMul(inv_mass, J);
        float effective_mass    = dot(J, impulse_direction);

        // J M^-1 J^T lambda = -(Jv + b), where Jv is the relative velocity
        float lambda = -(rel_velocity + contact_constraint.bias_velocities[0])
                       / effective_mass;

        float old_impulse = contact_constraint.normal_impulses[0];
        contact_constraint.normal_impulses[0] += lambda;
        contact_constraint.normal_impulses[0] = std::max(
            contact_constraint.normal_impulses[0], 0.f
        );

        float diff_impulse = contact_constraint.normal_impulses[0] - old_impulse;

        // Compute friction constraint
        float relative_tangent_velocity = relative_tangent_velocity_at_contact(
            contact_constraint, object_data, 0
        );
        Vec6 friction_jacobian = friction_constraint_jacobian(
            contact_constraint, object_data, 0
        );

        Vec6 friction_impulse_direction = elementWiseMul(inv_mass, friction_jacobian);
        float friction_effective_mass = dot(friction_jacobian, friction_impulse_direction);
        float friction_lambda = -relative_tangent_velocity / friction_effective_mass;

        float old_friction_impulse = contact_constraint.tangent_impulse;
        contact_constraint.tangent_impulse += friction_lambda;
        contact_constraint.tangent_impulse = clamp(
            contact_constraint.tangent_impulse,
            -contact_constraint.normal_impulses[0],
            contact_constraint.normal_impulses[0]
        );

        float diff_friction_impulse = contact_constraint.tangent_impulse
                                      - old_friction_impulse;

        // Apply diff_impulses to objects
        Vec6 velocity_change = impulse_direction * diff_impulse
                               + friction_impulse_direction * diff_friction_impulse;
        object_data.velocity_a.linear += Vec2(velocity_change[0], velocity_change[1]);
        object_data.velocity_a.angular += velocity_change[2];
        object_data.velocity_b.linear += Vec2(velocity_change[3], velocity_change[4]);
        object_data.velocity_b.angular += velocity_change[5];

    } else if (contact_constraint.contacts.n_contacts == 2) {
        Vec2 rel_velocities = Vec2(
            relative_normal_velocity_at_contact(contact_constraint, object_data, 0),
            relative_normal_velocity_at_contact(contact_constraint, object_data, 1)
        );

        // J M^-1 J^T
        Vec6 J0 = contact_constraint_jacobian(contact_constraint, object_data, 0);
        Vec6 J1 = contact_constraint_jacobian(contact_constraint, object_data, 1);
        auto J = Matrix<float, 2, 6>::fromRows({J0, J1});

        Vec6 inv_mass = inverse_mass(object_data);

        Mat2 effective_mass = J * Matrix<float, 6, 6>::diagonal(inv_mass)
                              * transpose(J);

        // J M^-1 J^T lambda >= -(Jv + b), where Jv is the relative velocity
        // TODO: Handle degeneracies better

        Vec2 applied_rel_velocity = effective_mass * contact_constraint.normal_impulses;
        Vec2 lambda = solveLcp(
            effective_mass,
            -(rel_velocities - applied_rel_velocity + contact_constraint.bias_velocities)
        );

        Vec2 old_impulses                  = contact_constraint.normal_impulses;
        contact_constraint.normal_impulses = max(lambda, Vec2::filled(0.f));
        Vec2 diff_impulse = contact_constraint.normal_impulses - old_impulses;

        // Compute friction constraint
        // Friction can be applied anywhere along the contact surface
        //      without affecting the result
        float relative_tangent_velocity = relative_tangent_velocity_at_contact(
            contact_constraint, object_data, 0
        );


        Vec6 friction_jacobian = friction_constraint_jacobian(
            contact_constraint, object_data, 0
        );

        float friction_effective_mass = dot(
            friction_jacobian, elementWiseMul(inv_mass, friction_jacobian)
        );

        float friction_lambda = -relative_tangent_velocity / friction_effective_mass;

        float old_friction_impulse = contact_constraint.tangent_impulse;
        contact_constraint.tangent_impulse += friction_lambda;
        contact_constraint.tangent_impulse = clamp(
            contact_constraint.tangent_impulse,
            -norm(contact_constraint.normal_impulses),
            norm(contact_constraint.normal_impulses)
        );

        float diff_friction_impulse = contact_constraint.tangent_impulse
                                      - old_friction_impulse;

        // Apply diff_impulses to objects
        Vec6 impulse = transpose(J) * diff_impulse
                       + friction_jacobian * diff_friction_impulse;

        Vec6 velocity_change = elementWiseMul(inv_mass, impulse);
        object_data.velocity_a.linear += Vec2(velocity_change[0], velocity_change[1]);
        object_data.velocity_a.angular += velocity_change[2];
        object_data.velocity_b.linear += Vec2(velocity_change[3], velocity_change[4]);
        object_data.velocity_b.angular += velocity_change[5];
    }
}

} // namespace simu
