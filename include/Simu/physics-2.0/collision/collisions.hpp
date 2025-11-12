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
#include "Simu/math/Matrix.hpp"
#include "colliders/BoundingBox.hpp"
#include "colliders/Polygon.hpp"
#include "colliders/Circle.hpp"
#include "colliders/Capsule.hpp"

namespace simu
{

inline bool collides(const BoundingBox& a, const BoundingBox& b) {
    if (!a.isValid() || !b.isValid())
        return false;

    return (
        a.max()[0] >= b.min()[0] && b.max()[0] >= a.min()[0]
        && a.max()[1] >= b.min()[1] && b.max()[1] >= a.min()[1]
    );
}

template <std::size_t max_contacts>
struct Contacts
{
    static constexpr Contacts none() { return Contacts{}; }

    /// The unit normal pointing out of the `a` body
    Vec2 normal;

    /// The contacts on body a
    std::array<Vec2, max_contacts> contacts_a;
    /// The contacts on body b
    std::array<Vec2, max_contacts> contacts_b;

    /// Actual number of contacts, 0 if no collision
    Uint32 n_contacts = 0;
};

inline Contacts<1> collides(const Circle& a, const Circle& b) {
    float min_dist = a.radius() + b.radius();
    Vec2  dir      = b.center() - a.center();
    bool  collides = normSquared(dir) <= min_dist * min_dist;

    if (collides) {
        dir = normalized(dir);
        return {
            .normal     = dir,
            .contacts_a = {a.center() + a.radius() * dir},
            .contacts_b = {b.center() - b.radius() * dir},
            .n_contacts = 1
        };
    } else {
        return Contacts<1>::none();
    }
}

inline Contacts<1> collides(const Circle& a, const Capsule& b) {
    Vec2  axis     = normalized(b.top() - b.bottom());
    float axis_len = norm(b.top() - b.bottom());

    float circle_pos_along_axis = dot(a.center() - b.bottom(), axis);

    bool is_under = circle_pos_along_axis + a.radius() <= 0.f;
    bool is_over  = circle_pos_along_axis - a.radius() >= axis_len;
    if (is_under || is_over) {
        return Contacts<1>::none();
    }

    Vec2  perp         = simu::perp(axis);
    float perp_dist    = dot(a.center() - b.bottom(), perp);
    bool  is_near_axis = std::abs(perp_dist) <= a.radius() + b.radius();
    if (!is_near_axis) {
        return Contacts<1>::none();
    }

    if (circle_pos_along_axis <= b.radius()) {
        // Collision with bottom circle
        return collides(a, Circle(b.bottom() + axis * b.radius(), b.radius()));
    }

    if (circle_pos_along_axis >= axis_len - b.radius()) {
        // Collision with top circle
        return collides(a, Circle(b.top() - axis * b.radius(), b.radius()));
    }

    // Collision with axis
    if (perp_dist > 0.f) {
        return {
            .normal     = -perp,
            .contacts_a = {a.center() - perp * a.radius()},
            .contacts_b = {b.bottom() + axis * circle_pos_along_axis + perp * b.radius()},
            .n_contacts = 1
        };
    } else {
        return {
            .normal     = perp,
            .contacts_a = {a.center() + perp * a.radius()},
            .contacts_b = {b.bottom() + axis * circle_pos_along_axis - perp * b.radius()},
            .n_contacts = 1
        };
    }
}

inline Contacts<1> collides(const Circle& a, const Polygon& b) {
    Contacts<1> contact;
    float       min_pen = std::numeric_limits<float>::max();

    // Create a normal from the circle to each vertex
    // Can exit early if all vertices are away
    // Can only define a contact with the vertex used to create the normal
    for (std::size_t i = 0; i < b.n_vertices(); ++i) {
        Vec2 vertex = b.vertex(i);
        Vec2 normal = normalized(vertex - a.center());

        // Can create a contact only for this vertex
        float current_dist = dot(normal, b.vertex(i) - a.center()) - a.radius();
        if (current_dist <= 0.f && -current_dist < min_pen) {
            min_pen = -current_dist;
            contact = {
                .normal     = normal,
                .contacts_a = {a.center() + a.radius() * normal},
                .contacts_b = {vertex},
                .n_contacts = 1
            };
        } else if (min_pen == std::numeric_limits<float>::max()) {
            // Check if a separating plane exists if penetration is not already found
            float min_dist = current_dist;
            for (std::size_t j = 0; j < b.n_vertices() && i != j; ++j) {
                float dist = dot(normal, b.vertex(j) - a.center()) - a.radius();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }

            if (min_dist > 0.f) {
                return Contacts<1>::none();
            }
        }
    }

    // Test all face normals (regular SAT)
    for (std::size_t i = 0; i < b.n_vertices(); ++i) {
        Vec2 current_vertex = b.vertex(i);
        Vec2 next_vertex    = b.vertex((i + 1) == b.n_vertices() ? 0 : i + 1);

        Vec2 edge   = normalized(next_vertex - current_vertex);
        Vec2 normal = perp(edge);

        float dist = dot(a.center() - current_vertex, normal);
        if (dist > 0.f) {
            return Contacts<1>::none();
        }

        if (-dist < min_pen) {
            min_pen = -dist;
            contact = {
                .normal     = -normal,
                .contacts_a = {a.center() + a.radius() * contact.normal},
                .contacts_b = {current_vertex + edge * dot(edge, a.center() - current_vertex)},
                .n_contacts = 1
            };
        }
    }

    return contact;
}

inline Contacts<2> collides(const Capsule& a, const Capsule& b) {
    Vec2 a_top_center    = a.top_center();
    Vec2 a_bottom_center = a.bottom_center();
    Vec2 b_top_center    = b.top_center();
    Vec2 b_bottom_center = b.bottom_center();

    // Project each of the capsule's centers onto the other capsule's axis
    Vec2 proj_a_top_center
        = LineBarycentric{b_bottom_center, b_top_center, a_top_center}.closestPoint;
    Vec2 proj_a_bottom_center
        = LineBarycentric{b_bottom_center, b_top_center, a_bottom_center}.closestPoint;
    Vec2 proj_b_top_center
        = LineBarycentric{a_bottom_center, a_top_center, b_top_center}.closestPoint;
    Vec2 proj_b_bottom_center
        = LineBarycentric{a_bottom_center, a_top_center, b_bottom_center}.closestPoint;

    float dist_a_top_center = normSquared(proj_a_top_center - a_top_center);
    float dist_a_bottom_center = normSquared(proj_a_bottom_center - a_bottom_center);
    float dist_b_top_center = normSquared(proj_b_top_center - b_top_center);
    float dist_b_bottom_center = normSquared(proj_b_bottom_center - b_bottom_center);

    float min_dist = (a.radius() + b.radius()) * (a.radius() + b.radius());

    bool has_contact_a_top_center    = dist_a_top_center < min_dist;
    bool has_contact_a_bottom_center = dist_a_bottom_center < min_dist;
    bool has_contact_b_top_center    = dist_b_top_center < min_dist;
    bool has_contact_b_bottom_center = dist_b_bottom_center < min_dist;

    if (has_contact_a_top_center) {
        Vec2 normal = normalized(proj_a_top_center - a_top_center);

        Contacts<2> result = {
            .normal     = normal,
            .contacts_a = {a_top_center + normal * a.radius()},
            .contacts_b = {proj_a_top_center - normal * b.radius()},
            .n_contacts = 1,
        };

        if (has_contact_a_bottom_center) {
            result.contacts_a[1] = a_bottom_center + normal * a.radius();
            result.contacts_b[1] = proj_a_bottom_center - normal * b.radius();
            result.n_contacts    = 2;
        } else if (has_contact_b_top_center) {
            result.contacts_a[1] = proj_b_top_center + normal * a.radius();
            result.contacts_b[1] = b_top_center - normal * b.radius();
            result.n_contacts    = 2;
        } else if (has_contact_b_bottom_center) {
            result.contacts_a[1] = proj_b_bottom_center + normal * a.radius();
            result.contacts_b[1] = b_bottom_center - normal * b.radius();
            result.n_contacts    = 2;
        }

        return result;
    }

    if (has_contact_a_bottom_center) {
        Vec2 normal = normalized(proj_a_bottom_center - a_bottom_center);

        Contacts<2> result = {
            .normal     = normal,
            .contacts_a = {a_bottom_center + normal * a.radius()},
            .contacts_b = {proj_a_bottom_center - normal * b.radius()},
            .n_contacts = 1,
        };

        if (has_contact_b_top_center) {
            result.contacts_a[1] = proj_b_top_center + normal * a.radius();
            result.contacts_b[1] = b_top_center - normal * b.radius();
            result.n_contacts    = 2;
        } else if (has_contact_b_bottom_center) {
            result.contacts_a[1] = proj_b_bottom_center + normal * a.radius();
            result.contacts_b[1] = b_bottom_center - normal * b.radius();
            result.n_contacts    = 2;
        }

        return result;
    }

    if (has_contact_b_top_center) {
        Vec2 normal = -normalized(proj_b_top_center - b_top_center);

        Contacts<2> result = {
            .normal     = normal,
            .contacts_a = {proj_b_top_center + normal * a.radius()},
            .contacts_b = {b_top_center - normal * b.radius()},
            .n_contacts = 1
        };

        if (has_contact_b_bottom_center) {
            result.contacts_a[1] = proj_b_bottom_center + normal * a.radius();
            result.contacts_b[1] = b_bottom_center - normal * b.radius();
            result.n_contacts    = 2;
        }

        return result;
    }

    if (has_contact_b_bottom_center) {
        Vec2 normal = -normalized(proj_b_bottom_center - b_bottom_center);

        return Contacts<2>{
            .normal     = normal,
            .contacts_a = {proj_b_bottom_center + normal * a.radius()},
            .contacts_b = {b_bottom_center - normal * b.radius()},
            .n_contacts = 1
        };
    }

    return Contacts<2>::none();
}

inline Contacts<2> collides(const Capsule& a, const Polygon& b, float epsilon) {
    Contacts<2> contact        = Contacts<2>::none();
    float       min_pen        = std::numeric_limits<float>::max();
    Vec2        top_center     = a.top_center();
    Vec2        bottom_center  = a.bottom_center();

    // Project each vertex onto the capsule
    for (std::size_t i = 0; i < b.n_vertices(); ++i) {
        Vec2 vertex = b.vertex(i);
        Vec2 projection = LineBarycentric{bottom_center, top_center, vertex}.closestPoint;
        Vec2 normal = vertex - projection;

        float current_dist = norm(normal) - a.radius();
        if (current_dist <= 0.f && -current_dist < min_pen) {
            normal         = normalized(normal);
            min_pen        = -current_dist;
            contact        = {
                       .normal     = normal,
                       .contacts_a = {projection + a.radius() * normal},
                       .contacts_b = {vertex},
                       .n_contacts = 1
            };
        } else if (min_pen == std::numeric_limits<float>::max()) {
            // Check if a separating plane exists if penetration is not already found
            normal = normalized(normal);
            float min_dist = current_dist;
            for (std::size_t j = 0; j < b.n_vertices() && i != j; ++j) {
                float dist = dot(normal, b.vertex(j) - projection) - a.radius();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }

            if (min_dist > 0.f) {
                return Contacts<2>::none();
            }
        }
    }

    // Regular SAT
    for (std::size_t i = 0 ; i < b.n_vertices(); i++) {
        Vec2 current_vertex = b.vertex(i);
        Vec2 next_vertex    = b.vertex((i + 1) == b.n_vertices() ? 0 : i + 1);

        Vec2 edge   = next_vertex - current_vertex;
        Vec2 normal = perp(edge);

        // Project centers on the edge
        LineBarycentric proj_top_center_along_edge = LineBarycentric{
            current_vertex, next_vertex, top_center
        };
        LineBarycentric proj_bottom_center_along_edge = LineBarycentric{
            current_vertex, next_vertex, bottom_center
        };

        // Take closest point to projection along axis when edge is short
        Vec2 reverse_proj_top_center
            = proj_top_center_along_edge.is_projection_inside_segment()
                  ? top_center
                  : LineBarycentric(
                        bottom_center, top_center, proj_top_center_along_edge.closestPoint
                    )
                        .closestPoint;

        Vec2 reverse_proj_bottom_center
            = proj_bottom_center_along_edge.is_projection_inside_segment()
                  ? bottom_center
                  : LineBarycentric(
                        bottom_center, top_center, proj_bottom_center_along_edge.closestPoint
                    )
                        .closestPoint;


        float dist_proj_top_center = norm(reverse_proj_top_center - top_center);
        float dist_proj_bottom_center = norm(
            reverse_proj_bottom_center - bottom_center
        );

        bool has_contact_top_center    = dist_proj_top_center < a.radius();
        bool has_contact_bottom_center = dist_proj_bottom_center < a.radius();

        float current_min_pen = std::min(
            dist_proj_top_center - a.radius(), dist_proj_bottom_center - a.radius()
        );
        bool has_better_min_pen_vector = -current_min_pen < min_pen;

        if ((!has_contact_top_center && !has_contact_bottom_center)
            || !has_better_min_pen_vector) {
            continue;
        }

        bool produces_same_contacts
            = all(approx(reverse_proj_top_center, Vec2::filled(epsilon))
                      .contains(reverse_proj_bottom_center))
              || all(approx(proj_top_center_along_edge.closestPoint, Vec2::filled(epsilon))
                         .contains(proj_bottom_center_along_edge.closestPoint));

        if (has_contact_top_center && has_contact_bottom_center
            && !produces_same_contacts) {
            normal             = -normal;
            contact.normal     = normal;
            contact.contacts_a = {
                reverse_proj_bottom_center + normal * a.radius(),
                reverse_proj_top_center + normal * a.radius()
            };
            contact.contacts_b = {
                proj_bottom_center_along_edge.closestPoint,
                proj_top_center_along_edge.closestPoint
            };
            contact.n_contacts = 2;
        } else if (has_contact_top_center) {
            normal             = -normal;
            contact.normal     = normal;
            contact.contacts_a = {
                reverse_proj_top_center + normal * a.radius(),
            };
            contact.contacts_b = {proj_top_center_along_edge.closestPoint};
            contact.n_contacts = 1;
        } else if (has_contact_bottom_center) {
            normal             = -normal;
            contact.normal     = normal;
            contact.contacts_a = {
                reverse_proj_bottom_center + normal * a.radius(),
            };
            contact.contacts_b = {proj_bottom_center_along_edge.closestPoint};
            contact.n_contacts = 1;
        }
    }

    return contact;
}

inline bool collides(const Polygon& a, const Polygon& b) {}

} // namespace simu
