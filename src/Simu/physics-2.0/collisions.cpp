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


#include "Simu/physics-2.0/collision/collisions.hpp"


namespace
{

using namespace simu;

inline float closest_point(const Polygon& polygon, const Vec2& point, Vec2 normal) {
    float min_dist = std::numeric_limits<float>::max();

    for (std::size_t j = 0; j < polygon.n_vertices(); ++j) {
        min_dist = std::min(min_dist, dot(normal, polygon.vertex(j) - point));
    }

    return min_dist;
}

inline std::size_t most_opposite_face(const Polygon& polygon, Vec2 normal) {
    float       min_dot_prod = std::numeric_limits<float>::max();
    std::size_t edge_index   = std::numeric_limits<std::size_t>::max();

    for (std::size_t j = 0; j < polygon.n_vertices(); ++j) {
        Vec2 edge_normal = perp(
            normalized(polygon.vertex((j + 1) % polygon.n_vertices()) - polygon.vertex(j)), true
        );

        float dot_prod = dot(normal, edge_normal);
        if (dot_prod < min_dot_prod) {
            min_dot_prod = dot_prod;
            edge_index   = j;
        }
    }

    return edge_index;
}

/// Clips the edge along the given normal starting from the reference point
inline std::optional<Vec2>
clip_edge(Vec2 reference_point, Vec2 normal, Vec2 edge_start, Vec2 edge_end) {
    Vec2 edge_direction = edge_end - edge_start;
    if (dot(edge_direction, normal) == 0.f)
        return std::nullopt;

    Vec2 parametricCoefficients = solve(
        Mat2::fromCols({normal, -edge_direction}), edge_start - reference_point
    );

    float u = parametricCoefficients[1]; // along edge
    if (u > 1.f) {
        return edge_end;
    } else if (u < 0.f) {
        return edge_start;
    }

    return edge_start + u * edge_direction;
}

inline Contacts<2> create_contacts(
    Vec2 ref_edge_start,
    Vec2 ref_edge_end,
    Vec2 opposite_edge_start,
    Vec2 opposite_edge_end,
    Vec2 normal,
    float epsilon
) {
    // Due to positive vertex ordering, the start of the ref edge is matched to
    // the end of the opposite edge

    // Clip opposite edge
    Vec2 opposite_contact = clip_edge(ref_edge_end, normal, opposite_edge_start, opposite_edge_end)
                                .value_or(opposite_edge_start);

    Vec2 next_opposite_contact = clip_edge(ref_edge_start, normal, opposite_edge_start, opposite_edge_end)
                                     .value_or(opposite_edge_end);

    // Project back onto the reference edge
    Vec2 ref_contact
        = LineBarycentric(ref_edge_start, ref_edge_end, next_opposite_contact).closestPoint;

    Vec2 next_ref_contact
        = LineBarycentric(ref_edge_start, ref_edge_end, opposite_contact).closestPoint;

    bool first_has_contact = dot(normal, next_opposite_contact - ref_contact) <= 0.f;
    bool second_has_contact = dot(normal, opposite_contact - next_ref_contact) <= 0.f;
    bool duplicated_contact = all(approx(ref_contact, Vec2::filled(epsilon)).contains(next_ref_contact))
        || all(approx(opposite_contact, Vec2::filled(epsilon)).contains(next_opposite_contact));

    Contacts<2> contacts;
    contacts.normal     = normal;
    contacts.n_contacts = 0;

    if (first_has_contact) {
        contacts.contacts_a[0] = ref_contact;
        contacts.contacts_b[0] = next_opposite_contact;
        contacts.n_contacts    = contacts.n_contacts + 1;
    }
    if (second_has_contact && !duplicated_contact) {
        contacts.contacts_a[contacts.n_contacts] = next_ref_contact;
        contacts.contacts_b[contacts.n_contacts] = opposite_contact;
        contacts.n_contacts                      = contacts.n_contacts + 1;
    }

    return contacts;
}

} // namespace

namespace simu
{

Contacts<1> collides(const Circle& a, const Circle& b) {
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

Contacts<1> collides(const Circle& a, const Capsule& b) {
    Vec2  axis     = normalized(b.top() - b.bottom());
    float axis_len = norm(b.top() - b.bottom());

    float circle_pos_along_axis = dot(a.center() - b.bottom(), axis);

    bool is_under = circle_pos_along_axis + a.radius() < 0.f;
    bool is_over  = circle_pos_along_axis - a.radius() > axis_len;
    if (is_under || is_over) {
        return Contacts<1>::none();
    }

    Vec2  perp         = simu::perp(axis, true);
    float perp_dist    = dot(a.center() - b.bottom(), perp);
    bool  is_near_axis = std::abs(perp_dist) <= a.radius() + b.radius();
    if (!is_near_axis) {
        return Contacts<1>::none();
    }

    Vec2 top_center = b.top_center();
    Vec2 bottom_center = b.bottom_center();
    float squared_dist_to_top = normSquared(a.center() - top_center);
    float squared_dist_to_bottom = normSquared(a.center() - bottom_center);

    Vec2 closest_point_on_axis = LineBarycentric{bottom_center, top_center, a.center()}.closestPoint;
    float squared_dist_to_axis = normSquared(closest_point_on_axis - a.center());

    if (squared_dist_to_top <= squared_dist_to_bottom && squared_dist_to_top <= squared_dist_to_axis) {
        // Collision with top circle
        return collides(a, Circle(top_center, b.radius()));
    }

    if (squared_dist_to_bottom <= squared_dist_to_top && squared_dist_to_bottom <= squared_dist_to_axis) {
        // Collision with bottom circle
        return collides(a, Circle(bottom_center, b.radius()));
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

Contacts<1> collides(const Circle& a, const Polygon& b) {
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
            for (std::size_t j = 0; j < b.n_vertices(); ++j) {
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
        Vec2 normal = perp(edge, true);

        float dist = dot(a.center() - current_vertex, normal) - a.radius();
        if (dist > 0.f) {
            return Contacts<1>::none();
        }

        if (-dist < min_pen) {
            normal = -normal;
            min_pen = -dist;
            contact = {
                .normal     = normal,
                .contacts_a = {a.center() + a.radius() * normal},
                .contacts_b = {current_vertex + edge * dot(edge, a.center() - current_vertex)},
                .n_contacts = 1
            };
        }
    }

    return contact;
}

Contacts<2> collides(const Capsule& a, const Capsule& b, float epsilon) {
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

    bool has_contact_a_top_center    = dist_a_top_center <= min_dist;
    bool has_contact_a_bottom_center = dist_a_bottom_center <= min_dist;
    bool has_contact_b_top_center    = dist_b_top_center <= min_dist;
    bool has_contact_b_bottom_center = dist_b_bottom_center <= min_dist;

    Contacts<2> result = Contacts<2>::none();
    if (has_contact_a_top_center) {
        Vec2 normal = normalized(proj_a_top_center - a_top_center);

        result = {
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
    } else if (has_contact_a_bottom_center) {
        Vec2 normal = normalized(proj_a_bottom_center - a_bottom_center);

        result = {
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
    } else if (has_contact_b_top_center) {
        Vec2 normal = -normalized(proj_b_top_center - b_top_center);

        result = {
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
    } else if (has_contact_b_bottom_center) {
        Vec2 normal = -normalized(proj_b_bottom_center - b_bottom_center);

        result = {
            .normal     = normal,
            .contacts_a = {proj_b_bottom_center + normal * a.radius()},
            .contacts_b = {b_bottom_center - normal * b.radius()},
            .n_contacts = 1
        };
    }

    if (result.n_contacts == 2) {
        if (all(approx(result.contacts_a[0], Vec2::filled(epsilon))
                    .contains(result.contacts_a[1]))
            || all(approx(result.contacts_b[0], Vec2::filled(epsilon))
                       .contains(result.contacts_b[1]))) {
            result.n_contacts = 1;
        }
    }

    return result;
}

Contacts<2> collides(const Capsule& a, const Polygon& b, float epsilon) {
    Contacts<2> contact       = Contacts<2>::none();
    float       min_pen       = std::numeric_limits<float>::max();
    Vec2        top_center    = a.top_center();
    Vec2        bottom_center = a.bottom_center();

    // Project each vertex onto the capsule
    for (std::size_t i = 0; i < b.n_vertices(); ++i) {
        Vec2 vertex = b.vertex(i);
        Vec2 projection = LineBarycentric{bottom_center, top_center, vertex}.closestPoint;
        Vec2 normal = vertex - projection;

        float current_dist = norm(normal) - a.radius();
        if (current_dist <= 0.f && -current_dist < min_pen) {
            normal  = normalized(normal);
            min_pen = -current_dist;
            contact = {
                .normal     = normal,
                .contacts_a = {projection + a.radius() * normal},
                .contacts_b = {vertex},
                .n_contacts = 1
            };
        } else if (min_pen == std::numeric_limits<float>::max()) {
            // Check if a separating plane exists if penetration is not already found
            normal         = normalized(normal);
            float min_dist = current_dist;
            for (std::size_t j = 0; j < b.n_vertices(); ++j) {
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
    for (std::size_t i = 0; i < b.n_vertices(); i++) {
        Vec2 current_vertex = b.vertex(i);
        Vec2 next_vertex    = b.vertex((i + 1) == b.n_vertices() ? 0 : i + 1);

        Vec2 edge   = next_vertex - current_vertex;
        Vec2 normal = perp(edge, true);

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
        float dist_proj_bottom_center = norm(reverse_proj_bottom_center - bottom_center);

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
            = all(approx(reverse_proj_top_center, Vec2::filled(epsilon)).contains(reverse_proj_bottom_center)
              )
              || all(approx(proj_top_center_along_edge.closestPoint, Vec2::filled(epsilon))
                         .contains(proj_bottom_center_along_edge.closestPoint));

        normal = normalized(normal);
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

Contacts<2> collides(const Polygon& a, const Polygon& b, float epsilon) {
    std::size_t current_contact_edge = std::numeric_limits<std::size_t>::max();
    float       min_pen              = std::numeric_limits<float>::max();
    bool        contact_edge_is_on_a = true;

    for (std::size_t i = 0; i < a.n_vertices(); ++i) {
        Vec2 current_vertex = a.vertex(i);
        Vec2 next_vertex    = a.vertex((i + 1) == a.n_vertices() ? 0 : i + 1);
        Vec2 normal         = perp(next_vertex - current_vertex, true);

        float dist = closest_point(b, current_vertex, normal);
        if (dist > 0.f) {
            return Contacts<2>::none();
        } else if (-dist < min_pen) {
            min_pen              = -dist;
            current_contact_edge = i;
        }
    }

    for (std::size_t i = 0; i < b.n_vertices(); ++i) {
        Vec2 current_vertex = b.vertex(i);
        Vec2 next_vertex    = b.vertex((i + 1) == b.n_vertices() ? 0 : i + 1);
        Vec2 normal         = perp(next_vertex - current_vertex, true);

        float dist = closest_point(a, current_vertex, normal);
        if (dist > 0.f) {
            return Contacts<2>::none();
        } else if (-dist < min_pen) {
            min_pen              = -dist;
            current_contact_edge = i;
            contact_edge_is_on_a = false;
        }
    }

    if (contact_edge_is_on_a) {
        std::size_t next_index = current_contact_edge + 1 == a.n_vertices()
                                     ? 0
                                     : current_contact_edge + 1;

        Vec2 edge   = a.vertex(next_index) - a.vertex(current_contact_edge);
        Vec2 normal = perp(normalized(edge), true);

        std::size_t opposite_edge_index  = most_opposite_face(b, normal);
        Vec2        opposite_vertex      = b.vertex(opposite_edge_index);
        Vec2        next_opposite_vertex = b.vertex(
            opposite_edge_index + 1 == b.n_vertices() ? 0 : opposite_edge_index + 1
        );

        return create_contacts(
            a.vertex(current_contact_edge), a.vertex(next_index), opposite_vertex, next_opposite_vertex, normal, epsilon
        );
    } else {
        std::size_t next_index = current_contact_edge + 1 == b.n_vertices()
                                     ? 0
                                     : current_contact_edge + 1;
        Vec2 edge   = b.vertex(next_index) - b.vertex(current_contact_edge);
        Vec2 normal = perp(normalized(edge), true);

        std::size_t opposite_edge_index  = most_opposite_face(a, normal);
        Vec2        opposite_vertex      = a.vertex(opposite_edge_index);
        Vec2        next_opposite_vertex = a.vertex(
            opposite_edge_index + 1 == a.n_vertices() ? 0 : opposite_edge_index + 1
        );

        Contacts<2> contacts = create_contacts(
            b.vertex(current_contact_edge), b.vertex(next_index), opposite_vertex, next_opposite_vertex, normal, epsilon
        );
        contacts.normal = -contacts.normal;
        std::swap(contacts.contacts_a, contacts.contacts_b);
        return contacts;
    }
}

} // namespace simu