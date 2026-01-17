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


#include "Simu/physics-2.0/collision/colliders/collisions.hpp"


namespace
{

using namespace simu;

inline float closest_point(const Polygon& polygon, const Vec2& point, Vec2 normal) {
    float min_dist = std::numeric_limits<float>::max();

    for (std::size_t j = 0; j < polygon.size(); ++j) {
        min_dist = std::min(min_dist, dot(normal, polygon.vertex(j) - point));
    }

    return min_dist;
}

inline std::size_t most_opposite_face(const Polygon& polygon, Vec2 normal) {
    float       min_dot_prod = std::numeric_limits<float>::max();
    std::size_t edge_index   = std::numeric_limits<std::size_t>::max();

    for (std::size_t j = 0; j < polygon.size(); ++j) {
        Vec2 edge_normal = perp(
            normalized(polygon.vertex((j + 1) % polygon.size()) - polygon.vertex(j)), true
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
    if (cross(edge_direction, normal) == 0.f)
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
    Vec2  ref_edge_start,
    Vec2  ref_edge_end,
    Vec2  opposite_edge_start,
    Vec2  opposite_edge_end,
    Vec2  normal,
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
    bool duplicated_contact
        = all(approx(ref_contact, Vec2::filled(epsilon)).contains(next_ref_contact))
          || all(approx(opposite_contact, Vec2::filled(epsilon)).contains(next_opposite_contact)
          );

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

Contacts<1> collide(const Circle& a, const Circle& b) {
    float min_dist     = a.radius() + b.radius();
    Vec2  dir          = b.center() - a.center();
    float dist_squared = normSquared(dir);

    // degenerate case, take no action
    if (dist_squared == 0.f) {
        return Contacts<1>::none();
    }

    if (dist_squared <= min_dist * min_dist) {
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

Contacts<1> collide(const Circle& a, const Capsule& b) {
    Vec2 closest_point_on_axis
        = LineBarycentric{b.bottom_center(), b.top_center(), a.center()}.closestPoint;

    Vec2  dir          = closest_point_on_axis - a.center();
    float min_dist     = a.radius() + b.radius();
    float dist_squared = normSquared(dir);
    if (dist_squared > min_dist * min_dist) {
        return Contacts<1>::none();
    }

    // Degenerate case, circle along the segment. Take no action
    if (dist_squared == 0.f) {
        return Contacts<1>::none();
    }

    Vec2 normal = normalized(dir);
    return {
        .normal     = normal,
        .contacts_a = {a.center() + a.radius() * normal},
        .contacts_b = {closest_point_on_axis - normal * b.radius()},
        .n_contacts = 1
    };
}

Contacts<1> collide(const Circle& a, const Polygon& b) {
    Contacts<1> contact;
    float       min_pen = std::numeric_limits<float>::max();

    // Create a normal from the circle to each vertex
    // Can exit early if all vertices are away
    // Can only define a contact with the vertex used to create the normal
    for (std::size_t i = 0; i < b.size(); ++i) {
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
            for (std::size_t j = 0; j < b.size(); ++j) {
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
    for (std::size_t i = 0; i < b.size(); ++i) {
        Vec2 current_vertex = b.vertex(i);
        Vec2 next_vertex    = b.vertex((i + 1) == b.size() ? 0 : i + 1);

        Vec2 edge   = normalized(next_vertex - current_vertex);
        Vec2 normal = perp(edge, true);

        float dist = dot(a.center() - current_vertex, normal) - a.radius();
        if (dist > 0.f) {
            return Contacts<1>::none();
        }

        if (-dist < min_pen) {
            normal  = -normal;
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

Contacts<2> collide(const Capsule& a, const Capsule& b, float epsilon) {
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

// Valid only if the segment of the capsule a is completely outside of b
// Use SAT on every edge of polygon and also the projection of each point on the capsule (n = proj - v)
Contacts<2> collide_shallow(const Capsule& a, const Polygon& b, float epsilon) {
    float min_penetration = std::numeric_limits<float>::lowest();
    bool  is_edge_contact = false;
    int   edge_index      = -1;

    for (int i = 0; i < b.size(); ++i) {
        Vec2  v0               = b.vertex(i);
        Vec2  v1               = b.vertex((i + 1) % b.size());
        Vec2  edge             = v1 - v0;
        Vec2  normal           = normalized(perp(edge, true));
        float edge_penetration = std::min(
                                     dot(normal, a.bottom_center() - v0),
                                     dot(normal, a.top_center() - v0)
                                 )
                                 - a.radius();

        if (edge_penetration >= 0.f) {
            return Contacts<2>::none();
        }

        if (edge_penetration > min_penetration) {
            min_penetration = edge_penetration;
            is_edge_contact = true;
            edge_index      = i;
        }

        // TODO: Useless?
        auto projection = LineBarycentric(a.bottom_center(), a.top_center(), v0);
        Vec2 vertex_normal = normalized(projection.closestPoint - v0);
        float vertex_penetration = closest_point(b, projection.closestPoint, vertex_normal) - a.radius();
        if (vertex_penetration < 0.f && vertex_penetration > min_penetration) {
            min_penetration = vertex_penetration;
            is_edge_contact = false;
            edge_index      = i;
        }
    }

    if (edge_index == -1) {
        return Contacts<2>::none();
    }

    if (is_edge_contact) {
        Vec2 current_vertex = b.vertex(edge_index);
        Vec2 next_vertex = b.vertex((edge_index + 1) == b.size() ? 0 : edge_index + 1);

        Vec2 edge   = next_vertex - current_vertex;
        Vec2 normal = -normalized(perp(edge, true));

        // Project centers on the edge
        LineBarycentric proj_top_center_along_edge = LineBarycentric{
            current_vertex, next_vertex, a.top_center()
        };
        LineBarycentric proj_bottom_center_along_edge = LineBarycentric{
            current_vertex, next_vertex, a.bottom_center()
        };

        // Take the closest point to projection along axis when edge is short
        Vec2 reverse_proj_top_center = proj_top_center_along_edge.is_projection_inside_segment()
                                           ? a.top_center()
                                           : LineBarycentric(
                                                 a.bottom_center(),
                                                 a.top_center(),
                                                 proj_top_center_along_edge.closestPoint
                                             )
                                                 .closestPoint;

        Vec2 reverse_proj_bottom_center
            = proj_bottom_center_along_edge.is_projection_inside_segment()
                  ? a.bottom_center()
                  : LineBarycentric(
                        a.bottom_center(),
                        a.top_center(),
                        proj_bottom_center_along_edge.closestPoint
                    )
                        .closestPoint;


        float dist_proj_top_center = dot(normal,
                                         proj_top_center_along_edge.closestPoint
                                             - reverse_proj_top_center)
                                     - a.radius();

        float dist_proj_bottom_center = dot(normal,
                                            proj_bottom_center_along_edge.closestPoint
                                                - reverse_proj_bottom_center)
                                        - a.radius();

        bool has_contact_top_center    = dist_proj_top_center <= 0.f;
        bool has_contact_bottom_center = dist_proj_bottom_center <= 0.f;

        bool produces_same_contacts = normSquared(reverse_proj_top_center - reverse_proj_bottom_center)
                                          <= epsilon * epsilon
                                      || normSquared(
                                             proj_top_center_along_edge.closestPoint
                                             - proj_bottom_center_along_edge.closestPoint
                                         ) <= epsilon * epsilon;


        // TODO: If single contact and projection inside segment, adjust normal instead of using edge normal, should be axis normal.
        Contacts<2> contacts;
        if (has_contact_top_center && has_contact_bottom_center
            && !produces_same_contacts) {
            contacts.normal     = normal;
            contacts.contacts_a = {
                reverse_proj_bottom_center + normal * a.radius(),
                reverse_proj_top_center + normal * a.radius()
            };
            contacts.contacts_b = {
                proj_bottom_center_along_edge.closestPoint,
                proj_top_center_along_edge.closestPoint
            };
            contacts.n_contacts = 2;
        } else if (has_contact_top_center) {
            contacts.normal     = normal;
            contacts.contacts_a = {
                reverse_proj_top_center + normal * a.radius(),
            };
            contacts.contacts_b = {proj_top_center_along_edge.closestPoint};
            contacts.n_contacts = 1;
        } else if (has_contact_bottom_center) {
            contacts.normal     = normal;
            contacts.contacts_a = {
                reverse_proj_bottom_center + normal * a.radius(),
            };
            contacts.contacts_b = {proj_bottom_center_along_edge.closestPoint};
            contacts.n_contacts = 1;
        } else {
            // TODO: should check along capsule's sides? would prevent this code path
            return Contacts<2>::none();
        }

        return contacts;
    } else {
        Vec2 proj = LineBarycentric{a.bottom_center(), a.top_center(), b.vertex(edge_index)}
                        .closestPoint;
        Vec2 normal = normalized(b.vertex(edge_index) - proj);
        return {
            .normal     = normal,
            .contacts_a = {proj + normal * a.radius()},
            .contacts_b = {b.vertex(edge_index)},
            .n_contacts = 1
        };
    }
}

// TODO: Don't do explicit SAT, keep best two contacts during projection.
//  If two contacts, use normal from edge connecting them
// TODO: Algo is just generally incorrect...
Contacts<2> collide(const Capsule& a, const Polygon& b, float epsilon) {
    return collide_shallow(a, b, epsilon);

    // float min_pen = std::numeric_limits<float>::max();
    // Vec2  contact_normal;
    //
    // // polygon edge normals
    // for (std::size_t i = 0; i < b.size(); ++i) {
    //     Vec2 v0     = b.vertex(i);
    //     Vec2 v1     = b.vertex((i + 1) % b.size());
    //     Vec2 normal = normalized(perp(v1 - v0, true));
    //
    //     float capsule_dist = std::min(
    //                              dot(normal, a.bottom_center() - v0),
    //                              dot(normal, a.top_center() - v1)
    //                          )
    //                          - a.radius();
    //
    //     if (capsule_dist > 0.f) {
    //         return Contacts<2>::none();
    //     }
    //
    //     if (-capsule_dist < min_pen) {
    //         min_pen        = -capsule_dist;
    //         contact_normal = -normal;
    //     }
    // }
    //
    // // capsule segment normals
    // Vec2 axis = a.top_center() - a.bottom_center();
    // if (normSquared(axis) > EPSILON * EPSILON) {
    //     axis = normalized(axis);
    //
    //     for (Vec2 normal : {perp(axis), -perp(axis)}) {
    //         float dist = closest_point(b, a.bottom_center(), normal);
    //         dist -= a.radius();
    //
    //         if (dist > 0.f)
    //             return Contacts<2>::none();
    //
    //         if (-dist < min_pen) {
    //             min_pen        = -dist;
    //             contact_normal = normal;
    //         }
    //     }
    // }
    //
    // std::size_t edge_index = most_opposite_face(b, contact_normal);
    // std::size_t next_index = (edge_index + 1) % b.size();
    // Vec2        v0         = b.vertex(edge_index);
    // Vec2        v1         = b.vertex(next_index);
    // Vec2        edge       = v1 - v0;
    //
    // bool parallel = cross(edge, axis)
    //
    //     auto bottom_on_edge
    //     = LineBarycentric(v0, v1, a.bottom_center());
    // float bottom_dist = dot(contact_normal, v0 - a.bottom_center()) - a.radius();
    //
    // auto  top_on_edge = LineBarycentric(v0, v1, a.top_center());
    // float top_dist    = dot(contact_normal, v0 - a.top_center()) - a.radius();
    //
    // bool top_is_valid = top_on_edge.is_projection_inside_segment() && top_dist < 0.f;
    // bool bottom_is_valid = bottom_on_edge.is_projection_inside_segment()
    //                        && bottom_dist < 0.f;
    //
    // if (!top_is_valid && !bottom_is_valid) {
    //     // may need to use end circles for normal to vertex
    //
    //     auto v0_on_segment = LineBarycentric(a.bottom_center(), a.top_center(), v0);
    //     float v0_dist = normSquared(v0_on_segment.closestPoint - v0);
    //
    //     auto v1_on_segment = LineBarycentric(a.bottom_center(), a.top_center(), v1);
    //     float v1_dist = normSquared(v1_on_segment.closestPoint - v1);
    //
    //     if (v0_dist < v1_dist) {
    //         Vec2 normal = v0 - v0_on_segment.closestPoint;
    //
    //
    //         return {}
    //     }
    // }

    // Contacts<2> contact;
    // float       min_pen       = std::numeric_limits<float>::max();
    // Vec2        top_center    = a.top_center();
    // Vec2        bottom_center = a.bottom_center();

    // Project each vertex onto the capsule
    // for (std::size_t i = 0; i < b.size(); ++i) {
    //     Vec2 vertex = b.vertex(i);
    //     Vec2 projection = LineBarycentric{bottom_center, top_center,
    //     vertex}.closestPoint; Vec2 normal = vertex - projection;
    //
    //     float current_dist = norm(normal) - a.radius();
    //     if (current_dist <= 0.f && -current_dist < min_pen) {
    //         normal  = normalized(normal);
    //         min_pen = -current_dist;
    //         contact = {
    //             .normal     = normal,
    //             .contacts_a = {projection + a.radius() * normal},
    //             .contacts_b = {vertex},
    //             .n_contacts = 1
    //         };
    //     } else if (min_pen == std::numeric_limits<float>::max()) {
    //         // Check if a separating plane exists if penetration is not
    //         already found normal         = normalized(normal); float min_dist
    //         = current_dist; for (std::size_t j = 0; j < b.size(); ++j) {
    //             float dist = dot(normal, b.vertex(j) - projection) -
    //             a.radius(); if (dist < min_dist) {
    //                 min_dist = dist;
    //             }
    //         }
    //
    //         if (min_dist > 0.f) {
    //             return Contacts<2>::none();
    //         }
    //     }
    // }

    // Regular SAT
    // for (std::size_t i = 0; i < b.size(); i++) {
    //     Vec2 current_vertex = b.vertex(i);
    //     Vec2 next_vertex    = b.vertex((i + 1) == b.size() ? 0 : i + 1);
    //
    //     Vec2 edge   = next_vertex - current_vertex;
    //     Vec2 normal = normalized(perp(edge, true));
    //
    //     // Project centers on the edge
    //     LineBarycentric proj_top_center_along_edge = LineBarycentric{
    //         current_vertex, next_vertex, top_center
    //     };
    //     LineBarycentric proj_bottom_center_along_edge = LineBarycentric{
    //         current_vertex, next_vertex, bottom_center
    //     };
    //
    //     // Take closest point to projection along axis when edge is short
    //     Vec2 reverse_proj_top_center
    //         = proj_top_center_along_edge.is_projection_inside_segment()
    //               ? top_center
    //               : LineBarycentric(
    //                     bottom_center, top_center, proj_top_center_along_edge.closestPoint
    //                 )
    //                     .closestPoint;
    //
    //     Vec2 reverse_proj_bottom_center
    //         = proj_bottom_center_along_edge.is_projection_inside_segment()
    //               ? bottom_center
    //               : LineBarycentric(
    //                     bottom_center, top_center, proj_bottom_center_along_edge.closestPoint
    //                 )
    //                     .closestPoint;
    //
    //
    //     float dist_proj_top_center = dot(normal,
    //                                      proj_top_center_along_edge.closestPoint
    //                                          - reverse_proj_top_center)
    //                                  - a.radius();
    //
    //     float dist_proj_bottom_center = dot(normal,
    //                                         proj_bottom_center_along_edge.closestPoint
    //                                             - reverse_proj_bottom_center)
    //                                     - a.radius();
    //
    //     bool has_contact_top_center    = dist_proj_top_center <= 0.f;
    //     bool has_contact_bottom_center = dist_proj_bottom_center <= 0.f;
    //
    //     float current_min_pen = std::min(dist_proj_top_center, dist_proj_bottom_center);
    //
    //     bool has_better_min_pen_vector = -current_min_pen < min_pen;
    //
    //     if ((!has_contact_top_center && !has_contact_bottom_center)
    //         || !has_better_min_pen_vector) {
    //         continue;
    //     }
    //
    //     bool produces_same_contacts = normSquared(reverse_proj_top_center - reverse_proj_bottom_center)
    //                                       <= epsilon * epsilon
    //                                   || normSquared(
    //                                          proj_top_center_along_edge.closestPoint
    //                                          - proj_bottom_center_along_edge.closestPoint
    //                                      ) <= epsilon * epsilon;
    //
    //     normal = -normal;
    //     if (has_contact_top_center && has_contact_bottom_center
    //         && !produces_same_contacts) {
    //         contact.normal     = normal;
    //         contact.contacts_a = {
    //             reverse_proj_bottom_center + normal * a.radius(),
    //             reverse_proj_top_center + normal * a.radius()
    //         };
    //         contact.contacts_b = {
    //             proj_bottom_center_along_edge.closestPoint,
    //             proj_top_center_along_edge.closestPoint
    //         };
    //         contact.n_contacts = 2;
    //     } else if (has_contact_top_center) {
    //         contact.normal     = normal;
    //         contact.contacts_a = {
    //             reverse_proj_top_center + normal * a.radius(),
    //         };
    //         contact.contacts_b = {proj_top_center_along_edge.closestPoint};
    //         contact.n_contacts = 1;
    //     } else if (has_contact_bottom_center) {
    //         contact.normal     = normal;
    //         contact.contacts_a = {
    //             reverse_proj_bottom_center + normal * a.radius(),
    //         };
    //         contact.contacts_b = {proj_bottom_center_along_edge.closestPoint};
    //         contact.n_contacts = 1;
    //     }
    // }
    //
    // return contact;
}

Contacts<2> collide(const Polygon& a, const Polygon& b, float epsilon) {
    std::size_t current_contact_edge = std::numeric_limits<std::size_t>::max();
    float       min_pen              = std::numeric_limits<float>::max();
    bool        contact_edge_is_on_a = true;

    for (std::size_t i = 0; i < a.size(); ++i) {
        Vec2 current_vertex = a.vertex(i);
        Vec2 next_vertex    = a.vertex((i + 1) == a.size() ? 0 : i + 1);
        Vec2 normal = normalized(perp(next_vertex - current_vertex, true));

        float dist = closest_point(b, current_vertex, normal);
        if (dist > 0.f) {
            return Contacts<2>::none();
        } else if (-dist < min_pen) {
            min_pen              = -dist;
            current_contact_edge = i;
        }
    }

    for (std::size_t i = 0; i < b.size(); ++i) {
        Vec2 current_vertex = b.vertex(i);
        Vec2 next_vertex    = b.vertex((i + 1) == b.size() ? 0 : i + 1);
        Vec2 normal = normalized(perp(next_vertex - current_vertex, true));

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
        std::size_t next_index = current_contact_edge + 1 == a.size()
                                     ? 0
                                     : current_contact_edge + 1;

        Vec2 edge   = a.vertex(next_index) - a.vertex(current_contact_edge);
        Vec2 normal = perp(normalized(edge), true);

        std::size_t opposite_edge_index  = most_opposite_face(b, normal);
        Vec2        opposite_vertex      = b.vertex(opposite_edge_index);
        Vec2        next_opposite_vertex = b.vertex(
            opposite_edge_index + 1 == b.size() ? 0 : opposite_edge_index + 1
        );

        return create_contacts(
            a.vertex(current_contact_edge),
            a.vertex(next_index),
            opposite_vertex,
            next_opposite_vertex,
            normal,
            epsilon
        );
    } else {
        std::size_t next_index = current_contact_edge + 1 == b.size()
                                     ? 0
                                     : current_contact_edge + 1;
        Vec2 edge   = b.vertex(next_index) - b.vertex(current_contact_edge);
        Vec2 normal = perp(normalized(edge), true);

        std::size_t opposite_edge_index  = most_opposite_face(a, normal);
        Vec2        opposite_vertex      = a.vertex(opposite_edge_index);
        Vec2        next_opposite_vertex = a.vertex(
            opposite_edge_index + 1 == a.size() ? 0 : opposite_edge_index + 1
        );

        Contacts<2> contacts = create_contacts(
            b.vertex(current_contact_edge),
            b.vertex(next_index),
            opposite_vertex,
            next_opposite_vertex,
            normal,
            epsilon
        );
        contacts.normal = -contacts.normal;
        std::swap(contacts.contacts_a, contacts.contacts_b);
        return contacts;
    }
}

} // namespace simu