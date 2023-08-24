////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2023 Matthieu Beauchamp-Boulay
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

#include <algorithm>
#include <array>

#include "Simu/math/Shape.hpp"
#include "Simu/math/Edges.hpp"

#include "Simu/utility/View.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief Compute properties of some non self-intersecting Geometry.
///
/// Convex, concave and geometry containing holes are all valid input.
/// The hole(s) should be of opposite Orientation and attached to a vertex
///     of the outer (solid) perimeter.
///
/// if area == 0, then the geometry isDegenerate (collinear) and the
///     properties are undefined but no exception is raised
///
/// If vertices are negatively oriented, then the area is negative, but the
///     centroid and momentOfArea are unaffected.
///
////////////////////////////////////////////////////////////
template <Geometry G>
Shape::Properties computeProperties(const G& geometry)
{
    // These formulas can be derived from the definitions with a double
    // integral. Using Green's theorem to reduce to an integral over the contour
    //  (edges) and evaluating to a sum.

    Shape::Properties p;

    for (const auto& edge : edgesOf(geometry))
    {
        float vertexCross = cross(edge.from(), edge.to());
        p.area += vertexCross;

        p.centroid += (edge.from() + edge.to()) * vertexCross;

        p.momentOfArea += vertexCross
                          * (normSquared(edge.from()) + dot(edge.from(), edge.to())
                             + normSquared(edge.to()));
    }

    if (p.area == 0.f)
    {
        p.isDegenerate = true;
    }
    else
    {
        p.area /= 2;
        p.centroid /= 6 * p.area;
        p.momentOfArea /= 12;

        // relative to center of mass (parallel axis theorem).
        p.momentOfArea -= p.area * normSquared(p.centroid);

        // in case of negative orientation.
        p.momentOfArea = std::abs(p.momentOfArea);
    }

    return p;
}

////////////////////////////////////////////////////////////
/// \brief A polygon has at least 3 vertices in a positive Orientation
///
/// While the Shape::Properties will be correct if the Polygon is concave or has holes
///  as long as it does not self-intersect, the collision detection will only
///  take the Polygon's convex hull into accout.
///
/// The vertices are reordered to be positively oriented.
///
/// \warning no special measures are taken if properties indicate that the geometry isDegenerate.
////////////////////////////////////////////////////////////
class Polygon final : public Shape
{
public:

    // Avoid dealing with allocators.
    static constexpr Uint32 maxVertices = 10;

    Polygon(const std::initializer_list<Vec2>& vertices)
        : Polygon{vertices.begin(), vertices.end()}
    {
    }

    template <VertexIterator It>
    Polygon(It begin, It end) : Shape{polygon}
    {
        auto n = std::distance(begin, end);
        SIMU_ASSERT(n >= 3, "Convex Geometry must have at least 3 vertices");
        SIMU_ASSERT(n <= maxVertices, "Too many vertices.");

        while (begin != end && nVertices < maxVertices)
            vertices_[nVertices++] = *begin++;

        properties_ = computeProperties(*this);
        if (properties_.area < 0)
        {
            std::reverse(vertices_.data(), vertices_.data() + nVertices);
            properties_.area = -properties_.area;
        }
    }

    static Polygon box(Vec2 dim, Vec2 center = Vec2{})
    {
        float w = dim[0] / 2.f;
        float h = dim[1] / 2.f;

        return Polygon{
            center + Vec2{-w, -h},
            center + Vec2{w,  -h},
            center + Vec2{w,  h },
            center + Vec2{-w, h }
        };
    }

    void copyAt(Shape* dest) const override
    {
        std::construct_at(static_cast<Polygon*>(dest), *this);
    }

    Properties properties() const override { return properties_; }

    BoundingBox boundingBox() const override { return BoundingBox{*this}; }

    void transform(const Transform& transform) override
    {
        for (Uint32 i = 0; i < nVertices; ++i)
        {
            vertices_[i] = transform * vertices_[i];
        }

        properties_.centroid = transform * properties_.centroid;
    }

    const Vec2* begin() const { return vertices_.data(); }
    const Vec2* end() const { return vertices_.data() + nVertices; }

    auto vertexView() const { return makeView(*this); }

private:

    std::array<Vec2, maxVertices> vertices_;
    Uint32                        nVertices = 0;
    Properties                    properties_;
};

} // namespace simu
