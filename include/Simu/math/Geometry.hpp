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

#include <vector>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/math/GeometricProperties.hpp"

namespace simu
{

template <class T, Uint32 dim>
concept VertexIterator = requires(T it) {
    {
        *it
    } -> std::convertible_to<Vector<float, dim>>;
    {
        ++it
    };
};

template <class T>
concept VertexIterator2D = VertexIterator<T, 2>;


class ConvexGeometry;
typedef std::vector<ConvexGeometry> Polygons;

typedef Vec2                Vertex;
typedef std::vector<Vertex> Vertices;

class Geometry
{
public:

    virtual ~Geometry() = default;

    // (minkowski support function)
    virtual Vec2 furthestVertexInDirection(const Vec2& direction) const = 0;

    const GeometricProperties& properties() const { return properties_; }

    virtual Polygons polygons() const = 0;

protected:

    GeometricProperties properties_;
};


class ConvexGeometry : public Geometry
{
public:

    ConvexGeometry(const std::initializer_list<Vertex>& vertices)
        : ConvexGeometry(vertices.begin(), vertices.end())
    {
    }

    template <VertexIterator2D It>
    ConvexGeometry(It begin, It end)
    {
        SIMU_ASSERT(
            std::distance(begin, end) >= 3,
            "Convex Geometry must have at least 3 vertices"
        );

        while (begin != end)
            vertices_.emplace_back(*begin++);

        properties_ = GeometricProperties{*this};

        for (Vertex& v : vertices_)
            v -= properties_.centroid;

        properties_.centroid = Vertex{0, 0};
    }

    Vec2 furthestVertexInDirection(const Vec2& direction) const override
    {
        Vec2  furthest = vertices_[0];
        float maxDist  = dot(furthest, direction);
        for (const Vertex& v : vertices_)
        {
            float dist = dot(v, direction);
            if (dist > maxDist)
            {
                maxDist  = dist;
                furthest = v;
            }
        }

        return furthest;
    }


    Polygons polygons() const override { return {*this}; }

    Vertices::const_iterator begin() const { return vertices_.begin(); }
    Vertices::const_iterator end() const { return vertices_.end(); }

private:

    Vertices vertices_;
};

class CompositeGeometry : public Geometry
{
    // TODO:
};

} // namespace simu
