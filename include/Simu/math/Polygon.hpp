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

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/math/Geometry.hpp"

#include "Simu/utility/View.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \ingroup Geometry
/// \brief A polygon has at least 3 vertices in a positive Orientation
///
/// Polygons are allowed to be concave and have holes as long as they do not
///     self-intersect, in which case behavior is undefined.
///
/// The vertices are reordered to be positively oriented.
/// The GeometricProperties are modified to reflect this change.
///
/// Polygons are not allowed to modify their vertices after construction.
///
/// \warning no special measures are taken if properties indicate that the geometry isDegenerate.
////////////////////////////////////////////////////////////
class Polygon
{
public:

    Polygon(const std::initializer_list<Vertex>& vertices)
        : Polygon{vertices.begin(), vertices.end()}
    {
    }

    template <VertexIterator2D It>
    Polygon(It begin, It end);

    static Polygon box(Vec2 dim, Vec2 center = Vec2{})
    {
        float w = dim[0] / 2.f;
        float h = dim[1] / 2.f;

        return Polygon{
            center + Vertex{-w, -h},
            center + Vertex{w,  -h},
            center + Vertex{w,  h },
            center + Vertex{-w, h }
        };
    }

    GeometricProperties properties() const { return properties_; }

    Vertices::const_iterator begin() const { return vertices_.begin(); }
    Vertices::const_iterator end() const { return vertices_.end(); }

    auto vertexView() const
    {
        return makeView(vertices_.data(), vertices_.data() + vertices_.size());
    }

private:

    GeometricProperties properties_;
    Vertices            vertices_;
};

} // namespace simu

#include "Simu/math/Polygon.inl.hpp"
