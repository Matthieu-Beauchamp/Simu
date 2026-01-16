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

#include <array>
#include <ranges>
#include <format>
#include "Simu/math/Matrix.hpp"

namespace simu
{


/// A convex polygon with vertices in a positive orientation (vertices are ordered counter-clockwise)
class Polygon
{
public:

    static constexpr int         max_vertices = 8;
    static constexpr const char* error_msg
        = "Polygon cannot have more than 8 vertices";

    template <class R>
        requires std::ranges::sized_range<R>
                 && std::same_as<std::ranges::range_value_t<R>, Vec2>
    Polygon(const R& vertices) : _n_vertices{vertices.size()} {
        SIMU_ASSERT(vertices.size() <= max_vertices, error_msg);

        std::copy(vertices.begin(), vertices.end(), _vertices.begin());
    };

    Polygon(const std::initializer_list<Vec2>& vertices)
        : _n_vertices{vertices.size()} {
        SIMU_ASSERT(vertices.size() <= max_vertices, error_msg);

        std::copy(vertices.begin(), vertices.end(), _vertices.begin());
    };

    /// Creates a box-shaped polygon from the given dimensions (width, height) and center
    static Polygon box(Vec2 dim, Vec2 center = Vec2{}) {
        float w = dim[0] / 2.f;
        float h = dim[1] / 2.f;

        return Polygon{
            {center + Vec2{-w, -h},
             center + Vec2{w, -h},
             center + Vec2{w, h},
             center + Vec2{-w, h}}
        };
    }

    [[nodiscard]] auto begin() const { return _vertices.begin(); }
    [[nodiscard]] auto end() const { return _vertices.begin() + size(); }

    [[nodiscard]] std::size_t size() const { return _n_vertices; }
    [[nodiscard]] const Vec2& vertex(std::size_t i) const { return _vertices[i]; }

private:

    std::array<Vec2, max_vertices> _vertices;
    std::size_t                    _n_vertices;
};

} // namespace simu
