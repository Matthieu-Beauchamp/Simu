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

#include "Simu/app/Renderer.hpp"
#include "Simu/physics/Transform.hpp"

namespace simu
{

template <Uint32 precision>
void Renderer::drawPoint(Vec2 P, Rgba color, float radius)
{
    static_assert(precision >= 3, "");

    static std::array<Vec2, precision> offsets{};
    static float                       prevRadius = 1.f;
    static bool                        firstCall  = true;

    if (radius != prevRadius || firstCall)
    {
        constexpr float pi = std::numbers::pi_v<float>;
        Rotation        rot{2.f * pi / precision};
        Vec2            offset = radius * Vec2::i();
        for (Uint8 i = 0; i < precision; ++i)
        {
            offsets[i] = offset;
            offset     = rot * offset;
        }

        firstCall  = false;
        prevRadius = radius;
    }

    std::array<Vec2, precision> vertices{};
    for (Uint8 i = 0; i < precision; ++i)
    {
        vertices[i] = P + offsets[i];
    }

    const auto& cv = vertices;
    drawPolygon(P, makeView(cv.data(), cv.data() + cv.size()), color);
}

} // namespace simu
