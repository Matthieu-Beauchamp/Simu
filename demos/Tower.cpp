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

#include <numbers>

#include "Demos.hpp"


Tower::Tower()
{
    registerAllTools();
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
    camera().panTo(simu::Vec2{0.f, 0.f});
}

void Tower::init(simu::Renderer& renderer)
{
    renderer.setPointPrecision(4);
    renderer.setPointRadius(0.1f);
    renderer.setLineWidth(0.1f);

    world().makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

    simu::BodyDescriptor     descr{};
    simu::ColliderDescriptor cDescr{
        simu::Polygon{
                      simu::Vertex{-100.f, -20.f},
                      simu::Vertex{100.f, -20.f},
                      simu::Vertex{100.f, 0.f},
                      simu::Vertex{-100.f, 0.f}}
    };

    descr.dominance                = 0.f;
    cDescr.material.friction.value = 0.8f;
    world()
        .makeBody<simu::VisibleBody>(descr, simu::Rgba{0, 0, 0, 255}, &renderer)
        ->addCollider(cDescr);


    int h = 20;

    for (int y = 0; y < h; ++y)
    {
        makeSlab(
            simu::Vec2{-w / 2.f + thickness / 2.f, w / 2.f + y * (w + thickness)}, true
        );
        makeSlab(
            simu::Vec2{w / 2.f - thickness / 2.f, w / 2.f + y * (w + thickness)}, true
        );

        makeSlab(simu::Vec2{0.f, (y + 1) * w + (y + 0.5f) * thickness}, false);
    }
}


void Tower::makeSlab(simu::Vec2 pos, bool vertical)
{
    float width = vertical ? w : 2.f * w;

    simu::BodyDescriptor     d{};
    simu::ColliderDescriptor cDescr{
        simu::Polygon{
                      simu::Vec2{-width / 2.f, -thickness / 2.f},
                      simu::Vec2{width / 2.f, -thickness / 2.f},
                      simu::Vec2{width / 2.f, thickness / 2.f},
                      simu::Vec2{-width / 2.f, thickness / 2.f}}
    };

    d.position    = pos;
    d.orientation = vertical ? std::numbers::pi_v<float> / 2 : 0.f;
    cDescr.material.friction.value = 0.5f;

    world()
        .makeBody<simu::VisibleBody>(d, simu::Rgba::filled(200), app()->renderer())
        ->addCollider(cDescr);
}
