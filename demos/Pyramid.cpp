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

#include "Demos.hpp"

#include "imgui.h"

Pyramid::Pyramid()
{
    registerAllTools();
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
    camera().panTo(simu::Vec2{0.f, 0.f});
}


void Pyramid::init(simu::Renderer& renderer)
{
    renderer.setPointPrecision(4);
    renderer.setPointRadius(0.1f);
    renderer.setLineWidth(0.1f);

    world().makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

    simu::BodyDescriptor descr{};
    descr.dominance = 0.f;
    auto ground     = world().makeBody<simu::VisibleBody>(
        descr, simu::Rgba{0, 0, 0, 255}, &renderer
    );

    simu::ColliderDescriptor cDescr{
        simu::Polygon::box(simu::Vec2{400.f, 20.f}, simu::Vec2{0.f, -11.f})};
    cDescr.material.friction.value = 0.8f;

    ground->addCollider(cDescr);

    simu::BoxSpawner spawn{*this};

    float start = -(w + spacing_) * height_ / 2;

    for (int y = 0; y < height_; ++y)
    {
        for (int x = 0; x < height_ - y; ++x)
        {
            spawn.makeBox(
                simu::Vec2{
                    start + (w + spacing_) * x + y * (1.f + spacing_ / 2.f),
                    (h + spacing_) * y},
                simu::Vec2{w, h}
            );
        }
    }
}

void Pyramid::doGui()
{
    ImGui::SliderInt("Height", &height_, 1, 100);
    ImGui::SliderFloat("Spacing", &spacing_, 0.f, w);
}