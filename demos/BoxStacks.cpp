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

BoxStacks::BoxStacks()
{
    registerAllTools();
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
}

void BoxStacks::init(simu::Renderer& renderer)
{
    renderer.setPointPrecision(4);
    renderer.setPointRadius(0.1f);
    renderer.setLineWidth(0.1f);

    simu::BoxSpawner spawner{*this};
    auto             dims = spawner.dims;

    float      floorWidth  = (nStacks_ * 2 + 4) * dims[0];
    float      floorHeight = 20.f;
    simu::Vec2 center{0.f, -floorHeight / 2.f - 1.f};

    for (int stack = 0; stack < nStacks_; ++stack)
    {
        for (int h = 0; h < height_; ++h)
        {
            float x = -floorWidth / 2.f + dims[0] * (1 + (stack+1) * 2);
            float y = h * dims[1];
            spawner.makeBox(simu::Vec2{x, y});
        }
    }

    world().makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

    simu::BodyDescriptor     descr{};
    simu::ColliderDescriptor cDescr{
        simu::Polygon::box(simu::Vec2{floorWidth + 2.f*height_*dims[1], floorHeight}, center)};

    descr.dominance                = 0.f;
    cDescr.material.friction.value = 0.8f;
    world()
        .makeBody<simu::VisibleBody>(descr, simu::Rgba{0, 0, 0, 255}, &renderer)
        ->addCollider(cDescr);
}

void BoxStacks::doGui()
{
    ImGui::SliderInt("Number of stacks", &nStacks_, 1, 100);
    ImGui::SliderInt("Stack height", &height_, 1, 100);
}
