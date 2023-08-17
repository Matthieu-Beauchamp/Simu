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

#include "Simu/app/Tool.hpp"

#include "Simu/app/Scene.hpp"
#include "Simu/app/Application.hpp"

#include "imgui.h"

namespace simu
{

////////////////////////////////////////////////////////////
// Grabber
////////////////////////////////////////////////////////////

bool Grabber::onMousePress(Mouse::Input input)
{
    if (input.button != Mouse::Button::left)
        return false;

    if (input.action == Mouse::Action::press)
    {
        scene_.world().forEachAt(input.pos, [&, this](Body* b) {
            if (!b->isStructural())
            {
                if (mc_ != nullptr)
                    mc_->kill();

                mc_ = this->makeMouseConstraint(b, input.pos);
            }
        });
    }
    else if (input.action == Mouse::Action::release)
    {
        if (mc_ != nullptr)
        {
            mc_->kill();
            mc_ = nullptr;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool Grabber::onMouseMove(Vec2 pos)
{
    if (mc_ != nullptr)
    {
        mc_->updateMousePos(pos);
        return true;
    }

    return false;
}

VisibleMouseConstraint* Grabber::makeMouseConstraint(Body* b, Vec2 pos)
{
    return scene_.world().makeConstraint<VisibleMouseConstraint>(
        b, pos, scene_.app()->renderer()
    );
}


////////////////////////////////////////////////////////////
// BoxSpawner
////////////////////////////////////////////////////////////

void BoxSpawner::doGui()
{
    ImGui::SliderFloat2("Dimensions", dims.data, 0.1f, 50.f);

    ImGui::SliderFloat("Density", &density, 0.1f, 100.f);
    ImGui::SliderFloat("Friction", &friction, 0.f, 1.f);
    ImGui::SliderFloat("Bounciness", &bounciness, 0.f, 1.f);

    constexpr float pi    = std::numbers::pi_v<float>;
    float           ratio = orientation / (2.f * pi);
    orientation -= std::floor(ratio) * pi;
    int degrees = static_cast<int>(180 * orientation / pi);
    ImGui::SliderInt("Orientation", &degrees, -180, 180);
    orientation = degrees * pi / 180.f;

    Vec4 rgba = color_ / 255.f;
    ImGui::ColorEdit3("Color", rgba.data);
    color_ = static_cast<Rgba>(rgba * 255.f);
}

bool BoxSpawner::onMousePress(Mouse::Input input)
{
    if (input.action == Mouse::Action::press && input.button == Mouse::Button::left)
    {
        makeBox(input.pos);
        return true;
    }

    return false;
}


Body* BoxSpawner::makeBox(Vec2 pos, std::optional<Vec2> dimensions)
{
    BodyDescriptor descr{pos, orientation, dominance};

    auto b = scene_.world().makeBody<VisibleBody>(
        descr, color_, scene_.app()->renderer()
    );

    Material material;
    material.density          = density;
    material.friction.value   = friction;
    material.bounciness.value = bounciness;

    ColliderDescriptor cDescr{
        Polygon::box(dimensions.value_or(this->dims)), material};
    b->addCollider(cDescr);

    return b;
}

} // namespace simu
