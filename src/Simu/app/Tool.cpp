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

namespace simu
{

////////////////////////////////////////////////////////////
// Grabber
////////////////////////////////////////////////////////////

bool Grabber::onMousePress(simu::Mouse::Input input)
{
    if (input.button != simu::Mouse::Button::left)
        return false;

    if (input.action == simu::Mouse::Action::press)
    {
        scene_.world().forEachAt(input.pos, [&, this](simu::Body* b) {
            if (!b->isStructural())
            {
                if (mc_ != nullptr)
                    mc_->kill();

                mc_ = this->makeMouseConstraint(b, input.pos);
            }
        });
    }
    else if (input.action == simu::Mouse::Action::release)
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

bool Grabber::onMouseMove(simu::Vec2 pos)
{
    if (mc_ != nullptr)
    {
        mc_->updateMousePos(pos);
        return true;
    }

    return false;
}

simu::VisibleMouseConstraint*
Grabber::makeMouseConstraint(simu::Body* b, simu::Vec2 pos)
{
    return scene_.world().makeConstraint<VisibleMouseConstraint>(
        b,
        pos,
        scene_.app()->renderer()
    );
}


////////////////////////////////////////////////////////////
// BoxSpawner
////////////////////////////////////////////////////////////

bool BoxSpawner::onMousePress(simu::Mouse::Input input)
{
    if (input.action == simu::Mouse::Action::press
        && input.button == simu::Mouse::Button::left)
    {
        makeBox(input.pos);
        return true;
    }

    return false;
}


simu::Body* BoxSpawner::makeBox(simu::Vec2 pos)
{
    simu::BodyDescriptor descr{
        simu::Polygon{
                      simu::Vertex{-1.f, -1.f},
                      simu::Vertex{1.f, -1.f},
                      simu::Vertex{1.f, 1.f},
                      simu::Vertex{-1.f, 1.f}}
    };

    descr.position                  = pos;
    descr.material.bounciness.value = 0.f;
    descr.material.friction.value   = 0.5f;
    return scene_.world().makeBody<simu::VisibleBody>(
        descr,
        simu::Rgba{225, 150, 240},
        scene_.app()->renderer()
    );
}
} // namespace simu
