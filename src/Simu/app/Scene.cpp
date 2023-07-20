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

#include "Simu/app/Scene.hpp"
#include "Simu/app/Application.hpp"

namespace simu
{

void Scene::moveCamera(float dt)
{
    if (app() == nullptr)
        return;

    // 500 pixel / s at normal zoom
    float panSpeed = 500.f * camera().pixelSize() / camera().zoom();

    Vec2 panDir{};
    if (app()->isKeyPressed(Keyboard::Key::left))
        panDir += Vec2{-1.f, 0.f};
    if (app()->isKeyPressed(Keyboard::Key::right))
        panDir += Vec2{1.f, 0.f};
    if (app()->isKeyPressed(Keyboard::Key::up))
        panDir += Vec2{0.f, 1.f};
    if (app()->isKeyPressed(Keyboard::Key::down))
        panDir += Vec2{0.f, -1.f};

    if (any(panDir != Vec2{0.f, 0.f}))
        camera().pan(normalized(panDir) * panSpeed * dt);
};

bool Scene::onKeypress(Keyboard::Input input)
{
    if (input.action == Keyboard::Action::release)
        return false;


    if (input.key == Keyboard::Key::S)
    {
        if (isPaused())
        {
            resume();
            step(1.f / 60.f);
            pause();
        }
        else
            step(1.f / 60.f);

        return true;
    }

    if (input.action == Keyboard::Action::press)
    {
        switch (input.key)
        {
            case Keyboard::Key::escape:
            {
                if (app() != nullptr)
                    app()->close();

                break;
            }
            case Keyboard::Key::P:
            {
                isPaused() ? resume() : pause();
                break;
            }
            case Keyboard::Key::minus:
            {
                setPlaySpeed(playSpeed() / 2.f);
                break;
            }
            case Keyboard::Key::equal:
            {
                setPlaySpeed(playSpeed() * 2.f);
                break;
            }
            default: return false;
        }
    }
    return true;
}

bool Scene::onMouseScroll(Vec2 scroll)
{
    float zoomRatio = (scroll[1] < 0.f) ? 4.f / 5.f : 5.f / 4.f;
    camera().setZoom(camera().zoom() * std::abs(scroll[1]) * zoomRatio);
    return true;
}


void Scene::init(Application* app)
{
    app_      = app;
    renderer_ = app->renderer();

    reset();

    isInit_ = true;
}

void Scene::keypress(Keyboard::Input input)
{
    tool_->onKeypress(input) || onKeypress(input);
}

void Scene::mousePress(Mouse::Input input)
{
    tool_->onMousePress(input) || onMousePress(input);
}

void Scene::mouseMove(Vec2 newPos)
{
    tool_->onMouseMove(newPos) || onMouseMove(newPos);
}

void Scene::mouseScroll(Vec2 scroll)
{
    tool_->onMouseScroll(scroll) || onMouseScroll(scroll);
}

} // namespace simu
