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

void Scene::preStep(float dt)
{
    if (app() == nullptr)
        return;

    float panSpeed = 100.f * camera().pixelSize(); // 100 pixel / s
    Vec2  panDir{};
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
    if ((input.key == Keyboard::Key::escape)
        && (input.action == Keyboard::Action::press))
    {
        if (app() != nullptr)
            app()->close();

        return true;
    }

    return false;
}
} // namespace simu
