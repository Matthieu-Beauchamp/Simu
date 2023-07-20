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

#include "Simu/app/Event.hpp"
#include "Simu/app/VisiblePhysics.hpp"

namespace simu
{

class Tool
{
public:

    Tool()          = default;
    virtual ~Tool() = default;

    virtual bool onKeypress(Keyboard::Input input) = 0;
    virtual bool onMousePress(Mouse::Input input)  = 0;
    virtual bool onMouseMove(Vec2 newPos)          = 0;
    virtual bool onMouseScroll(Vec2 scroll)        = 0;
};

class NoTool : public Tool
{
public:

    NoTool() = default;

    bool onKeypress(Keyboard::Input /* input */) override { return false; }
    bool onMousePress(Mouse::Input /* input */) override { return false; }
    bool onMouseMove(Vec2 /* newPos */) override { return false; }
    bool onMouseScroll(Vec2 /* scroll */) override { return false; }
};


class Scene;

class Grabber : public Tool
{
public:

    Grabber(Scene& scene) : scene_{scene} {}

    bool onMousePress(simu::Mouse::Input input) override;
    bool onMouseMove(simu::Vec2 pos) override;

    bool onKeypress(Keyboard::Input /* input */) override { return false; }
    bool onMouseScroll(Vec2 /* scroll */) override { return false; }

private:

    simu::VisibleMouseConstraint*
    makeMouseConstraint(simu::Body* b, simu::Vec2 pos);

    VisibleMouseConstraint* mc_ = nullptr;
    Scene&                  scene_;
};


class BoxSpawner : public Tool
{
public:

    BoxSpawner(Scene& scene) : scene_{scene} {}

    bool onMousePress(simu::Mouse::Input input) override;
    bool onMouseMove(simu::Vec2 /* pos */) override { return false; }

    bool onKeypress(Keyboard::Input /* input */) override { return false; }
    bool onMouseScroll(Vec2 /* scroll */) override { return false; }

    simu::Body* makeBox(simu::Vec2 pos, simu::Vec2 dims = simu::Vec2{2.f, 2.f});

private:

    Scene& scene_;
};

} // namespace simu
