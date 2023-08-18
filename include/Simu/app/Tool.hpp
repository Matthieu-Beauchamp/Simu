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

    // called inside an ImGui window Begin/End block
    virtual void        doGui() = 0;

    // Must be unique. Must not change during execution
    virtual const char* getName() const { return ""; }

    virtual bool onKeypress(Keyboard::Input input) = 0;
    virtual bool onMousePress(Mouse::Input input)  = 0;
    virtual bool onMouseMove(Vec2 newPos)          = 0;
    virtual bool onMouseScroll(Vec2 scroll)        = 0;
};

class NoTool : public Tool
{
public:

    static constexpr char name[] = "No Tool";

    NoTool() = default;

    void        doGui() override {}
    const char* getName() const override { return name; }

    bool onKeypress(Keyboard::Input /* input */) override { return false; }
    bool onMousePress(Mouse::Input /* input */) override { return false; }
    bool onMouseMove(Vec2 /* newPos */) override { return false; }
    bool onMouseScroll(Vec2 /* scroll */) override { return false; }
};


class Scene;

class Grabber : public Tool
{
public:

    static constexpr char name[] = "Grabber";


    Grabber(Scene& scene) : scene_{scene} {}

    void        doGui() override {}
    const char* getName() const override { return name; }

    bool onMousePress(Mouse::Input input) override;
    bool onMouseMove(Vec2 pos) override;

    bool onKeypress(Keyboard::Input /* input */) override { return false; }
    bool onMouseScroll(Vec2 /* scroll */) override { return false; }

private:

    VisibleMouseConstraint* makeMouseConstraint(Body* b, Vec2 pos);

    VisibleMouseConstraint* mc_ = nullptr;
    Scene&                  scene_;
};


class BoxSpawner : public Tool
{
public:

    static constexpr char name[] = "Box Spawner";

    BoxSpawner(Scene& scene) : scene_{scene} {}

    void        doGui() override;
    const char* getName() const override { return name; }

    bool onMousePress(Mouse::Input input) override;
    bool onMouseMove(Vec2 /* pos */) override { return false; }

    bool onKeypress(Keyboard::Input /* input */) override { return false; }
    bool onMouseScroll(Vec2 /* scroll */) override { return false; }

    Body* makeBox(Vec2 pos, std::optional<Vec2> dims = std::nullopt);


    float orientation = 0.f;
    float dominance   = 1.f;
    Vec2  dims{2.f, 2.f};
    
    float density    = 1.f;
    float friction   = 0.5f;
    float bounciness = 0.f;

    Rgba color{225, 150, 240, 255};

private:

    Scene& scene_;
};

} // namespace simu
