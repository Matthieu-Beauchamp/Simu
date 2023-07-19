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

#include "Simu/app.hpp"


class Tower : public simu::Scene
{
public:

    Tower()
    {
        camera().setPixelSize(1.f / 10.f);
        camera().panTo(simu::Vec2{10.f, 0.f});
    }

    void init(simu::Renderer& renderer) override
    {
        world().makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

        simu::BodyDescriptor descr{
            simu::Polygon{
                          simu::Vertex{-100.f, -20.f},
                          simu::Vertex{100.f, -20.f},
                          simu::Vertex{100.f, -1.f},
                          simu::Vertex{-100.f, -1.f}}
        };

        descr.dominance               = 0.f;
        descr.material.friction.value = 0.8f;
        world().makeBody<simu::VisibleBody>(descr, simu::Rgba{}, &renderer);

        simu::BoxSpawner spawn{*this};

        int  h       = 10;
        bool bricked = true;

        for (int y = 0; y < h; ++y)
            if (bricked)
                for (int x = 0; x < h - y; ++x)
                    spawn.makeBox(simu::Vec2{2.f * x + y, 2.f * y});
            else
                for (int x = y; x < 2*h - y; ++x)
                    spawn.makeBox(simu::Vec2{2.f * x, 2.f * y});

        // Baumgarte is much more stable for bricked pyramid, NGS needs a LOT of iterations and still falls apart pretty fast.
        auto s = world().settings();
        // s.nPositionIterations = 40;
        // s.nVelocityIterations = 10;
        world().updateSettings(s);

        useTool<simu::Grabber>(*this);
    }

    bool onKeypress(simu::Keyboard::Input input) override
    {
        if (input.action != simu::Mouse::Action::press)
            return false;

        if (input.key == simu::Keyboard::Key::G)
            useTool<simu::Grabber>(*this);
        else if (input.key == simu::Keyboard::Key::B)
            useTool<simu::BoxSpawner>(*this);
        else
            return false;

        return true;
    }
};

class DummyApp : public simu::Application
{
public:

    DummyApp() = default;

    std::shared_ptr<simu::Scene>
    nextScene(std::shared_ptr<simu::Scene> current) override
    {
        if (current == nullptr)
        {
            return std::make_shared<Tower>();
        }

        return current;
    }
};

int main()
{
    DummyApp dummy{};
    dummy.run();
    return 0;
}