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


class BoxStacks : public simu::Scene
{
public:

    BoxStacks() { camera().setPixelSize(1.f / 10.f); }

    void init(simu::Renderer& renderer) override
    {
        for (simu::Int32 x = -90; x < 90; x+= 5)
            for (simu::Uint32 i = 0; i < 10; ++i)
                makeBox(simu::Vec2{(float)x, (float)5 * i});


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
    }

    bool onMousePress(simu::Mouse::Input input) override
    {
        if (input.action == simu::Mouse::Action::press
            && input.button == simu::Mouse::Button::left)
        {
            makeBox(input.pos);
            return true;
        }

        return false;
    }

private:

    void makeBox(simu::Vec2 pos)
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
        world().makeBody<simu::VisibleBody>(
            descr,
            simu::Rgba{225, 150, 240},
            app()->renderer()
        );
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
            return std::make_shared<BoxStacks>();
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