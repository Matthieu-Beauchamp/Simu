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


class Friction : public simu::Scene
{
public:

    Friction()
    {
        camera().setPixelSize(1.f / 10.f);
        camera().panTo(simu::Vec2{-90.f, 0.f});
    }

    void init(simu::Renderer& renderer) override
    {
        renderer.setPointPrecision(4);
        renderer.setPointRadius(0.1f);
        renderer.setLineWidth(0.1f);

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
        world().makeBody<simu::VisibleBody>(
            descr, simu::Rgba{0, 0, 0, 255}, &renderer
        );

        auto box = simu::BoxSpawner{*this}.makeBox(simu::Vec2{-90.f, 0.f});
        box->setVelocity(simu::Vec2{10.f, 0.f});

        useTool<simu::Grabber>(*this);
    }

    bool onKeypress(simu::Keyboard::Input input) override
    {
        if (simu::Scene::onKeypress(input))
            return true;

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


int main()
{
    simu::Application dummy{};
    dummy.registerScene<Friction>("Friction");
    dummy.run();
    return 0;
}