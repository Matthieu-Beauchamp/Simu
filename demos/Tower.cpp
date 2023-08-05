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

#include <numbers>

#include "Simu/app.hpp"


class Tower : public simu::Scene
{
public:

    Tower()
    {
        camera().setPixelSize(1.f / 10.f);
        camera().panTo(simu::Vec2{0.f, 0.f});
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
                          simu::Vertex{100.f, 0.f},
                          simu::Vertex{-100.f, 0.f}}
        };

        descr.dominance               = 0.f;
        descr.material.friction.value = 0.8f;
        world().makeBody<simu::VisibleBody>(
            descr,
            simu::Rgba{0, 0, 0, 255},
            &renderer
        );


        int h = 20;

        for (int y = 0; y < h; ++y)
        {
            makeSlab(
                simu::Vec2{
                    -w / 2.f + thickness / 2.f,
                    w / 2.f + y * (w + thickness)},
                true
            );
            makeSlab(
                simu::Vec2{
                    w / 2.f - thickness / 2.f,
                    w / 2.f + y * (w + thickness)},
                true
            );

            makeSlab(simu::Vec2{0.f, (y + 1) * w + (y + 0.5f) * thickness}, false);
        }

        auto s = world().settings();
        // s.nPositionIterations = 5;
        // s.nVelocityIterations = 20;
        world().updateSettings(s);

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

private:

    static constexpr float w         = 5.f;
    static constexpr float thickness = 1.f;

    void makeSlab(simu::Vec2 pos, bool vertical)
    {
        float width = vertical ? w : 2.f * w;

        simu::BodyDescriptor d{
            simu::Polygon{
                          simu::Vec2{-width / 2.f, -thickness / 2.f},
                          simu::Vec2{width / 2.f, -thickness / 2.f},
                          simu::Vec2{width / 2.f, thickness / 2.f},
                          simu::Vec2{-width / 2.f, thickness / 2.f}}
        };

        d.material.friction.value = 0.5f;
        // d.position                = simu::Vec2{pos[0], 1.2f* pos[1]};
        d.position    = pos;
        d.orientation = vertical ? std::numbers::pi_v<float> / 2 : 0.f;

        world().makeBody<simu::VisibleBody>(
            d,
            simu::Rgba::filled(200.f),
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