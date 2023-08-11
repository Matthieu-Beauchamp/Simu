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


class Pyramid : public simu::Scene
{
public:

    Pyramid()
    {
        camera().setPixelSize(1.f / 10.f);
        camera().panTo(simu::Vec2{0.f, 0.f});
    }

    static constexpr float w = 2.f;
    static constexpr float h = 2.f;


    void init(simu::Renderer& renderer) override
    {
        renderer.setPointPrecision(4);
        renderer.setPointRadius(0.1f);
        renderer.setLineWidth(0.1f);

        world().makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

        simu::BodyDescriptor descr{};
        descr.dominance = 0.f;
        auto ground     = world().makeBody<simu::VisibleBody>(
            descr,
            simu::Rgba{0, 0, 0, 255},
            &renderer
        );

        simu::ColliderDescriptor cDescr{
            simu::Polygon::box(simu::Vec2{400.f, 20.f}, simu::Vec2{0.f, -11.f})};
        cDescr.material.friction.value = 0.8f;

        ground->addCollider(cDescr);

        simu::BoxSpawner spawn{*this};

        int  height  = 60;
        bool bricked = true;
        // only for bricked, otherwise we are only doing stacks.
        float spacing = 0.5f;

        float start = -(w + spacing) * height / 2;

        for (int y = 0; y < height; ++y)
            if (bricked)
                for (int x = 0; x < height - y; ++x)
                    spawn.makeBox(
                        simu::Vec2{
                            start + (w + spacing) * x + y * (1.f + spacing / 2.f),
                            (h + spacing) * y},
                        simu::Vec2{w, h}
                    );
            else
                for (int x = y; x < 2 * height - y; ++x)
                    spawn.makeBox(
                        simu::Vec2{start + w * x, h * y},
                        simu::Vec2{w, h}
                    );

        // Baumgarte is much more stable for bricked pyramid, NGS needs a LOT of iterations and still falls apart pretty fast.
        auto s = world().settings();
        // s.nPositionIterations = 50;
        // s.nVelocityIterations = 50;
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
            return std::make_shared<Pyramid>();
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