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

class NewtonPendulum : public simu::Scene
{
public:

    NewtonPendulum()
    {
        camera().setPixelSize(1.f / 10.f);
        camera().panTo(simu::Vec2{0.f, 0.f});
    }

    void init(simu::Renderer& renderer) override
    {
        simu::BodyDescriptor descr{
            simu::Polygon{
                          simu::Vertex{-5.f, 8.f},
                          simu::Vertex{5.f, 8.f},
                          simu::Vertex{5.f, 11.f},
                          simu::Vertex{-5.f, 11.f}}
        };

        descr.dominance = 0.f;
        auto bar
            = world().makeBody<simu::VisibleBody>(descr, simu::Rgba{}, &renderer);


        simu::Vertices v{};

        // TODO: The fact that we don't have vertex-vertex contacts is very apparent here,
        int   nPoints = 48;
        float theta   = 0.f;
        for (int i = 0; i < nPoints; ++i)
        {
            theta += (2.f * std::numbers::pi_v<float>) / nPoints;
            v.emplace_back(simu::Vec2{std::cos(theta), std::sin(theta)});
        }

        descr.polygon                   = simu::Polygon{v.begin(), v.end()};
        descr.dominance                 = 1.f;
        descr.material.density          = 10.f;
        descr.material.bounciness.value = 1.f;
        for (int i = 0; i < 5; ++i)
        {
            float x = -4.f + 2 * i;

            descr.position = simu::Vec2{x, 0.f};
            auto ball      = world().makeBody<simu::VisibleBody>(
                descr,
                simu::Rgba::filled(200),
                &renderer
            );
            world().makeConstraint<simu::VisibleDistanceConstraint>(
                simu::Bodies<2>{
                    ball,
                    bar
            },
                std::array<simu::Vec2, 2>{simu::Vec2{x, 1.f}, simu::Vec2{x, 8.f}},
                &renderer,
                std::nullopt,
                false
            );

            // if (i >= 2)
            //     ball->setVelocity(simu::Vec2{7.f, 0.f});
        }

        world().makeForceField<simu::Gravity>(simu::Vec2{0, -10.f});

        auto settings = world().settings();
        world().updateSettings(settings);
    }

    bool onMousePress(simu::Mouse::Input input) override
    {
        if (input.button != simu::Mouse::Button::left)
            return false;

        if (input.action == simu::Mouse::Action::press)
        {
            world().forEachAt(input.pos, [&, this](simu::Body* b) {
                if (!b->isStructural())
                    mc_ = this->makeMouseConstraint(b, input.pos);
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

    bool onMouseMove(simu::Vec2 pos) override
    {
        if (mc_ != nullptr)
        {
            mc_->updateMousePos(pos);
            return true;
        }
    
        return false;
    }


private:

    simu::VisibleMouseConstraint*
    makeMouseConstraint(simu::Body* b, simu::Vec2 pos)
    {
        return world().makeConstraint<simu::VisibleMouseConstraint>(
            b,
            pos,
            app()->renderer()
        );
    }

    simu::VisibleMouseConstraint* mc_ = nullptr;
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
            return std::make_shared<NewtonPendulum>();
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