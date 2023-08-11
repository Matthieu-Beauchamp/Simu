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


class Tumbler : public simu::Scene
{
public:

    static constexpr int maxCount = 1500;

    Tumbler()
    {
        camera().setPixelSize(1.f / 20.f);
        camera().panTo(simu::Vec2{0.f, 0.f});
    }


    void init(simu::Renderer& renderer) override
    {
        count_ = 0;
        renderer.setPointPrecision(4);
        renderer.setPointRadius(0.01f);
        renderer.setLineWidth(0.01f);


        world().makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

        constexpr float pi = std::numbers::pi_v<float>;

        constexpr float size      = 20.f;
        constexpr float thickness = 1.f;

        simu::Rgba           black{0, 0, 0, 255};
        simu::BodyDescriptor descr{};
        descr.dominance = 0.f;
        auto tumbler = world().makeBody<simu::VisibleBody>(descr, black, &renderer);

        simu::Vec2 horizontalDim{size, thickness};
        simu::Vec2 verticalDim{thickness, size};

        simu::Vec2 xOffset{size / 2.f, 0.f};
        simu::Vec2 yOffset{0.f, size / 2.f};

        simu::ColliderDescriptor cDescr{
            simu::Polygon::box(horizontalDim, yOffset)};
        cDescr.material.density        = 5.f;
        cDescr.material.friction.value = 0.8f;

        tumbler->addCollider(cDescr);
        cDescr.polygon = simu::Polygon::box(horizontalDim, -yOffset);
        tumbler->addCollider(cDescr);
        cDescr.polygon = simu::Polygon::box(verticalDim, xOffset);
        tumbler->addCollider(cDescr);
        cDescr.polygon = simu::Polygon::box(verticalDim, -xOffset);
        tumbler->addCollider(cDescr);

        auto motor = world().makeConstraint<simu::RotationMotor>(
            simu::Bodies::singleBody(tumbler),
            simu::RotationMotor::Specs::fromTorque(0.15f * pi, 1e8f)
        );
        motor->direction(simu::Vector<float, 1>{-1.f});

        auto s = world().settings();
        // s.nPositionIterations = 50;
        // s.nVelocityIterations = 50;
        world().updateSettings(s);

        useTool<simu::Grabber>(*this);
    }

    void postStep(float) override
    {
        if (isPaused())
            return;

        if (count_ < maxCount)
        {
            simu::BodyDescriptor descr{};

            auto b = world().makeBody<simu::VisibleBody>(
                descr,
                simu::Rgba{200, 100, 200, 255},
                getRenderer()
            );

            simu::ColliderDescriptor cDescr{
                simu::Polygon::box(simu::Vec2{0.25f, 0.25f})};
            cDescr.material.friction.value = 0.5f;
            b->addCollider(cDescr);
            ++count_;
        }
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

    int count_ = 0;
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
            return std::make_shared<Tumbler>();
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