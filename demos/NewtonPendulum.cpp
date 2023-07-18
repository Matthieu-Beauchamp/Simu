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
                          simu::Vertex{-5.f, 2.f},
                          simu::Vertex{5.f, 2.f},
                          simu::Vertex{5.f, 3.f},
                          simu::Vertex{-5.f, 3.f}}
        };

        descr.dominance = 0.f;
        auto bar
            = world().makeBody<simu::VisibleBody>(descr, simu::Rgba{}, &renderer);


        simu::Vertices v{};
        for (int i = 0; i < 12; ++i)
        {
            float theta = i / (2.f * std::numbers::pi_v<float>);
            v.emplace_back(simu::Vec2{std::cos(theta), std::sin(theta)});
        }
        descr.polygon = simu::Polygon{v.begin(), v.end()};

        for (int i = 0; i < 5; ++i)
        {
            // TODO: distance constraint from ball to bar.
        }
    }


private:

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