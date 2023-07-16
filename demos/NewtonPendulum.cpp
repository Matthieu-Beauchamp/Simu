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

#include <thread>

#include "Simu/app.hpp"

class Triangle : public simu::Entity
{
public:

    Triangle() {}

    void declarePhysics(simu::World&) override{};

    void draw(simu::Renderer& renderer)
    {
        renderer.drawTriangle(
            simu::Vec2{0, 0},
            simu::Vec2{1, 0},
            simu::Vec2{0.5f, 0.5f},
            simu::Rgba{255, 0, 0}
        );
    }
};

class NewtonPendulum : public simu::Scene
{
public:

    NewtonPendulum() { camera().setPixelSize(1.f / 100.f); }

    void init() override { this->makeEntity<Triangle>(); }
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
            auto scene = std::make_shared<NewtonPendulum>();
            scene->init();
            return scene;
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