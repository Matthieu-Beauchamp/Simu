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

#include "Simu/config.hpp"

#include "Simu/physics.hpp"

#include "Simu/utility/View.hpp"

#include "Simu/app/Renderer.hpp"
#include "Simu/app/Camera.hpp"
#include "Simu/app/Event.hpp"
#include "Simu/app/VisiblePhysics.hpp"

namespace simu
{

class Application;

class Scene
{
public:

    Scene()          = default;
    virtual ~Scene() = default;

    bool isInit() const { return isInit_; }

protected:

    virtual void init(Renderer& renderer) = 0;

    virtual void onClear(){};
    virtual void preStep(float /* dt */);

    virtual void postStep(float /* dt */){};

public:

    // TODO: In protected...

    // returns true if the input was used
    // base Scene will use escape to close() the application
    virtual bool onKeypress(Keyboard::Input input);

    virtual bool onMousePress(Mouse::Input /* input */) { return false; }
    virtual bool onMouseMove(Vec2 /* newPos */) { return false; }
    virtual bool onMouseScroll(Vec2 scroll)
    {
        float zoomRatio = (scroll[1] < 0.f) ? 4.f / 5.f : 5.f / 4.f;
        camera().setZoom(camera().zoom() * std::abs(scroll[1]) * zoomRatio);
        return true;
    }

    /////////////

    void reset()
    {
        world_ = makeWorld();
        this->init(*renderer_);
    }

    void step(float dt)
    {
        renderer_->fillScreen(Rgba{50, 10, 50, 255});
        renderer_->setCameraTransform(camera_.transform());

        this->preStep(dt);
        world_.step(dt);
        this->postStep(dt);

        renderer_->flush();
    }

    Camera&       camera() { return camera_; }
    const Camera& camera() const { return camera_; }

    Application*       app() { return app_; }
    const Application* app() const { return app_; }

    World&       world() { return world_; }
    const World& world() const { return world_; }

private:

    // called by Application
    friend Application;
    void init(Application* app);

    World makeWorld()
    {
        Renderer* r = this->renderer_;
        return World([=](Bodies<2> bodies) {
            return std::make_unique<VisibleContactConstraint>(bodies, r);
        });
    }

    World  world_{};
    Camera camera_{};

    Renderer*    renderer_ = nullptr;
    Application* app_      = nullptr;

    bool isInit_ = false;
};


} // namespace simu
