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
#include "Simu/app/Tool.hpp"

namespace simu
{

class Application;

class Scene
{
public:

    Scene() : world_{}
    {
        auto contactFactory = [this](
                                  Collider&                           first,
                                  Collider&                           second,
                                  const typename World::ContactAlloc& alloc
                              ) {
            return makeUnique<VisibleContactConstraint>(
                alloc, first, second, this->renderer_
            );
        };

        world_.setContactFactory(contactFactory);
        tool_ = tools_.emplace_back(std::make_unique<NoTool>()).get();
    }

    virtual ~Scene() = default;

    bool isInit() const { return isInit_; }
    void reset()
    {
        world_.clear();
        this->init(*renderer_);
    }

    // play speeds above 1 may hinder the physics accuracy
    void setPlaySpeed(float speed)
    {
        if (speed > 0.f)
            playSpeed_ = speed;
    }
    float playSpeed() const { return playSpeed_; }

    void resume() { isPaused_ = false; }
    void pause() { isPaused_ = true; }
    bool isPaused() const { return isPaused_; }

    void step(float dt)
    {
        dt = std::min(dt, 1.f / 60.f);

        this->moveCamera(dt);
        renderer_->setCameraTransform(camera_.transform());

        renderer_->fillScreen(Rgba{50, 10, 50, 255});

        dt *= playSpeed() * static_cast<float>(!isPaused());
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

protected:

    Renderer* getRenderer() const { return renderer_; }

    virtual void init(Renderer& renderer) = 0;

    // called inside an ImGui window Begin/End block
    virtual void doGui() {}

    virtual void onClear(){};
    virtual void preStep(float /* dt */){};
    virtual void postStep(float /* dt */){};

    virtual void moveCamera(float dt);

    // returns true if the input was used, derived classes should always override
    //  and call Base::on...

    // base Scene will use:
    // escape to close() the application
    // P to pause/resume
    // - and = to slow and speed the simulation respectively
    // s to single step
    // r to reset() the scene
    virtual bool onKeypress(Keyboard::Input input);
    virtual bool onMousePress(Mouse::Input /* input */) { return false; }
    virtual bool onMouseMove(Vec2 /* newPos */) { return false; }

    // zooms in or out
    virtual bool onMouseScroll(Vec2 scroll);

    // Tools should be registered in the constructor, only once per tool.
    template <std::derived_from<Tool> T, class... Args>
    T* registerTool(Args&&... args)
    {
        tools_.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
        return static_cast<T*>(tools_.back().get());
    }

    void registerAllTools()
    {
        registerTool<Grabber>(*this);
        registerTool<BoxSpawner>(*this);
    }

    template <std::derived_from<Tool> T>
    T* useTool()
    {
        return static_cast<T*>(useTool(T::name));
    }

    Tool* useTool(const std::string& name)
    {
        for (const auto& t : tools_)
        {
            if (t->getName() == name)
            {
                tool_ = t.get();
                return currentTool();
            }
        }

        SIMU_ASSERT(false, "No tool of this class was registered");
        return tools_.front().get();
    }

    Tool* currentTool() const { return tool_; }

private:

    // called by Application
    friend Application;
    void init(Application* app);
    void keypress(Keyboard::Input input);
    void mousePress(Mouse::Input input);
    void mouseMove(Vec2 newPos);
    void mouseScroll(Vec2 scroll);

    World  world_{};
    Camera camera_{};

    Tool*                            tool_ = nullptr;
    std::list<std::unique_ptr<Tool>> tools_{};

    Renderer*    renderer_ = nullptr;
    Application* app_      = nullptr;

    float playSpeed_ = 1.f;
    bool  isPaused_  = false;
    bool  isInit_    = false;
};


} // namespace simu
