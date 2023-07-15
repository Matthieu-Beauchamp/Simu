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

#include "Simu/utility/View.hpp"

#include "Simu/app/Renderer.hpp"
#include "Simu/app/Camera.hpp"
#include "Simu/app/Entity.hpp"
#include "Simu/app/Event.hpp"

namespace simu
{

class Scene
{
public:

    Scene()          = default;
    virtual ~Scene() = default;

    virtual void init() = 0;
    // TODO: virtual void processEvent(Event& e);

protected:

    virtual void onClear(){};
    virtual void preStep(float dt)
    {
        if (app_ == nullptr)
            return;

        float panSpeed = 10.f * camera().pixelSize(); // 10 pixel / s
        Vec2  panDir{};
        if (app_->isKeyPressed(Keyboard::Key::left))
            panDir += Vec2{-1.f, 0.f};
        if (app_->isKeyPressed(Keyboard::Key::right))
            panDir += Vec2{1.f, 0.f};
        if (app_->isKeyPressed(Keyboard::Key::up))
            panDir += Vec2{0.f, 1.f};
        if (app_->isKeyPressed(Keyboard::Key::down))
            panDir += Vec2{0.f, -1.f};
    
        camera().pan(panDir * panSpeed * dt);
    };

    virtual void postStep(float dt){};

public:

    // returns true if the input was used
    // base Scene will use escape to close() the application
    virtual bool onKeypress(Keyboard::Input input)
    {
        if ((input.key == Keyboard::Key::escape)
            && (input.action == Keyboard::Action::press))
        {
            if (app_ != nullptr)
                app_->close();
        }
    }


    auto entities() { return makeView(entities_, DoubleDereference{}); }
    auto entities() const { return makeView(entities_, DoubleDereference{}); }


    void clear()
    {
        this->onClear();
        entities_.clear();
    };

    void reset()
    {
        clear();
        this->init();
    }

    void render(Renderer& renderer)
    {
        renderer.setCameraTransform(camera_.transform());

        for (Entity& e : entities())
            e.draw(renderer);

        renderer.flush();
    }


    void step(float dt)
    {
        this->preStep(dt);
        world_.step(dt);
        this->postStep(dt);
    }

    Camera&       camera() { return camera_; }
    const Camera& camera() const { return camera_; }


protected:

    template <std::derived_from<Entity> T, class... Args>
    T* makeEntity(Args&&... args)
    {
        auto ent = std::make_unique<T>(std::forward<Args>(args)...);
        ent->declarePhysics(world_);
        return entities_.emplace_back(std::move(ent))->get();
    }

    template <std::derived_from<Entity> T>
    void removeEntity(T* entity)
    {
        auto it = std::find_if(
            entities_.begin(),
            entities_.end(),
            [=](const std::unique_ptr<Entity>& ptr) -> bool {
                return ptr->get() == entity;
            }
        );

        if (it != entities_.end())
            entities_.erase(it);
    }

private:

    World                              world_{};
    std::list<std::unique_ptr<Entity>> entities_{};
    Camera                             camera_{};

    friend class Application;
    Application* app_ = nullptr;
};


} // namespace simu
