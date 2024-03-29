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


#include "Simu/physics.hpp"

#include "Simu/app/Scene.hpp"
#include "Simu/app/Event.hpp"

struct GLFWwindow;

namespace simu
{

class Application
{
public:

    Application();
    ~Application();

    void setName(const char* name);

    template <std::derived_from<Scene> S, class... Args>
    void registerScene(const char* name, Args&&... args)
    {
        scenes_[name] = std::make_shared<S>(std::forward<Args>(args)...);

        if (scene_ == nullptr)
            changeScene(scenes_[name]);
    }

    void run();

    // TODO: Scenes should know their Application, to allow setting cursors and such.
    Scene*       scene() { return scene_.get(); }
    const Scene* scene() const { return scene_.get(); }

    void close();

    bool isKeyPressed(Keyboard::Key key) const;

    Renderer* renderer() { return renderer_.get(); }

private:

    void changeScene(std::shared_ptr<Scene> next);

    void doGui(float dt);


    void show() const;

    static void frameBufferResizeCallback(GLFWwindow* window, int w, int h);
    static void windowResizeCallback(GLFWwindow* window, int w, int h);

    static void
    keypressCallback(GLFWwindow* window, int key, int scancode, int action, int modifiers);

    static void mouseMoveCallback(GLFWwindow* window, double x, double y);
    static void
    mousePressCallback(GLFWwindow* window, int button, int action, int mods);

    static void mouseScrollCallback(GLFWwindow* window, double x, double y);


    std::shared_ptr<Scene>    scene_    = nullptr;
    std::unique_ptr<Renderer> renderer_ = nullptr;
    GLFWwindow*               window_   = nullptr;

    std::unordered_map<const char*, std::shared_ptr<Scene>> scenes_{};
};


} // namespace simu
