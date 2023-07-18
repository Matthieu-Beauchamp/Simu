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

    void run();

    // TODO: Scenes should know their Application, to allow setting cursors and such.
    Scene*       scene() { return scene_.get(); }
    const Scene* scene() const { return scene_.get(); }

    void close();

    bool isKeyPressed(Keyboard::Key key) const;

    Renderer* renderer() { return renderer_.get(); }

protected:

    virtual std::shared_ptr<Scene> nextScene(std::shared_ptr<Scene> current) = 0;


private:

    void show() const;

    friend void frameBufferResizeCallback(GLFWwindow* window, int w, int h);

    std::shared_ptr<Scene>    scene_    = nullptr;
    std::unique_ptr<Renderer> renderer_ = nullptr;
    GLFWwindow*               window_   = nullptr;
};


} // namespace simu
