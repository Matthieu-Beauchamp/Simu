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

#include <sstream>

#include "GLFW/glfw3.h"
#include "glbinding/glbinding.h"


#include "Simu/app/Application.hpp"

namespace simu
{

void glfwErrorCallback(int error, const char* description)
{
    std::stringstream err;
    err << "GLFW error code " << error << ":\n" << description;
    throw simu::Exception{err.str()};
}

Application::Application()
{
    SIMU_ASSERT(glfwInit(), "glfw could not be initialised properly");
    glfwSetErrorCallback(glfwErrorCallback);

    // TODO: as args
    window_ = glfwCreateWindow(640, 480, "My Title", nullptr, nullptr);
    SIMU_ASSERT(window_ != nullptr, "Window creation failed");
    glfwSetWindowUserPointer(window_, this);

    glfwMakeContextCurrent(window_);
    glbinding::initialize(glfwGetProcAddress);
    renderer_ = std::make_unique<OpenGlRenderer>();

    int w, h;
    glfwGetWindowSize(window_, &w, &h);
    renderer_->setViewport(Vec2i{0, 0}, Vec2i{w, h});

    glfwSwapInterval(1);
}

Application::~Application()
{
    glfwDestroyWindow(window_);
    glfwTerminate();
}

void Application::run()
{
    glfwSetTime(0.0);
    while (!glfwWindowShouldClose(window_))
    {
        glfwPollEvents();
    
        scene_->step(glfwGetTime());
        glfwSetTime(0.0);
    
        render();
        
        scene_ = nextScene(scene_);
    }
}

void Application::render() const
{
    if (scene_ != nullptr)
        scene_->render(*renderer_);

    glfwSwapBuffers(window_);
}

} // namespace simu
