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

void frameBufferResizeCallback(GLFWwindow* window, int w, int h)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2i dim{w, h};

    if (app->renderer_ != nullptr)
        app->renderer_->setViewport(Vec2i{0, 0}, dim);
}

void windowResizeCallback(GLFWwindow* window, int w, int h)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2i dim{w, h};
    // TODO: Window content scale..?

    if (app->scene() != nullptr)
    {
        app->scene()->camera().setDimensionsFromScreenCoordinates(Vec2{dim});
    }
}

void keypressCallback(GLFWwindow* window, int key, int scancode, int action, int modifiers)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    if (app->scene() != nullptr)
        app->scene()->onKeypress(Keyboard::Input::fromGlfw(key, action, modifiers)
        );
}

void mouseMoveCallback(GLFWwindow* window, double x, double y)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2i windowSize;
    glfwGetWindowSize(window, &windowSize[0], &windowSize[1]);
    Vector<double, 2> posFromBottomleft{x, windowSize[1] - y};

    Vec2i frameBufferSize;
    glfwGetFramebufferSize(window, &frameBufferSize[0], &frameBufferSize[1]);

    // Vec2 screenToPixelRatios{
    //     static_cast<float>()
    // }

    // if (app->scene() != nullptr) app->scene()->onMouseMove();
}

void mousePressCallback(GLFWwindow* window, double x, double y)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    // if (app->scene() != nullptr)
    //     app->scene()->onKeypress(Keyboard::Input::fromGlfw(key, action, modifiers)
    //     );
}

void mouseScrollCallback(GLFWwindow* window, double x, double y)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    if (app->scene() != nullptr)
        app->scene()->onMouseScroll(Vec2{(float)x, (float)y});
}


Application::Application()
{
    SIMU_ASSERT(glfwInit(), "glfw could not be initialised properly");
    glfwSetErrorCallback(glfwErrorCallback);

    // TODO: as args
    window_ = glfwCreateWindow(640, 480, "My Title", nullptr, nullptr);
    SIMU_ASSERT(window_ != nullptr, "Window creation failed");
    glfwSetWindowUserPointer(window_, this);

    glfwSetFramebufferSizeCallback(window_, frameBufferResizeCallback);
    glfwSetWindowSizeCallback(window_, windowResizeCallback);

    glfwSetKeyCallback(window_, keypressCallback);

    glfwSetScrollCallback(window_, mouseScrollCallback);


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

        float dt = glfwGetTime();
        glfwSetTime(0.0);

        if (scene_ != nullptr)
            scene_->step(dt);

        render();

        auto current = scene_;
        auto next    = nextScene(current);
        if (next != current)
        {
            scene_ = next;

            if (current != nullptr)
                current->app_ = nullptr;

            if (next != nullptr)
            {
                next->app_ = this;

                int w, h;
                glfwGetWindowSize(window_, &w, &h);
                windowResizeCallback(window_, w, h);
            }
        };
    }
}

void Application::close() { glfwSetWindowShouldClose(window_, true); }

bool Application::isKeyPressed(Keyboard::Key key) const
{
    return glfwGetKey(window_, static_cast<int>(key)) == GLFW_PRESS;
}

void Application::render() const
{
    if (scene_ != nullptr)
        scene_->render(*renderer_);

    glfwSwapBuffers(window_);
}

} // namespace simu
