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

#define GLFW_INCLUDE_NONE
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

void Application::frameBufferResizeCallback(GLFWwindow* window, int w, int h)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2i dim{w, h};

    if (app->renderer_ != nullptr)
        app->renderer_->setViewport(Vec2i{0, 0}, dim);
}

void Application::windowResizeCallback(GLFWwindow* window, int w, int h)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2i dim{w, h};
    // TODO: Window content scale..?

    if (app->scene() != nullptr)
    {
        app->scene()->camera().setDimensionsFromScreenCoordinates(
            static_cast<Vec2>(dim)
        );
    }
}

void Application::keypressCallback(
    GLFWwindow* window,
    int         key,
    int /* scancode */,
    int action,
    int modifiers
)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    if (app->scene() != nullptr)
        app->scene()->keypress(Keyboard::Input::fromGlfw(key, action, modifiers));
}

Mat3 windowToScene(GLFWwindow* window, const Camera& camera)
{
    Vec2i windowSize;
    glfwGetWindowSize(window, &windowSize[0], &windowSize[1]);
    Vec2 wz{windowSize};

    Vec2i fz;
    glfwGetFramebufferSize(window, &fz[0], &fz[1]);

    // clang-format off
    Mat3 flipY {
        1.f,  0.f, 0.f,
        0.f, -1.f, wz[1],
        0.f,  0.f, 1.f
    };

    Mat3 screenToNdc {
        2.f / wz[0], 0.f,         -1.f,
        0.f,         2.f / wz[1], -1.f,
        0.f,         0.f,          1.f
    };
    // clang-format on

    return camera.invTransform() * screenToNdc * flipY;
}

void Application::mouseMoveCallback(GLFWwindow* window, double x, double y)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2 windowPos{(float)x, (float)y};

    if (app->scene() != nullptr)
        app->scene()->mouseMove(
            windowToScene(window, app->scene()->camera()) * windowPos
        );
}

void Application::mousePressCallback(
    GLFWwindow* window,
    int         button,
    int         action,
    int         mods
)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    double x, y;
    glfwGetCursorPos(window, &x, &y);
    Vec2 windowPos{(float)x, (float)y};

    if (app->scene() != nullptr)
        app->scene()->mousePress(Mouse::Input::fromGlfw(
            windowToScene(window, app->scene()->camera()) * windowPos,
            button,
            action,
            mods
        ));
}

void Application::mouseScrollCallback(GLFWwindow* window, double x, double y)
{
    Application* app
        = static_cast<Application*>(glfwGetWindowUserPointer(window));

    if (app->scene() != nullptr)
        app->scene()->mouseScroll(Vec2{(float)x, (float)y});
}


Application::Application()
{
    bool glfwIsInit = glfwInit();
    SIMU_ASSERT(glfwIsInit, "glfw could not be initialised properly");
    glfwSetErrorCallback(glfwErrorCallback);

    // TODO: as args
    window_ = glfwCreateWindow(640, 480, "My Title", nullptr, nullptr);
    SIMU_ASSERT(window_ != nullptr, "Window creation failed");
    glfwSetWindowUserPointer(window_, this);

    glfwSetFramebufferSizeCallback(window_, frameBufferResizeCallback);
    glfwSetWindowSizeCallback(window_, windowResizeCallback);

    glfwSetKeyCallback(window_, keypressCallback);

    glfwSetCursorPosCallback(window_, mouseMoveCallback);
    glfwSetMouseButtonCallback(window_, mousePressCallback);
    glfwSetScrollCallback(window_, mouseScrollCallback);


    glfwMakeContextCurrent(window_);
    glbinding::initialize(glfwGetProcAddress);
    renderer_ = std::make_unique<OpenGlRenderer>();

    int w, h;
    glfwGetWindowSize(window_, &w, &h);
    renderer_->setViewport(Vec2i{0, 0}, Vec2i{w, h});

    // glfwSwapInterval(1);
}

Application::~Application()
{
    renderer_.reset(nullptr);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

void Application::run()
{
    glfwSetTime(0.0);
    while (!glfwWindowShouldClose(window_))
    {
        glfwPollEvents();

        float dt = static_cast<float>(glfwGetTime());
        glfwSetTime(0.0);

        std::stringstream ss{};
        ss << "Elapsed: " << dt * 1000.f << " ms";
        if (scene_ != nullptr)
        {
            ss << (scene_->isPaused() ? " | Paused | " : " | ");
            ss << "x" << scene_->playSpeed();
        }

        glfwSetWindowTitle(window_, ss.str().c_str());

        if (scene_ != nullptr)
            scene_->step(dt);

        show();

        auto current = scene_;
        auto next    = nextScene(current);
        if (next != current)
        {
            scene_ = next;

            if (current != nullptr)
            {
                current->app_      = nullptr;
                current->renderer_ = nullptr;
            }

            if (next != nullptr)
            {
                if (!next->isInit() || next->app_ != this)
                    next->init(this);

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

void Application::show() const { glfwSwapBuffers(window_); }

} // namespace simu
