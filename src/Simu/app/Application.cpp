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

#include "Simu/app/Application.hpp"

#include <sstream>

#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#include "glbinding/glbinding.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"


namespace simu
{

void styleGui()
{
    ImGui::StyleColorsDark();

    ImGuiStyle& s                 = ImGui::GetStyle();
    s.Colors[ImGuiCol_WindowBg].w = 0.4f;
}

void glfwErrorCallback(int error, const char* description);

Application::Application()
{
    bool glfwIsInit = glfwInit();
    SIMU_ASSERT(glfwIsInit, "glfw could not be initialised properly");
    glfwSetErrorCallback(glfwErrorCallback);

    // TODO: as args
    window_ = glfwCreateWindow(640, 480, "My Title", nullptr, nullptr);
    SIMU_ASSERT(window_ != nullptr, "Window creation failed");
    glfwSetWindowUserPointer(window_, this);
    setName("Simu Application");

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

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();


    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // In case of trouble: https://github.com/ocornut/imgui/issues/1542#issuecomment-907318315
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    styleGui();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window_, true);

    // TODO: Window hints, don't duplicate with Renderer...
    const char* glslVersion = "#version 330";
    ImGui_ImplOpenGL3_Init(glslVersion);
}

Application::~Application()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    renderer_.reset(nullptr);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

void Application::setName(const char* name)
{
    glfwSetWindowTitle(window_, name);
}

void Application::run()
{
    glfwSetTime(0.0);
    while (!glfwWindowShouldClose(window_))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        float dt = static_cast<float>(glfwGetTime());
        glfwSetTime(0.0);

        doGui(dt);

        if (scene_ != nullptr)
            scene_->step(dt); // draws

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            ImGui::UpdatePlatformWindows(); // may change context
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(window_);
        }

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

void Application::doGui(float dt)
{
    struct MenuData
    {
        bool exit = false;

        // for scene
        float playSpeed   = 1.f;
        bool  restart     = false;
        bool  pauseToggle = false;
        bool  step        = false;

        bool showProfiler       = true;
        bool showEngineSettings = false;
        bool showSceneControls  = false;
    };

    static World::Settings s{};
    static MenuData        menu;

    bool hasScene = scene_ != nullptr;

    // TODO: Programmatically dock windows...
    //  seems to require using DockBuilder in imgui_internal.h
    /* auto dockId = */ ImGui::DockSpaceOverViewport(
        ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode
    );

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("Application"))
        {
            ImGui::MenuItem("Exit", "Esc", &menu.exit);
            if (menu.exit)
                close();

            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Engine"))
        {
            ImGui::MenuItem("Settings", nullptr, &menu.showEngineSettings);
            ImGui::MenuItem("Profiler", nullptr, &menu.showProfiler);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Tools"))
        {
            ImGui::EndMenu();
        }

        if (hasScene && ImGui::BeginMenu("Scene"))
        {
            ImGui::MenuItem("Controls", nullptr, &menu.showSceneControls);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    if (menu.showProfiler)
    {
        ImGui::Begin("Engine Profiler", &menu.showProfiler, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f * dt, 1.f / dt);
        // TODO: ...
        ImGui::End();
    }

    if (menu.showEngineSettings)
    {
        ImGui::Begin("Engine Settings", &menu.showEngineSettings, ImGuiWindowFlags_AlwaysAutoResize);

        int vIter = s.nVelocityIterations;
        int pIter = s.nPositionIterations;

        ImGui::PushItemWidth(150);
        ImGui::SliderInt("Velocity iterations", &vIter, 0, 50);
        ImGui::SliderInt("Position iterations", &pIter, 0, 50);

        s.nVelocityIterations = vIter;
        s.nPositionIterations = pIter;

        if (hasScene)
            scene_->world().updateSettings(s);

        ImGui::End();
    }

    if (hasScene && menu.showSceneControls)
    {
        ImGui::Begin("Scene Controls", &menu.showSceneControls, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("Play speed %f", scene_->playSpeed());
        ImGui::SameLine();
        if (ImGui::Button("-"))
            scene_->setPlaySpeed(scene_->playSpeed() / 2.f);
        ImGui::SameLine();
        if (ImGui::Button("+"))
            scene_->setPlaySpeed(scene_->playSpeed() * 2.f);

        if (ImGui::Button(menu.pauseToggle ? "Play (P)" : "Pause (P)"))
        {
            menu.pauseToggle = !menu.pauseToggle;
            if (menu.pauseToggle)
                scene_->pause();
            else
                scene_->resume();
        }

        if (ImGui::Button("Restart (R)"))
            scene_->reset();

        // TODO:
        // if (menu.pauseToggle)
        //     if (ImGui::Button("Step (S)"))
        //         scene_->step();

        ImGui::End();
    }
}


void Application::show() const { glfwSwapBuffers(window_); }


void glfwErrorCallback(int error, const char* description)
{
    std::stringstream err;
    err << "GLFW error code " << error << ":\n" << description;
    throw simu::Exception{err.str()};
}

void Application::frameBufferResizeCallback(GLFWwindow* window, int w, int h)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2i dim{w, h};

    if (app->renderer_ != nullptr)
        app->renderer_->setViewport(Vec2i{0, 0}, dim);
}

void Application::windowResizeCallback(GLFWwindow* window, int w, int h)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

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
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    if (app->scene() != nullptr && !ImGui::GetIO().WantCaptureKeyboard)
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
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    Vec2 windowPos{(float)x, (float)y};

    if (app->scene() != nullptr)
        app->scene()->mouseMove(
            windowToScene(window, app->scene()->camera()) * windowPos
        );
}

void Application::mousePressCallback(GLFWwindow* window, int button, int action, int mods)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    double x, y;
    glfwGetCursorPos(window, &x, &y);
    Vec2 windowPos{(float)x, (float)y};

    if (app->scene() != nullptr && !ImGui::GetIO().WantCaptureMouse)
        app->scene()->mousePress(Mouse::Input::fromGlfw(
            windowToScene(window, app->scene()->camera()) * windowPos, button, action, mods
        ));
}

void Application::mouseScrollCallback(GLFWwindow* window, double x, double y)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

    if (app->scene() != nullptr && !ImGui::GetIO().WantCaptureMouse)
        app->scene()->mouseScroll(Vec2{(float)x, (float)y});
}

} // namespace simu
