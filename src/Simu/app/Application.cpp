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
#include "imgui_internal.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

// ImGui docking API subject to change, operations are isolated here.
// Only valid during a single frame, use scope to properly complete operations
class DockSpaces
{
public:

    DockSpaces()
    {
        // https://github.com/ocornut/imgui/issues/2109#issuecomment-426204357
        viewPortDock_ = ImGui::DockSpaceOverViewport(
            ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode
        );

        if (!isFirstFrame)
            return;

        ImGui::DockBuilderRemoveNode(viewPortDock_); // Clear out existing layout
        ImGui::DockBuilderAddNode(viewPortDock_, ImGuiDockNodeFlags_DockSpace); // Add empty node
        ImGui::DockBuilderSetNodeSize(
            viewPortDock_, ImGui::GetMainViewport()->WorkSize
        ); // needed when splitting right after

        leftDock_ = ImGui::DockBuilderSplitNode(
            viewPortDock_, ImGuiDir_Left, 0.2f, nullptr, nullptr
        );
        // ImGuiID dock_id_bottom = ImGui::DockBuilderSplitNode(
        //     dock_main_id, ImGuiDir_Down, 0.20f, NULL, &dock_main_id
        // );
    }

    ~DockSpaces()
    {
        if (!isFirstFrame)
            return;

        auto getSize = [](const char* windowName) {
            // See imgui.cpp: CalcWindowContentSizes
            auto   w = ImGui::FindWindowByName(windowName);
            ImVec2 dim;
            dim.x = w->DC.CursorMaxPos.x - w->DC.CursorStartPos.x;
            dim.y = w->DC.CursorMaxPos.y - w->DC.CursorStartPos.y;
            return dim;
        };

        simu::Vec2 dim{};
        for (auto w : windows_)
        {
            ImVec2 s = getSize(w);
            dim[0]   = std::max(dim[0], s.x);
            dim[1] += s.y;
        }


        ImVec2 maxSize    = ImGui::GetMainViewport()->WorkSize;
        float  totalRatio = dim[1] / maxSize[1];
        // maxSize.x *= 0.4f;
        dim = std::min(dim, simu::Vec2{maxSize.x, maxSize.y});

        ImGui::DockBuilderSetNodeSize(leftDock_, ImVec2{dim[0], dim[1]});

        for (auto it = windows_.begin(); it != windows_.end(); ++it)
        {
            auto w = ImGui::FindWindowByName(*it);

            float ratio = getSize(*it).y * totalRatio / dim[1];
            dim[1] -= dim[1] * ratio;

            ImGuiID dock;
            ImGui::DockBuilderSplitNode(
                leftDock_, ImGuiDir_Up, ratio, &dock, &leftDock_
            );

            ImGui::DockBuilderDockWindow(w->Name, dock);
        }

        ImGui::DockBuilderFinish(viewPortDock_);
        windows_.clear();
        isFirstFrame = false;
    }

    void dockCurrentWindowLeft()
    {
        if (!isFirstFrame)
            return;

        windows_.emplace_back(ImGui::GetCurrentWindow()->Name);
    }

private:

    static bool              isFirstFrame;
    ImGuiID                  viewPortDock_;
    ImGuiID                  leftDock_;
    std::vector<const char*> windows_;
};
bool DockSpaces::isFirstFrame = true;

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
    }
}

void Application::close() { glfwSetWindowShouldClose(window_, true); }

bool Application::isKeyPressed(Keyboard::Key key) const
{
    return glfwGetKey(window_, static_cast<int>(key)) == GLFW_PRESS;
}

void Application::changeScene(std::shared_ptr<Scene> next)
{
    auto current = scene_;
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

void Application::doGui(float dt)
{
    struct MenuData
    {
        bool exit        = false;
        bool selectScene = true;

        // for scene
        float playSpeed   = 1.f;
        bool  restart     = false;
        bool  pauseToggle = false;
        bool  step        = false;

        bool showProfiler       = true;
        bool showEngineSettings = true;
        bool showSceneControls  = true;
    };

    static World::Settings s{};
    static MenuData        menu;

    DockSpaces docks{};
    bool       hasScene = scene_ != nullptr;


    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("Application"))
        {
            ImGui::MenuItem("Exit", "Esc", &menu.exit);
            if (menu.exit)
                close();

            ImGui::MenuItem("Select Scene", nullptr, &menu.selectScene);

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

    if (menu.selectScene)
    {
        ImGui::Begin(
            "Scene selection", &menu.selectScene, ImGuiWindowFlags_AlwaysAutoResize
        );

        docks.dockCurrentWindowLeft();

        for (const auto& scene : scenes_)
        {
            bool selected = scene.second == scene_;
            ImGui::Selectable(scene.first, &selected);
            if (selected && scene.second != scene_)
            {
                changeScene(scene.second);
                break;
            }
        }

        ImGui::End();
    }

    if (menu.showProfiler)
    {
        ImGui::Begin(
            "Engine Profiler", &menu.showProfiler, ImGuiWindowFlags_AlwaysAutoResize
        );

        docks.dockCurrentWindowLeft();

        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f * dt, 1.f / dt);

        if (hasScene)
        {
            auto printTimeEntry = [](const TimeEntry& t, const char* name) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text(name);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", t.last() * 1000.f);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", t.average() * 1000.f);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", t.max() * 1000.f);
            };

            Profiler& p = scene_->world().profiler();
            if (ImGui::BeginTable("Timers (ms)", 4))
            {
                ImGui::TableSetupColumn("Operation");
                ImGui::TableSetupColumn("last");
                ImGui::TableSetupColumn("average");
                ImGui::TableSetupColumn("max");

                ImGui::TableHeadersRow();

                printTimeEntry(p.islandConstruction, "Island construction");
                printTimeEntry(p.solveVelocities, "Solve velocities");
                printTimeEntry(p.solvePositions, "Solve Positions");
                printTimeEntry(p.treeUpdateAndCollision, "Tree update and collision");
                printTimeEntry(p.narrowPhaseCollision, "Narrow phase collision");

                ImGui::EndTable();
            }
            ImGui::Text("Tree height: %u", p.treeHeight);


            ImGui::Separator();
            if (ImGui::Button("Reset profiler"))
                p.reset();
        }


        ImGui::End();
    }

    if (menu.showEngineSettings)
    {
        ImGui::Begin(
            "Engine Settings", &menu.showEngineSettings, ImGuiWindowFlags_AlwaysAutoResize
        );

        docks.dockCurrentWindowLeft();

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
        ImGui::Begin(
            "Scene Controls", &menu.showSceneControls, ImGuiWindowFlags_AlwaysAutoResize
        );

        docks.dockCurrentWindowLeft();


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
