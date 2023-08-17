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

#include <numbers>

#include "Demos.hpp"


NewtonPendulum::NewtonPendulum()
{
    registerTool<simu::Grabber>(*this);
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
    camera().panTo(simu::Vec2{0.f, 0.f});
}


void NewtonPendulum::init(simu::Renderer& renderer)
{
    renderer.setPointPrecision(4);
    renderer.setPointRadius(0.1f);
    renderer.setLineWidth(0.1f);

    simu::BodyDescriptor     descr{};
    simu::ColliderDescriptor cDescr{
        simu::Polygon{
                      simu::Vertex{-(float)n, 8.f},
                      simu::Vertex{(float)n, 8.f},
                      simu::Vertex{(float)n, 11.f},
                      simu::Vertex{-(float)n, 11.f}}
    };

    descr.dominance = 0.f;
    cDescr.material.friction.value = 0.5f;
    auto bar        = world().makeBody<simu::VisibleBody>(
        descr, simu::Rgba{0, 0, 0, 255}, &renderer
    );

    bar->addCollider(cDescr);


    simu::Vertices v{};

    // TODO: The fact that we don't have vertex-vertex contacts is very apparent here,
    int   nPoints = 48;
    float theta   = 0.f;
    for (int i = 0; i < nPoints; ++i)
    {
        theta += (2.f * std::numbers::pi_v<float>) / nPoints;
        v.emplace_back(simu::Vec2{std::cos(theta), std::sin(theta)});
    }

    descr.dominance                  = 1.f;
    cDescr.polygon                   = simu::Polygon{v.begin(), v.end()};
    cDescr.material.density          = 10.f;
    cDescr.material.bounciness.value = 1.f;
    for (int i = 0; i < n; ++i)
    {
        float x = -(n - 1.f) + 2.f * i;

        descr.position = simu::Vec2{x, 0.f};
        auto ball      = world().makeBody<simu::VisibleBody>(
            descr, simu::Rgba::filled(200), &renderer
        );
        ball->addCollider(cDescr);

        world().makeConstraint<simu::VisibleDistanceConstraint>(
            simu::Bodies{
                ball, bar
        },
            std::array<simu::Vec2, 2>{simu::Vec2{x, 1.f}, simu::Vec2{x, 8.f}},
            std::nullopt,
            7.f,
            &renderer,
            false
        );

        if (i >= 2)
            ball->setVelocity(simu::Vec2{10.f, 0.f});
    }

    world().makeForceField<simu::Gravity>(simu::Vec2{0, -10.f});
}
