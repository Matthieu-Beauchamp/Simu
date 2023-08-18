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
#include "imgui.h"

// TODO: Has a bug when there are two contacts are on the same circle
// Say we have the circles A, B and C. B and C are at rest and A is moving towards B.
//  if A collides with B when B is also colliding with C, then A is pushed back
//  instead of coming to rest ater transfering its momentum to B.
//
// Disabling warmstarting has no effect.
// Adding space between the circles helps avoid the issue.
//
// https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=1584
//
// This link is partially broken, I had to change page manually in the URL.
// https://www.gamedev.net/forums/topic.asp?topic_id=291687&page=1
//
// After some debugging and research, this not a bug but a limitation of the engine.
// The Collision restitution model used is only valid for collision between 2 bodies.
// Some methods can be used to order the collisions sequentially an process only
//  collisions of 2 bodies at a time.
// See for example, Jan Bender's paper "Constraint-based collision and contact handling using impulses".
// For our engine, we can work around the issue by putting a gap between the circles,
//  this decreases the chances that an n-body collision happens.
//
// This also explains why stacking bouncy bodies is very unstable.


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

    float barWidth  = n_ * circleRadius_ * 2.f;
    float barBottom = circleRadius_ + stringLength_;

    simu::BodyDescriptor     descr{};
    simu::ColliderDescriptor cDescr{simu::Polygon::box(
        simu::Vec2{barWidth, 1.f}, simu::Vec2{0.f, barBottom + 0.5f}
    )};

    descr.dominance                = 0.f;
    cDescr.material.friction.value = 0.5f;
    auto bar                       = world().makeBody<simu::VisibleBody>(
        descr, simu::Rgba{0, 0, 0, 255}, &renderer
    );

    bar->addCollider(cDescr);


    simu::Vertices v{};

    float theta = 0.f;
    for (int i = 0; i < circlePrecision_; ++i)
    {
        theta += (2.f * std::numbers::pi_v<float>) / circlePrecision_;
        v.emplace_back(circleRadius_ * simu::Vec2{std::cos(theta), std::sin(theta)});
    }

    descr.dominance                  = 1.f;
    cDescr.polygon                   = simu::Polygon{v.begin(), v.end()};
    cDescr.material.density          = 10.f;
    cDescr.material.bounciness.value = 1.f;
    for (int i = 0; i < n_; ++i)
    {
        float gapRatio = 0.1f;
        float offset   = ((2 + gapRatio) * i + 1) * circleRadius_;
        float x        = -barWidth / 2.f + offset;

        descr.position = simu::Vec2{x, 0.f};
        auto ball      = world().makeBody<simu::VisibleBody>(
            descr, simu::Rgba::filled(200), &renderer
        );
        ball->addCollider(cDescr);

        world().makeConstraint<simu::VisibleDistanceConstraint>(
            simu::Bodies{
                ball, bar
        },
            std::array<simu::Vec2, 2>{
                simu::Vec2{x, circleRadius_}, simu::Vec2{x, barBottom}},
            std::nullopt,
            stringLength_,
            &renderer,
            false
        );

        if (i >= (n_ - initialPush_))
            ball->setVelocity(simu::Vec2{10.f, 0.f});
    }

    world().makeForceField<simu::Gravity>(simu::Vec2{0, -10.f});
}

void NewtonPendulum::doGui()
{
    ImGui::SliderInt("Number of circles", &n_, 1, 10);
    ImGui::SliderFloat("Radius", &circleRadius_, 0.1f, 10.f);
    ImGui::SliderFloat("String length", &stringLength_, 0.01f, 20.f);
    ImGui::SliderInt("Circle precision", &circlePrecision_, 3, 100);

    ImGui::SliderInt("Push how many?", &initialPush_, 0, n_);
}
