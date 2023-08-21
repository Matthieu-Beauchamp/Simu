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


#include "Demos.hpp"

#include "imgui.h"

Motorcycle::Motorcycle()
{
    registerAllTools();
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
}

void Motorcycle::init(simu::Renderer& renderer)
{
    renderer.setPointPrecision(4);
    renderer.setPointRadius(0.1f);
    renderer.setLineWidth(0.1f);

    buildMap();
    buildMotorcycle();
}

void Motorcycle::moveCamera(float) { camera().panTo(motorcycle_->centroid()); }

void Motorcycle::updateMotors()
{
    using simu::Keyboard;
    typedef simu::Vector<float, 1> MotorDir;

    float tiltDir = 0.f;
    if (app()->isKeyPressed(Keyboard::Key::left))
        tiltDir += 1.f;
    if (app()->isKeyPressed(Keyboard::Key::right))
        tiltDir += -1.f;

    riderTilt_->direction(MotorDir{tiltDir});

    if (app()->isKeyPressed(Keyboard::Key::up))
        motor_->direction(MotorDir{-1.f});
    else
        motor_->direction(MotorDir{0.f});

    float vel = rearWheel_->angularVelocity();
    if (app()->isKeyPressed(Keyboard::Key::down))
        breaks_->direction(MotorDir{vel > 0.f ? -1.f : 1.f});
    else
        breaks_->direction(MotorDir{0.f});
}

void Motorcycle::buildMap()
{
    simu::World&    w = world();
    simu::Renderer* r = app()->renderer();

    w.makeForceField<simu::Gravity>(simu::Vec2{0.f, -10.f});

    simu::BodyDescriptor descr{};
    descr.dominance = 0.f;

    simu::ColliderDescriptor floor{
        simu::Polygon::box(simu::Vec2{200.f, 20.f}, simu::Vec2{0.f, -30.f})};

    w.makeBody<simu::VisibleBody>(descr, bodyColor, r)->addCollider(floor);
}

void Motorcycle::buildMotorcycle()
{
    simu::World&    w = world();
    simu::Renderer* r = app()->renderer();

    simu::BodyDescriptor descr{};

    simu::Material chassisMaterial{};
    chassisMaterial.density = 5.f;
    simu::ColliderDescriptor chassisColliderDescr{
        simu::Polygon::box(simu::Vec2{6.f * wheelRadius, 2 * wheelRadius}),
        chassisMaterial};

    motorcycle_ = w.makeBody<simu::VisibleBody>(descr, chassisColor, r);
    motorcycle_->addCollider(chassisColliderDescr);

    descr.position = simu::Vec2{-2.f * wheelRadius, -2.f * wheelRadius};

    simu::Material wheelMaterial{};
    wheelMaterial.friction.value = 2.f;
    simu::ColliderDescriptor wheelColliderDescr{
        simu::Polygon::circle(wheelRadius, wheelPrecision), wheelMaterial};

    rearWheel_ = w.makeBody<simu::VisibleBody>(descr, wheelColor, r);
    rearWheel_->addCollider(wheelColliderDescr);


    descr.position[0] = 2.f * wheelRadius;
    frontWheel_       = w.makeBody<simu::VisibleBody>(descr, wheelColor, r);
    frontWheel_->addCollider(wheelColliderDescr);

    simu::Vec2 springDir{0.f, -1.f};
    float      springLength = wheelRadius * 1.25f;

    // w.makeConstraint<simu::HingeConstraint>(
    //     simu::Bodies{motorcycle_, rearWheel_}, rearWheel_->centroid()
    // );
    // w.makeConstraint<simu::HingeConstraint>(
    //     simu::Bodies{motorcycle_, frontWheel_}, frontWheel_->centroid()
    // );

    auto rearSusp = w.makeConstraint<simu::SuspensionConstraint>(
        simu::Bodies{motorcycle_, rearWheel_},
        springDir,
        springLength,
        simu::Vec2{-2.f * wheelRadius, -wheelRadius}
    );
    auto frontSusp = w.makeConstraint<simu::SuspensionConstraint>(
        simu::Bodies{motorcycle_, frontWheel_},
        springDir,
        springLength,
        simu::Vec2{2.f * wheelRadius, -wheelRadius}
    );

    rearSusp->setSpringDamping(options_.suspensionDamping);
    frontSusp->setSpringDamping(options_.suspensionDamping);

    rearSusp->setSpringRestitution(options_.suspensionRestitution);
    frontSusp->setSpringRestitution(options_.suspensionRestitution);


    auto specs = simu::RotationMotor::Specs::fromTorque(
        options_.maxVelocity, options_.maxTorque
    );
    auto breakSpecs = simu::RotationMotor::Specs::fromAccel(
        0.f, breakStrengthRatio * options_.maxTorque
    );

    motor_ = w.makeConstraint<simu::RotationMotor>(
        simu::Bodies::singleBody(rearWheel_), specs
    );
    breaks_ = w.makeConstraint<simu::RotationMotor>(
        simu::Bodies::singleBody(frontWheel_), breakSpecs
    );


    float chestHeight = 3.f * wheelRadius;
    descr.position    = simu::Vec2{0.f, wheelRadius + chestHeight / 2.f};
    simu::ColliderDescriptor chestDescr{
        simu::Polygon::box(simu::Vec2{0.3f * wheelRadius, chestHeight})};

    simu::ColliderDescriptor headDescr{simu::Polygon::circle(
        0.5f * wheelRadius,
        wheelPrecision,
        simu::Vec2{0.f, chestHeight / 2.f + 0.25f * wheelRadius}
    )};

    rider_ = w.makeBody<simu::VisibleBody>(descr, bodyColor, r);
    rider_->addCollider(chestDescr);
    rider_->addCollider(headDescr);

    simu::Vec2 shoulder = simu::Vec2{0.f, descr.position[1] + 0.4f * chestHeight};

    auto arm = w.makeConstraint<simu::VisibleDistanceConstraint>(
        simu::Bodies{
            motorcycle_, rider_
    },
        std::array<simu::Vec2, 2>{
            simu::Vec2{2.f * wheelRadius, 2.f * wheelRadius}, shoulder},
        0.4f * chestHeight,
        0.8f * chestHeight,
        r,
        false
    );

    simu::Vec2 hip = descr.position - simu::Vec2{0.f, 0.5f * chestHeight};

    auto leg = w.makeConstraint<simu::VisibleDistanceConstraint>(
        simu::Bodies{
            motorcycle_, rider_
    },
        std::array<simu::Vec2, 2>{simu::Vec2{0.f, -wheelRadius}, hip},
        std::nullopt,
        2.4f * wheelRadius,
        r,
        false
    );

    riderTilt_ = w.makeConstraint<simu::RotationMotor>(
        simu::Bodies::singleBody(rider_),
        simu::RotationMotor::Specs::fromTorque(
            options_.maxTiltVelocity, options_.maxTiltTorque
        )
    );
}

void Motorcycle::doGui()
{
    ImGui::SliderFloat(
        "Max motor velocity (radians)", &options_.maxVelocity, 0.1f, 1000.f
    );
    ImGui::SliderFloat("Max motor torque", &options_.maxTorque, 0.1f, 10000.f);

    ImGui::SliderFloat(
        "Max tilt velocity (radians)", &options_.maxTiltVelocity, 0.1f, 100.f
    );
    ImGui::SliderFloat("Max tilt torque", &options_.maxTiltTorque, 0.1f, 1000.f);

    ImGui::SliderFloat("Suspension damping", &options_.suspensionDamping, 0.f, 5.f);
    ImGui::SliderFloat(
        "Suspension restitution", &options_.suspensionRestitution, 0.f, 2.f
    );
}
