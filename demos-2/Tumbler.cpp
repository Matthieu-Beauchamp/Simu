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
#include "Simu/math/Random.hpp"

#include <random>

using namespace simu;

constexpr int   tumbler_sides     = 6;
constexpr float tumbler_radius    = 100.f;
constexpr float tumbler_thickness = 5.f;
constexpr float rotation_speed    = 2 * std::numbers::pi_v<float> / 50.f;


constexpr std::int64_t seed = 1234;

std::mt19937_64 engine{1234};


Tumbler::Tumbler() {
    registerAllTools();
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
    camera().setZoom(camera().zoom() / 2);
}

Vec2 rand_vec2(float min_length, float max_length) {
    float len = rand_float(min_length, max_length, engine);
    float x   = rand_float(-1.f, 1.f, engine);
    float y   = rand_float(-1.f, 1.f, engine);

    Vec2  v    = Vec2(x, y);
    float norm = simu::norm(v);
    if (norm > 0.f) {
        v /= norm;
    }

    return v * len;
}

ObjectBuilder random_object() {
    ObjectBuilder builder = ObjectBuilder();

    builder.set_density(rand_float(0.01f, 10.f, engine));

    switch (rand_int(1, 3, engine)) {
        case 1:
        {
            float radius       = rand_float(0.5f, 4.f, engine);
            auto  num_vertices = rand_int(3, Polygon::max_vertices, engine);
            std::array<Vec2, Polygon::max_vertices> vertices{};
            for (int i = 0; i < num_vertices; ++i) {
                float min = i * 2 * std::numbers::pi_v<float> / num_vertices;
                float max = (i + 1) * 2 * std::numbers::pi_v<float> / num_vertices;
                float theta = rand_float(min, max, engine);
                vertices[i] = Vec2(std::cos(theta), std::sin(theta)) * radius;
            }

            return builder.set_collider(Polygon(
                std::ranges::subrange(vertices.begin(), vertices.begin() + num_vertices)
            ));
        }
        case 2:
        {
            float theta = rand_float(0.f, 2 * std::numbers::pi_v<float>, engine);
            float len    = rand_float(0.5f, 4.f, engine);
            float radius = std::max(len, rand_float(0.5f, 4.f, engine));
            Vec2  top    = Vec2(std::cos(theta), std::sin(theta));
            top *= len;
            Vec2 bottom = -top;
            return builder.set_collider(Capsule(bottom, top, radius));
        }
        case 3:
        {
            return builder.set_collider(Circle(Vec2(), rand_float(0.5f, 4.f, engine)));
        }
    }

    UNREACHABLE;
}

void create_tumbler_outline(Simulation& simu) {
    float theta = 2.f * std::numbers::pi_v<float> / tumbler_sides;
    std::array<Vec2, tumbler_sides> vertices{};

    for (int i = 0; i < tumbler_sides; ++i) {
        vertices[i] = Vec2(std::cos(theta * i), std::sin(theta * i)) * tumbler_radius;
    }

    ObjectBuilder builder{};
    builder.set_static();
    for (int i = 0; i < tumbler_sides; ++i) {
        Capsule capsule = Capsule(
            vertices[i], vertices[(i + 1) % tumbler_sides], tumbler_thickness
        );
        Vec2      center    = (capsule.bottom() + capsule.top()) / 2.f;
        Transform to_origin = Transform(Rotation(0.f), Translation(-center));

        builder.set_position(center);
        builder.set_collider(to_origin * capsule);
        simu.create_object(builder);
    }
}


void Tumbler::init(simu::Renderer& renderer) {
    pause();

    count_ = 0;
    create_tumbler_outline(simu());
}

void Tumbler::preStep(float dt) {
    if (isPaused()) {
        return;
    }

    auto rot = Rotation(rotation_speed * dt);
    for (auto& obj : simu().static_objects()) {
        obj.position = Position(
            rot * obj.position.position(), obj.position.orientation() + rot.theta()
        );
    }

    if (count_ < maxCount_) {
        ObjectBuilder builder = random_object();
        builder.set_position(
            rand_vec2(0.f, tumbler_radius / 2),
            rand_float(0.f, 2 * std::numbers::pi_v<float>, engine)
        );
        builder.set_velocity(rand_vec2(0.f, 25.f));

        simu().create_object(builder);
        ++count_;
    }
}


void Tumbler::doGui() { ImGui::SliderInt("Max count", &maxCount_, 1, 2500); }
