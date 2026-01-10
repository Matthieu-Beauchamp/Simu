////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
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

#include "Simu/physics-2.0/Simulation.hpp"

using namespace simu;

static constexpr Vec2 dims = Vec2(2.f, 2.f);

static constexpr int n_stacks = 100;
static constexpr int height   = 100;

ObjectId make_box(simu::Simulation& simu, simu::Vec2 pos) {
    simu::ObjectBuilder descr = simu::ObjectBuilder();
    descr.set_position(pos);
    descr.set_collider(simu::Polygon::box(dims));

    // TODO:
    // descr.set_material(Material{density, friction, bounciness});

    return simu.create_object(descr);
}

int main() {
    Simulation sim;
    float      spacing     = 0.5f;
    float      floorWidth  = (n_stacks * 2 + 4) * dims[0];
    float      floorHeight = 20.f;
    simu::Vec2 center{0.f, -floorHeight / 2.f - 1.f};

    for (int stack = 0; stack < n_stacks; ++stack) {
        for (int h = 0; h < height; ++h) {
            float x = -floorWidth / 2.f + dims[0] * (1 + (stack + 1) * 2);
            float y = h * (dims[1] + spacing) + spacing;
            make_box(sim, simu::Vec2{x, y});
        }
    }

    simu::ObjectBuilder builder = simu::ObjectBuilder();
    builder.set_static();
    builder.set_position(center);
    builder.set_collider(
        simu::Polygon::box(simu::Vec2{floorWidth + 2.f * height * dims[1], floorHeight})
    );
    sim.create_object(builder);


    while (true) {
        sim.step();
    }
}
