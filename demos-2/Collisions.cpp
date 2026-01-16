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

using namespace simu;

Collisions::Collisions() {
    registerAllTools();
    useTool<simu::Grabber>();

    camera().setPixelSize(1.f / 10.f);
}

void Collisions::init(simu::Renderer& renderer) {
    simu::ObjectBuilder builder = simu::ObjectBuilder();

    builder.set_position(Vec2(-10.f, 2.f));
    builder.set_collider(Circle(Vec2(), 2.f));
    simu().create_object(builder);

    builder.set_position(Vec2(0.f, 2.f));
    builder.set_collider(Polygon::box(Vec2(2.f, 2.f)));
    simu().create_object(builder);

    builder.set_position(Vec2(10.f, 2.f));
    builder.set_collider(Capsule(Vec2(-1.f, 0.f), Vec2(1.f, 0.f), 1.f));
    simu().create_object(builder);

    builder.set_position(Vec2(10.f, 8.f), 0.f);
    builder.set_collider(Polygon::box(Vec2(2.f, 2.f)));
    simu().create_object(builder);

    builder.set_static();
    builder.set_position(simu::Vec2(0.f, -1.f));
    builder.set_collider(
        simu::Polygon::box(simu::Vec2{100.f, 2.f})
    );

    simu().create_object(builder);
}

void Collisions::doGui() {
}
