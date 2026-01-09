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

#pragma once

#include "Simu/app/Renderer.hpp"

#include "Simu/physics-2.0/Settings.hpp"
#include "Simu/physics-2.0/Simulation.hpp"

namespace simu
{

struct SimulationRendererSettings
{
    Rgba background = Rgba(172, 216, 227, 255);

    Rgba static_object  = Rgba(46, 46, 46, 255);
    Rgba dynamic_object = Rgba(150, 99, 153, 255);

    Rgba contact_points = Rgba(255, 255, 255, 255);
    Rgba contact_normal = Rgba(255, 255, 255, 255);

    std::uint32_t circle_segments = 64;
};

class SimulationRenderer
{
public:

    explicit SimulationRenderer(SimulationRendererSettings settings = {})
        : settings{settings} {}

    void draw(const Simulation& simulation, Renderer& renderer);

    void clear_screen(Renderer& renderer) const;

    void
    draw_static_object(const Simulation& simulation, const StaticPhysicsObject& object, Renderer& renderer);
    void draw_dynamic_object(
        const Simulation&           simulation,
        const DynamicPhysicsObject& object,
        Renderer&                   renderer
    );

    void
    draw_contact_constraint(const ContactConstraint2& constraint, Renderer& renderer) const;

    void draw_collider(
        const Simulation& simulation,
        ObjectId          id,
        Position          position,
        Rgba              color,
        Renderer&         renderer
    ) const;

    SimulationRendererSettings settings;
};

} // namespace simu
