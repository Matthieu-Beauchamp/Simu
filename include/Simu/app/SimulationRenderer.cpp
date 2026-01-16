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

#include "SimulationRenderer.hpp"

namespace simu
{

void SimulationRenderer::draw(const Simulation& simulation, Renderer& renderer) {
    clear_screen(renderer);

    for (const auto& object : simulation.dynamic_objects()) {
        draw_dynamic_object(simulation, object, renderer);
    }

    for (const auto& object : simulation.static_objects()) {
        draw_static_object(simulation, object, renderer);
    }

    for (const auto& constraint : simulation.contact_constraints()) {
        draw_contact_constraint(constraint.second, renderer);
    }
}

void SimulationRenderer::clear_screen(Renderer& renderer) const {
    renderer.fillScreen(settings.background);
}

void SimulationRenderer::draw_static_object(
    const Simulation&          simulation,
    const StaticPhysicsObject& object,
    Renderer&                  renderer
) {
    draw_collider(
        simulation, object.collider_id, object.position, settings.static_object, renderer
    );
}

void SimulationRenderer::draw_dynamic_object(
    const Simulation&           simulation,
    const DynamicPhysicsObject& object,
    Renderer&                   renderer
) {
    draw_collider(
        simulation, object.collider_id, object.position, settings.dynamic_object, renderer
    );
}

void SimulationRenderer::draw_contact_constraint(
    const ContactConstraint2& constraint,
    Renderer&                 renderer
) const {
    float normalLength = renderer.getLineWidth() * 10.f;

    renderer.setPointRadius(0.1f);
    renderer.setPointPrecision(4);
    for (std::uint32_t i = 0; i < constraint.contacts.n_contacts; ++i) {
        renderer.drawPoint(constraint.contacts.contacts_a[i], settings.contact_points);

        renderer.drawLine(
            constraint.contacts.contacts_a[i],
            constraint.contacts.contacts_a[i] + constraint.contacts.normal * normalLength,
            settings.contact_normal
        );

        renderer.drawPoint(constraint.contacts.contacts_b[i], settings.contact_points);
    }
}

void SimulationRenderer::draw_collider(
    const Simulation& simulation,
    ObjectId          id,
    Position          position,
    Rgba              color,
    Renderer&         renderer
) const {
    switch (simulation.collider_type(id)) {
        case ColliderType::Circle:
        {
            Circle circle = position.toWorldSpace() * simulation.get_circle(id);
            renderer.setPointRadius(circle.radius());
            renderer.setPointPrecision(settings.circle_segments);
            renderer.drawPoint(circle.center(), color);
            return;
        }
        case ColliderType::Capsule:
        {
            Capsule capsule = position.toWorldSpace() * simulation.get_capsule(id);
            renderer.setPointRadius(capsule.radius());
            renderer.setPointPrecision(settings.circle_segments);

            renderer.drawPoint(capsule.bottom_center(), color);
            renderer.drawPoint(capsule.top_center(), color);

            Vec2 axis      = normalized(capsule.top() - capsule.bottom());
            Vec2 perp_axis = perp(axis);

            Vec2 center = (capsule.bottom() + capsule.top()) / 2;

            std::array<Vec2, 4> box = {
                capsule.bottom_center() + perp_axis * capsule.radius(),
                capsule.top_center() + perp_axis * capsule.radius(),
                capsule.top_center() - perp_axis * capsule.radius(),
                capsule.bottom_center() - perp_axis * capsule.radius(),
            };

            renderer.drawPolygon(
                center, makeView(box.data(), box.data() + box.size()), color
            );
            return;
        }
        case ColliderType::Polygon:
        {
            Polygon polygon = position.toWorldSpace() * simulation.get_polygon(id);
            const Vec2* first_vertex = std::addressof(*polygon.begin());
            renderer.drawPolygon(
                position.position(),
                makeView(first_vertex, first_vertex + polygon.size()),
                color
            );
            return;
        }
    }

    UNREACHABLE;
}

} // namespace simu
