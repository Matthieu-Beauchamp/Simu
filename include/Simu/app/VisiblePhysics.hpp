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

#include "Simu/physics.hpp"

#include "Simu/app/Renderer.hpp"

#include "Simu/app/MouseConstraint.hpp"

namespace simu
{

// TODO: Store pointer to scene instead of renderer, in case scene changes application.
class Visible
{
public:

    Visible(Renderer* renderer) : renderer_{renderer} {}
    virtual ~Visible() = default;

    void draw()
    {
        if (renderer_ != nullptr)
            draw(*renderer_);
    }

protected:

    virtual void draw(Renderer& renderer) = 0;

private:

    Renderer* renderer_;
};


class VisibleBody : public Body, public Visible
{
public:

    VisibleBody(BodyDescriptor descr, Rgba color, Renderer* renderer)
        : Body{descr}, Visible{renderer}, color_{color}
    {
    }


protected:

    void postStep() override { Visible::draw(); }

    void draw(Renderer& renderer) override
    {
        // Rgba contourColor = Rgba::filled(255) - color_;

        // renderer.drawContouredPolygon(
        //     properties().centroid,
        //     collider().vertexView(),
        //     color_,
        //     contourColor,
        //     100 * material().penetration.value
        // );

        renderer.drawPolygon(centroid(), collider().vertexView(), color_);
    }

private:

    Rgba color_;
};


class VisibleContactConstraint : public ContactConstraint, public Visible
{
public:

    VisibleContactConstraint(Bodies<2> bodies, Renderer* renderer)
        : ContactConstraint{bodies}, Visible{renderer}
    {
    }

protected:

    void postStep() override { Visible::draw(); }

    void draw(Renderer& renderer) override
    {
        ContactInfo info = contactInfo();

        for (Uint32 i = 0; i < info.nContacts; ++i)
        {
            Rgba white = Rgba::filled(255);
            renderer.drawPoint(info.refContacts[i], white, 0.1f);
            renderer.drawLine(
                info.refContacts[i],
                info.refContacts[i] + info.normal,
                white,
                0.1f
            );

            renderer.drawPoint(info.incContacts[i], white, 0.1f);
        }
    }
};

class VisibleDistanceConstraint : public DistanceConstraint, public Visible
{
public:

    VisibleDistanceConstraint(
        const Bodies<2>&    bodies,
        std::array<Vec2, 2> fixedPoints,
        Renderer*           renderer,
        bool                disableContacts = true
    )
        : DistanceConstraint{bodies, fixedPoints, disableContacts},
          Visible{renderer}
    {
    }

    VisibleDistanceConstraint(
        const Bodies<2>&    bodies,
        std::array<Vec2, 2> fixedPoints,
        float               distance,
        Renderer*           renderer,
        bool                disableContacts = true
    )
        : DistanceConstraint{bodies, fixedPoints, distance, disableContacts},
          Visible{renderer}
    {
    }

    VisibleDistanceConstraint(
        const Bodies<2>&     bodies,
        std::array<Vec2, 2>  fixedPoints,
        std::optional<float> minDistance,
        std::optional<float> maxDistance,
        Renderer*            renderer,
        bool                 disableContacts = true
    )
        : DistanceConstraint{bodies, fixedPoints, minDistance, maxDistance, disableContacts},
          Visible{renderer}
    {
    }


protected:

    void postStep() override { Visible::draw(); }

    void draw(Renderer& renderer) override
    {
        Rgba turquoise = Rgba{50, 235, 235, 255};
        auto points    = f.worldSpaceFixedPoints(getBodies());
        renderer.drawLine(points[0], points[1], turquoise, 0.1f);
    }
};

class VisibleMouseConstraint : public MouseConstraint, public Visible
{
public:

    VisibleMouseConstraint(Body* b, Vec2 pos, Renderer* renderer)
        : MouseConstraint{b, pos}, Visible{renderer}
    {
    }

protected:

    void postStep() override { Visible::draw(); }

    void draw(Renderer& renderer) override
    {
        if (this->isActive() && any(bodyPos() != mousePos()))
        {
            Rgba purple = Rgba{235, 50, 235, 255};
            renderer.drawLine(bodyPos(), mousePos(), purple, 0.1f);
        }
    }
};


} // namespace simu
