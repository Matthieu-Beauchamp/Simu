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

#include "Simu/math/Shape.hpp"
#include "Simu/utility/Algo.hpp"

namespace simu
{

class Circle final : public Shape
{
public:

    Circle(float radius, Vec2 center = Vec2{})
        : Shape{circle}, radius_{radius}, center_{center}
    {
        SIMU_ASSERT(radius_ > 0.f, "");
    }

    float radius() const { return radius_; }
    Vec2  center() const { return center_; }

    void copyAt(Shape* dest) const override
    {
        std::construct_at(static_cast<Circle*>(dest), *this);
    }

    Properties properties() const override
    {
        Properties p{};

        p.centroid = center();

        float r2 = squared(radius());
        float pi = std::numbers::pi_v<float>;

        p.area         = pi * r2;
        p.momentOfArea = (pi / 2.f) * squared(r2);

        return p;
    }

    BoundingBox boundingBox() const override
    {
        Vec2 offset = Vec2::filled(radius());
        return BoundingBox{center() - offset, center() + offset};
    }

    void transform(const Transform& transform) override
    {
        center_ = transform * center_;
    }

private:

    float radius_;
    Vec2  center_;
};


} // namespace simu
