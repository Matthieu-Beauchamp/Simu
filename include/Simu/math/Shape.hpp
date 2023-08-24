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

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/math/Transform.hpp"
#include "Simu/math/BoundingBox.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief
///
/// Subclasses should be marked final since the Type identifier is
/// expected to correspond to a specific subclass.
////////////////////////////////////////////////////////////
class Shape
{
public:

    enum Type : Uint32
    {
        polygon = 0,
        circle,
        BEGIN_USER_TYPE
    };

    struct Properties
    {
        ////////////////////////////////////////////////////////////
        /// \brief The centroid of the shape
        ///
        ////////////////////////////////////////////////////////////
        Vec2 centroid{};

        ////////////////////////////////////////////////////////////
        /// \brief Area of the shape
        ///
        ////////////////////////////////////////////////////////////
        float area = 0.f;

        ////////////////////////////////////////////////////////////
        /// \brief Second polar moment of area of the shape relative to its centroid
        ///
        /// https://en.wikipedia.org/wiki/Second_polar_moment_of_area#Definition
        ////////////////////////////////////////////////////////////
        float momentOfArea = 0.f;

        ////////////////////////////////////////////////////////////
        /// \brief Indicates that the shape is degenerate / invalid.
        ///
        ////////////////////////////////////////////////////////////
        bool isDegenerate = false;
    };


    Shape(Uint32 type) : type_{type} {}
    virtual ~Shape() = default;

    Uint32 type() const { return type_; }

    virtual void        copyAt(Shape* dest) const             = 0;
    virtual Properties  properties() const                    = 0;
    virtual BoundingBox boundingBox() const                   = 0;
    virtual void        transform(const Transform& transform) = 0;

private:

    Uint32 type_;
};

} // namespace simu
