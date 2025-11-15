////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2025 Matthieu Beauchamp-Boulay
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
#include "Simu/math/Matrix.hpp"
#include "Simu/physics/Transform.hpp"

namespace simu
{

class Position
{
public:

    explicit Position() = default;

    /// @param position The world space position
    /// @param orientation The world space orientation (radians)
    Position(Vec2 position, float orientation)
        : pos_{position}, orientation_{orientation} {}

    /// @return The world space position
    Vec2 position() const { return pos_.offset(); }

    /// @return The world space orientation (radians)
    float orientation() const { return orientation_.theta(); }

    /// Update the position
    /// @param dPos position offset
    /// @param dTheta angle offset (radians)
    void advance(Vec2 dPos, float dTheta) {
        pos_ *= Translation{dPos};
        orientation_ *= Rotation{dTheta};
    }

    /// @return The transform from this object's local space to world space
    Transform toWorldSpace() const { return pos_ * orientation_; }

    /// @return The transform from world space to this object's local space
    Transform toLocalSpace() const {
        return orientation_.inverse() * pos_.inverse();
    }

private:

    Translation pos_         = Translation(Vec2{0.f, 0.f});
    Rotation    orientation_ = Rotation(0.f);
};

} // namespace simu
