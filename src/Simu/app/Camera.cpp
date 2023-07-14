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

#include "Simu/app/Camera.hpp"
#include "Simu/physics/Transform.hpp"

namespace simu
{


Camera::Camera(BoundingBox lookAt) : lookAt_{lookAt} {}

void        Camera::lookAt(BoundingBox where) { lookAt_ = where; }
BoundingBox Camera::lookingAt() const { return lookAt_; }

void Camera::zoom(float ratio)
{
    Vec2 center = lookAt_.center();
    Mat3 scale  = Transform::translation(center)
                 * Mat3::diagonal(Vec3::filled(1.f / ratio))
                 * Transform::translation(-center);

    lookAt_ = BoundingBox{scale * lookAt_.min(), scale * lookAt_.max()};
}

void Camera::pan(Vec2 offset)
{
    lookAt_ = BoundingBox{offset + lookAt_.min(), offset + lookAt_.max()};
}

Mat3 Camera::transform() const
{
    Vec2 dim = lookAt_.max() - lookAt_.min();
    return Mat3::diagonal(Vec3{1.f / dim[0], 1.f / dim[1], 1.f})
           * Transform::translation(-lookAt_.center());
}


} // namespace simu
