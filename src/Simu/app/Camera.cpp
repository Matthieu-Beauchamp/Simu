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


Camera::Camera() {}

float Camera::zoom() const { return zoom_; }
void  Camera::setZoom(float ratio)
{
    if (ratio > 0.f)
        zoom_ = ratio;
}

Vec2 Camera::center() const { return center_; }
void Camera::pan(Vec2 offset) { center_ += offset; }
void Camera::panTo(Vec2 center) { pan(center - this->center()); }

Mat3 Camera::transform() const
{
    Vec2 dimensions = viewDimensions() / zoom();
    Mat3 scale = Mat3::diagonal(Vec3{2.f / dimensions[0], 2.f / dimensions[1], 1.f});

    return scale * Transform::translation(-center());
}

Mat3 Camera::invTransform() const
{
    Vec2 dimensions = viewDimensions() / zoom();
    Mat3 scale = Mat3::diagonal(Vec3{dimensions[0] / 2.f, dimensions[1] / 2.f, 1.f});

    return Transform::translation(center()) * scale;
}

Vec2 Camera::viewDimensions() const { return viewDimensions_; }

void Camera::setViewDimensions(Vec2 dimensions)
{
    if (all(dimensions > Vec2::filled(0.f)))
        viewDimensions_ = dimensions;
}

void Camera::setDimensionsFromScreenCoordinates(Vec2 pixelDimensions)
{
    setViewDimensions(pixelDimensions * pixelSize());
}

} // namespace simu
