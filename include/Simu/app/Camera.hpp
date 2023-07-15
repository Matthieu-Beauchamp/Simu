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
#include "Simu/physics/BoundingBox.hpp"

namespace simu
{

class Camera
{
public:

    Camera();

    // a ratio of 2 zooms in, the looking at rect will be twice as small
    float zoom() const;
    void  setZoom(float ratio);

    // The current center of the camera's view
    Vec2 center() const;

    // pans or translate the camera's view by offset
    void pan(Vec2 offset);
    void panTo(Vec2 center);

    // the transform takes lookingAt and maps it to x in [-1, 1] and y in [-1, 1].
    Mat3 transform() const;

    // The desired size of a pixel in scene coordinates.
    float pixelSize() const { return pixelSize_; }
    void  setPixelSize(float size) { pixelSize_ = size; }


    // does not consider zoom
    Vec2 viewDimensions() const;

    // updated by application, notably on window resizes.
    void setViewDimensions(Vec2 dimensions);
    void setDimensionsFromPixels(Vec2 pixelDimensions);

private:

    Vec2 center_{};

    Vec2 viewDimensions_{100, 100}; // in scene units, ignoring zoom

    float zoom_ = 1.f;              // zooms in if zoom > 1

    float pixelSize_ = 1.f / 10.f;  // 10 pixels per scene unit.
};


} // namespace simu
