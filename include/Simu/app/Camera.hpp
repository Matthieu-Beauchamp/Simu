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

// TODO: What happens when the window is resized?
// See Application.cpp -> frameBufferResizeCallback...
// This whole class will need to be revised
class Camera
{
public:

    Camera(BoundingBox lookAt = BoundingBox{Vec2{0, 0}, Vec2{100, 100}});

    void        lookAt(BoundingBox where);
    BoundingBox lookingAt() const;

    // a ratio of 2 zooms in, the looking at rect will be twice as small
    void zoom(float ratio);

    // pans or translate the camera's view by offset
    void pan(Vec2 offset);

    // the transform takes lookingAt and maps it to x in [-1, 1] and y in [-1, 1].
    Mat3 transform() const;


private:

    BoundingBox lookAt_;
};


} // namespace simu
