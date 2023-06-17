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

namespace simu
{

class Transform
{
public:

    Transform() = delete;

    static Mat3 identity() { return Mat3::identity(); }

    // clang-format off
    static Mat3 rotation(float theta) 
    { 
        return Mat3{
            std::cos(theta), -std::sin(theta), 0,
            std::sin(theta),  std::cos(theta), 0,
            0,                0,               1
        }; 
    }

    static Mat3 translation(Vec2 offset) 
    { 
        return Mat3{
            1, 0, offset[0],
            0, 1, offset[1],
            0, 0, 1,
        }; 
    }
    // clang-format on

    static Mat3 transform(float theta, Vec2 offset)
    {
        return translation(offset) * rotation(theta);
    }
};

template <class T, class U>
Vector<Promoted<T, U>, 2>
operator*(const Matrix<T, 3, 3>& transform, const Vector<U, 2>& vec)
{
    Vector<Promoted<T, U>, 3> res = transform * Vector<U, 3>{vec[0], vec[1], 1};
    return Vector<Promoted<T, U>, 2>{res[0], res[1]};
}

} // namespace simu
