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

////////////////////////////////////////////////////////////
/// \brief Static helper class for creating transformation matrices.
///
/// Transformation matrices are always 3x3 for 2D.
/// A global operator* overload is provided to multiply Mat3 * Vec2 -> Vec2,
///     this overload will apply translations.
/// If translations should not be applied, then Transform::linear(Mat3, Vec2) -> Vec2 should be used.
/// 
/// To combine transformation matrices, multiply them together in order:
/// T = T3 * T2 * T1
/// where T1 is the transformation that acts first, then T2 and is T3 the last transformation.
/// 
/// To apply T to a vector v: v' = T * v
/// 
////////////////////////////////////////////////////////////
class Transform
{
public:

    Transform() = delete;

    ////////////////////////////////////////////////////////////
    /// \brief No-op transformation
    ///
    ////////////////////////////////////////////////////////////
    static Mat3 identity() { return Mat3::identity(); }

    // clang-format off
    
    ////////////////////////////////////////////////////////////
    /// \brief Rotates a 2D vector by theta radians around the origin {0, 0}
    /// 
    ////////////////////////////////////////////////////////////
    static Mat3 rotation(float theta) 
    { 
        return Mat3{
            std::cos(theta), -std::sin(theta), 0,
            std::sin(theta),  std::cos(theta), 0,
            0,                0,               1
        }; 
    }

    ////////////////////////////////////////////////////////////
    /// \brief Translate a 2D vector by offset 
    /// 
    ////////////////////////////////////////////////////////////
    static Mat3 translation(Vec2 offset) 
    { 
        return Mat3{
            1, 0, offset[0],
            0, 1, offset[1],
            0, 0, 1,
        }; 
    }
    // clang-format on

    ////////////////////////////////////////////////////////////
    /// \brief Combines Transform::translation and Transform::rotation
    ///
    /// The rotation is applied first, and the translation is applied after.
    ///
    ////////////////////////////////////////////////////////////
    static Mat3 transform(float theta, Vec2 offset)
    {
        return transformAround(theta, offset, Vec2{0, 0});
    }

    ////////////////////////////////////////////////////////////
    /// \brief Rotates around transformOrigin by theta radians and then translates by offset.
    ///
    ////////////////////////////////////////////////////////////
    static Mat3 transformAround(float theta, Vec2 offset, Vec2 transformOrigin)
    {
        return translation(offset + transformOrigin) * rotation(theta)
               * translation(-transformOrigin);
    }

    ////////////////////////////////////////////////////////////
    /// \brief Transforms v by transform, ignoring translations
    /// 
    /// 
    ////////////////////////////////////////////////////////////
    static Vec2 linear(Mat3 transform, Vec2 v)
    {
        Vec3 res = transform* Vec3{v[0], v[1], 0};
        return Vec2{res[0], res[1]};
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
