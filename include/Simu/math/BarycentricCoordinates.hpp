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

#include "Simu/math/Matrix.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \defgroup BarycentricCoordinates
/// \ingroup Geometry
/// 
/// \brief Barycentric coordinates allows finding the closest point on a 
///     line or triangle from a point Q.
/// 
/// For an intuitive explanation, see Erin Catto's GDC conference,
/// slides available at https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf
///
/// \{
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
/// \brief Barycentric coordinates of Q for the line A -> B.
///
/// closestPoint is the point on A->B closest to Q
///
/// The following equation holds if u and v are both greater than 0:
///     closestPoint = u*A + v*B;
///
/// \warning coordinates are normalized, they are such that u+v = 1
////////////////////////////////////////////////////////////
struct LineBarycentric
{
    LineBarycentric(Vec2 A, Vec2 B, Vec2 Q);

    Vec2  closestPoint;
    float u;
    float v;
};

////////////////////////////////////////////////////////////
/// \brief Barycentric coordinates of Q for the triangle ABC.
///
/// ABC can be given in any winding order.
///
/// closestPoint will be on vertices or edges if Q is outside ABC, 
///     else closestPoint = Q.
///
/// \warning coordinates are not normalized, they are such that u+v+w = abs(area(ABC))
////////////////////////////////////////////////////////////
struct TriangleBarycentric
{
    TriangleBarycentric(Vec2 A, Vec2 B, Vec2 C, Vec2 Q);

    LineBarycentric AB;
    LineBarycentric BC;
    LineBarycentric CA;

    Vec2  closestPoint;
    float u;
    float v;
    float w;
};

/// \}

} // namespace simu
