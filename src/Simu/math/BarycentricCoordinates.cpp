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

#include "Simu/math/BarycentricCoordinates.hpp"
#include "Simu/math/Geometry.hpp"

#include "Simu/utility/Algo.hpp"

namespace simu
{

LineBarycentric::LineBarycentric(Vec2 A, Vec2 B, Vec2 Q)
{
    Vec2  AB     = B - A;
    float norm   = normSquared(AB);
    float t      = (norm == 0.f) ? 0.f : dot(Q - A, AB) / norm;
    closestPoint = A + clamp(t, 0.f, 1.f) * AB;

    v = t;
    u = 1.f - t;
}

TriangleBarycentric::TriangleBarycentric(Vec2 A, Vec2 B, Vec2 C, Vec2 Q)
    : AB{A, B, Q}, BC{B, C, Q}, CA{C, A, Q}
{
    Orientation sign = orientation(A, B, C);

    u = 0.5f * cross(B - Q, C - Q);
    v = 0.5f * cross(C - Q, A - Q);
    w = 0.5f * cross(A - Q, B - Q);

    if (sign == Orientation::negative)
    {
        u = -u;
        v = -v;
        w = -w;
    }

    // vertex regions
    if ((AB.v <= 0) && (CA.u <= 0))
        closestPoint = A;
    else if ((AB.u <= 0) && (BC.v <= 0))
        closestPoint = B;
    else if ((BC.u <= 0) && (CA.v <= 0))
        closestPoint = C;
    else // edge regions
    {
        if ((u <= 0) && (BC.u >= 0) && (BC.v >= 0))
            closestPoint = BC.closestPoint;
        else if ((v <= 0) && (CA.u >= 0) && (CA.v >= 0))
            closestPoint = CA.closestPoint;
        else if ((w <= 0) && (AB.u >= 0) && (AB.v >= 0))
            closestPoint = AB.closestPoint;
        else // interior region
        {
            if ((u >= 0) && (v >= 0) && (w >= 0))
                closestPoint = Q;
            else
                throw simu::Exception{"Could not determine the closest point"};
        }
    }
}


} // namespace simu
