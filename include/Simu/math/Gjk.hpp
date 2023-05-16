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

#include <array>

#include "Simu/math/Polygon.hpp"

namespace simu
{

template <class T>
concept Collidable = requires(T collidable) {
    {
        collidable.furthestVertexInDirection(Vec2{})
    } -> std::convertible_to<Vec2>;
};

struct Simplex
{
    void pushPoint(Vertex v);
    Vec2 nextDirection() const;

    Vec2 closestPoint(Vec2 Q) const;

    std::array<Vertex, 3> pointStack{};

    // hold the outwards facing normals of edges P0-P1 and P0-P2 respectively,
    // updated on pushPoint once there are 3 valid vertices
    std::array<Vec2, 2> normals{};

    std::size_t nIterations = 0;

private:

    mutable bool keepBottomPoint_ = false;
};

template <Collidable T = simu::Polygon>
class Gjk
{
public:

    Gjk(const T& first, const T& second);

    bool areColliding() const { return areColliding_; }

    ////////////////////////////////////////////////////////////
    /// \brief Minimum translation vector such that areColliding() would be true  
    /// 
    /// If areColliding(), returns a null vector.
    /// 
    /// In order to make first and second only touch, both are equivalent:
    ///     - translate first by separation()
    ///     - translate seccond by -separation()
    ///
    ////////////////////////////////////////////////////////////
    Vec2 separation();

    ////////////////////////////////////////////////////////////
    /// \brief Minimum translation vector such that first and second are only touching
    /// 
    /// If not areColliding(), returns a null vector.
    /// 
    /// In order to make first and second only touch, both are equivalent:
    ///     - translate first by penetration()
    ///     - translate seccond by -penetration()
    ///     
    ////////////////////////////////////////////////////////////
    Vec2 penetration();

private:

    Vec2 furthestVertexInDirection(const Vec2& direction) const;

    const T& first_;
    const T& second_;

    Simplex simplex_;

    bool done_         = false;
    bool areColliding_ = false;
};


} // namespace simu

#include "Gjk.inl.hpp"
