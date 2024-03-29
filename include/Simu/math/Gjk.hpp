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

namespace priv
{

struct Simplex
{
    inline Simplex(Vec2 initialSearchDir = Vec2::i());

    inline void pushPoint(Vertex v);
    inline Vec2 nextDirection() const;

    inline bool hasPoint(Vertex v) const;
    inline bool containsOrigin() const;

    std::array<Vertex, 3> pointStack{};

    // hold the outwards facing normals of edges P0-P1, P0-P2, P1-P2
    // respectively, updated on pushPoint once there are 3 valid vertices
    std::array<Vec2, 3> normals{};

    Vec2   initialSearchDir_;
    Uint32 nIterations = 0;

private:

    mutable bool keepBottomPoint_ = false;
};


struct Polytope
{
    typedef typename Edges<Vertices>::Edge Edge;

    inline Polytope(const Simplex& simplex);

    inline bool addVertex(const Edge& where, Vertex v);

    Vertices vertices{};
};

} // namespace priv


////////////////////////////////////////////////////////////
/// \ingroup Geometry
/// \brief Determines if two collidable are in collision as well as the
///     separation/penetration vector between them
///
/// The boolean GJK algorithm is used upon construction and results are saved.
/// If areColliding(), then calling penetration() uses the EPA algorithm (results are not saved)
///
/// If the the underlying Geometry of the Collidables is concave or has holes,
///     only their convex hulls are considered.
///
/// \warning changing first or second between construction and calls to any method is undefined.
////////////////////////////////////////////////////////////
template <Geometry T = simu::Polygon>
class Gjk
{
public:

    // TODO: initialDir is not correctly taken into account, + what are we supposed to pass in?
    Gjk(const T& first, const T& second, Vec2 initialDir = Vec2::i());

    bool areColliding() const { return areColliding_; }

    ////////////////////////////////////////////////////////////
    /// \brief Minimum translation vector such that areColliding() would be true
    ///
    /// It is the vector of the closest feature of first to the closest feature of second
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
    /// It is the vector of the penetration of second into first
    ///
    /// If not areColliding(), returns a null vector.
    ///
    /// In order to make first and second only touch, both are equivalent:
    ///     - translate first by -penetration()
    ///     - translate second by penetration()
    ///
    ////////////////////////////////////////////////////////////
    Vec2 penetration() const;

private:

    Vec2 furthestVertexInDirection(const Vec2& direction) const;

    const T& first_;
    const T& second_;

    priv::Simplex simplex_;

    bool done_         = false;
    bool areColliding_ = false;
};


} // namespace simu

#include "Gjk.inl.hpp"
