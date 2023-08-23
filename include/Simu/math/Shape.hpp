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
#include <vector>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/math/Transform.hpp"

#include "Simu/utility/Memory.hpp"
#include "Simu/utility/Algo.hpp"

namespace simu
{

class Shape
{
public:

    enum Type : Uint32
    {
        polygon = 0,
        circle,
        BEGIN_USER_TYPE
    };

    struct Properties
    {
        ////////////////////////////////////////////////////////////
        /// \brief The centroid of the shape
        ///
        ////////////////////////////////////////////////////////////
        Vec2 centroid{};

        ////////////////////////////////////////////////////////////
        /// \brief Area of the shape
        ///
        ////////////////////////////////////////////////////////////
        float area = 0.f;

        ////////////////////////////////////////////////////////////
        /// \brief Second polar moment of area of the shape
        ///
        /// https://en.wikipedia.org/wiki/Second_polar_moment_of_area#Definition
        ////////////////////////////////////////////////////////////
        float momentOfArea = 0.f;

        ////////////////////////////////////////////////////////////
        /// \brief Indicates that the shape is degenerate / invalid.
        ///
        ////////////////////////////////////////////////////////////
        bool isDegenerate = false;
    };


    Shape(Uint32 type) : type_{type} {}
    virtual ~Shape() = default;

    Uint32 type() const { return type_; }

    virtual Properties properties() const = 0;

private:

    Uint32 type_;
};


////////////////////////////////////////////////////////////
/// \brief Determines collision information of Shapes A and B.
///
////////////////////////////////////////////////////////////
struct CollisionManifold
{
    ////////////////////////////////////////////////////////////
    /// \brief Number of contact points [0, 2]
    ///
    /// if 0, then there is no collision.
    ///
    ////////////////////////////////////////////////////////////
    Uint32 nContacts;

    ////////////////////////////////////////////////////////////
    /// \brief Contact points on Shape A.
    ///
    /// Only the first nContacts entries are valid data.
    ///
    ////////////////////////////////////////////////////////////
    std::array<Vec2, 2> contactsA;

    ////////////////////////////////////////////////////////////
    /// \see contactsA
    ////////////////////////////////////////////////////////////
    std::array<Vec2, 2> contactsB;

    ////////////////////////////////////////////////////////////
    /// \brief The normalized contact normal, pointing out of Shape B.
    ///
    ////////////////////////////////////////////////////////////
    Vec2 normal;

    ////////////////////////////////////////////////////////////
    /// \brief The normalized contact tangent.
    ///
    /// cross(normal, tangent) == 1.f
    ////////////////////////////////////////////////////////////
    Vec2 tangent;

    ////////////////////////////////////////////////////////////
    /// \brief Transforms the collision information.
    ///
    /// This is useful for converting the world space collision information
    ///     to a local space coordinates and back.
    ///
    /// The collision manifold can be tested for validity by checking
    ///     penetration along the contact normal.
    ///
    /// \param transformA The transformation of Shape A
    /// \param transformB The transformation of Shape B
    ////////////////////////////////////////////////////////////
    CollisionManifold
    transformed(const Transform& transformA, const Transform& transformB)
    {
        CollisionManifold trans;

        trans.nContacts = nContacts;

        for (Uint32 i = 0; i < nContacts; ++i)
        {
            trans.contactsA[i] = transformA * contactsA[i];
            trans.contactsB[i] = transformB * contactsB[i];
        }

        trans.normal  = transformB.rotation() * normal;
        trans.tangent = transformB.rotation() * tangent;
        return trans;
    }
};


typedef CollisionManifold (*CollisionCallback)(const Shape&, const Shape&);

namespace details
{

template <Uint32 shapeTypeA, Uint32 shapeTypeB, CollisionCallback callback>
CollisionManifold symetricCallback(const Shape& shapeA, const Shape& shapeB)
{
    static_assert(shapeTypeA > shapeTypeB, "");
    SIMU_ASSERT(shapeA.type() == shapeTypeA, "");
    SIMU_ASSERT(shapeB.type() == shapeTypeB, "");

    CollisionManifold inverted = callback(shapeB, shapeA);

    for (Uint32 i = 0; i < inverted.nContacts; ++i)
    {
        std::swap(inverted.contactsA[i], inverted.contactsB[i]);
    }

    inverted.normal  = -inverted.normal;
    inverted.tangent = -inverted.tangent;
    return inverted;
}

} // namespace details


template <class Alloc>
class ShapeCollider
{
public:

    ShapeCollider() = default;

    CollisionManifold collide(const Shape& A, const Shape& B)
    {
        CollisionCallback callback = callbacks_[indexOf(A.type(), B.type())];
        SIMU_ASSERT(
            callback != nullptr, "No callback registered corresponds to the shape types."
        );

        return callback(A, B);
    }

    template <Uint32 shapeTypeA, Uint32 shapeTypeB, CollisionCallback callback>
    void registerCollisionCallback()
    {
        static_assert(
            shapeTypeA <= shapeTypeB,
            "Callbacks for different arguments order will be generated "
            "automatically. Only provide the callbacks where shapeTypeA <= "
            "shapeTypeB"
        );

        resizeToFitType(std::max(shapeTypeA, shapeTypeB));
        callbacks_[indexOf(shapeTypeA, shapeTypeB)] = callback;

        if constexpr (shapeTypeA < shapeTypeB)
        {
            CollisionCallback symetric
                = details::symetricCallback<shapeTypeB, shapeTypeA, callback>;

            callbacks_[indexOf(shapeTypeB, shapeTypeA)] = symetric;
        }
    }

private:

    void resizeToFitType(Uint32 type)
    {
        Uint32 size = requiredSizeFor(type);
        if (size > callbacks_.size())
        {
            Callbacks tmp{std::move(callbacks_)};
            callbacks_.resize(size);

            for (Uint32 typeA = 0; typeA < lastType_; ++typeA)
            {
                Uint32 base = typeA * type;
                for (Uint32 typeB = 0; typeB < lastType_; ++typeB)
                {
                    callbacks_[base + typeB] = tmp[indexOf(typeA, typeB)];
                }
            }

            lastType_ = type;
        }
    }

    Uint32 requiredSizeFor(Uint32 type) { return squared(type + 1); }

    Uint32 indexOf(Uint32 typeA, Uint32 typeB)
    {
        return typeA * (lastType_+1) + typeB;
    }

    typedef std::vector<CollisionCallback, ReboundTo<Alloc, CollisionCallback>> Callbacks;

    Callbacks callbacks_{};
    Uint32    lastType_ = 0;
};


} // namespace simu
