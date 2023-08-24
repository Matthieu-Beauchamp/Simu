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

#include <list>

#include "Simu/config.hpp"
#include "Simu/math/Shape.hpp"
#include "Simu/math/BoundingBox.hpp"

#include "Simu/physics/Material.hpp"
#include "Simu/physics/ColliderTree.hpp"

#include "Simu/utility/Memory.hpp"

namespace simu
{

// TODO: We often need to transform properties
//  a simple method can be added.
struct MassProperties
{
    MassProperties() = default;

    MassProperties(const Shape::Properties& properties, float density)
    {
        SIMU_ASSERT(density > 0.f, "Must have a positive density");

        SIMU_ASSERT(
            !properties.isDegenerate, "Must have a surface. (not a point or line)."
        );

        m        = properties.area * density;
        I        = properties.momentOfArea * density;
        centroid = properties.centroid;
    }

    friend MassProperties
    operator+(const MassProperties& lhs, const MassProperties& rhs)
    {
        MassProperties combined{};

        combined.m = lhs.m + rhs.m;
        if (combined.m == 0.f)
            return combined; // all null

        combined.centroid = (lhs.centroid * lhs.m + rhs.centroid * rhs.m)
                            / combined.m;

        float inertiaAtOrigin = lhs.I + lhs.m * normSquared(lhs.centroid);
        inertiaAtOrigin += rhs.I + rhs.m * normSquared(rhs.centroid);

        combined.I = inertiaAtOrigin - combined.m * normSquared(combined.centroid);

        return combined;
    }

    float m        = 0.f;
    float I        = 0.f;
    Vec2  centroid = Vec2{};
};


class Body;

////////////////////////////////////////////////////////////
/// \brief Represents a shape that moves and its mass properties
///
/// Keeps a local space copy and a world space shape that is transformed
/// on updates.
///
/// A collider is associated with a single body.
///
////////////////////////////////////////////////////////////
class Collider
{
public:

    typedef FreeListAllocator<int> Alloc;

    // Must call make<>() after, otherwise the Collider is in an invalid state.
    Collider(Body* body, const Material& material)
        : body_{body}, material_{material}
    {
    }

    template <std::derived_from<Shape> S, class... Args>
    void make(const Alloc& alloc, Args&&... args)
    {
        typedef ReboundTo<Alloc, S> ShapeAlloc;
        ShapeAlloc                  a{alloc};

        local_ = std::allocator_traits<ShapeAlloc>::allocate(a, 2);
        world_ = std::next(static_cast<S*>(local_));

        std::allocator_traits<ShapeAlloc>::construct(
            a, static_cast<S*>(local_), std::forward<Args>(args)...
        );

        local_->copyAt(world_);

        dealloc_ = [=](Shape* local) mutable {
            S* first = static_cast<S*>(local);

            std::allocator_traits<ShapeAlloc>::destroy(a, first);
            std::allocator_traits<ShapeAlloc>::destroy(a, std::next(first));

            std::allocator_traits<ShapeAlloc>::deallocate(a, first, 2);
        };
    }

    Collider(const Collider&) = delete;
    Collider(Collider&&)      = delete;

    ~Collider() { dealloc_(local_); }

    Body*       body() { return body_; }
    const Body* body() const { return body_; }

    const Material& material() const { return material_; }

    Shape&       shape() { return *world_; }
    const Shape& shape() const { return *world_; }

    MassProperties localProperties() const
    {
        return MassProperties{local_->properties(), material_.density};
    }

    MassProperties worldProperties() const
    {
        return MassProperties{world_->properties(), material_.density};
    }

    void update(const Transform& transform)
    {
        local_->copyAt(world_);
        world_->transform(transform);
    }

private:

    Body*    body_;
    Material material_;

    Shape* local_ = nullptr;
    Shape* world_ = nullptr;

    std::function<void(Shape*)> dealloc_{};

    friend class World;
    typename ColliderTree::iterator treeLocation_; // only used for removal...
};


class Colliders
{
public:

    typedef Collider::Alloc Alloc;
    Colliders(const Alloc& alloc) : colliders_{alloc} {}

    auto begin() const { return colliders_.begin(); }
    auto end() const { return colliders_.end(); }

    bool isEmpty() const { return colliders_.empty(); }

    // properties of the local space combined colliders.
    const MassProperties& properties() const { return properties_; }

    template <std::derived_from<Shape> S, class... Args>
    Collider* add(Body* owner, const Material& material, Args&&... args)
    {
        colliders_.emplace_back(owner, material)
            .make<S>(colliders_.get_allocator(), std::forward<Args>(args)...);

        MassProperties p = colliders_.back().localProperties();
        properties_      = properties_ + p;

        return &colliders_.back();
    }

    void remove(Collider* collider)
    {
        for (auto it = colliders_.begin(); it != colliders_.end(); ++it)
        {
            if (&(*it) == collider)
            {
                colliders_.erase(it);
                break;
            }
        }

        properties_ = MassProperties{};
        for (const Collider& c : colliders_)
            properties_ = properties_ + c.localProperties();
    }

    void update(const Transform& transform)
    {
        for (Collider& c : colliders_)
            c.update(transform);
    }

private:

    friend class Body;
    typedef std::list<Collider, ReboundTo<Alloc, Collider>> ColliderList;

    ColliderList   colliders_{};
    MassProperties properties_{};
};


} // namespace simu
