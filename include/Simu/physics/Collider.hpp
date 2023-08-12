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
#include "Simu/math/Polygon.hpp"

#include "Simu/physics/BoundingBox.hpp"
#include "Simu/physics/Material.hpp"

namespace simu
{

struct MassProperties
{
    MassProperties() = default;

    // Assumes geometry is positively oriented.
    template <Geometry G>
    MassProperties(const G& geometry, float density)
    {
        SIMU_ASSERT(density > 0.f, "Must have a positive density");

        GeometricProperties properties{geometry};
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


struct ColliderDescriptor
{
    ////////////////////////////////////////////////////////////
    /// \brief The Geometry for the Collider. This is put as-is in the Body's local space.
    ///
    /// Does not need to be centered on origin.
    ///
    ////////////////////////////////////////////////////////////
    Polygon polygon;

    ////////////////////////////////////////////////////////////
    /// \brief The Material of the Collider. Affects how it interacts with other bodies.
    ///
    ////////////////////////////////////////////////////////////
    Material material{};
};


class Body;

////////////////////////////////////////////////////////////
/// \brief Represents geometry that moves
///
/// The Collider holds a local space Polygon and a set of transformed vertices
///     that defines the geometry in world space.
///
/// All of its geometry is always positively oriented.
///
/// The local space geometry will be recentered to have its centroid at origin
///
////////////////////////////////////////////////////////////
class Collider
{
public:

    typedef FreeListAllocator<Vec2> Alloc;

    ////////////////////////////////////////////////////////////
    /// Creates a Collider from a local space polygon and an initial transform.
    ////////////////////////////////////////////////////////////
    Collider(const ColliderDescriptor& descr, Body* body, const Alloc& alloc)
        : local_{descr.polygon.begin(), descr.polygon.end(), alloc},
          transformed_{alloc},
          material_{descr.material},
          body_{body}
    {
        transformed_.resize(local_.size());
    }

    Body*       body() { return body_; }
    const Body* body() const { return body_; }

    const Material& material() const { return material_; }

    MassProperties properties() const
    {
        return MassProperties{local_, material_.density};
    }

    void replaceAlloc(const Alloc& alloc)
    {
        replaceAllocator(local_, alloc);
        replaceAllocator(transformed_, alloc);
    }

    ////////////////////////////////////////////////////////////
    /// The world space bounding box of the Collider
    ////////////////////////////////////////////////////////////
    BoundingBox boundingBox() const { return boundingBox_; }

    ////////////////////////////////////////////////////////////
    /// Iterators for the transformed geometry (world space vertices)
    ////////////////////////////////////////////////////////////
    auto begin() { return transformed_.begin(); }
    auto begin() const { return transformed_.begin(); }

    auto end() { return transformed_.end(); }
    auto end() const { return transformed_.end(); }

    auto vertexView() const
    {
        return makeView(
            transformed_.data(), transformed_.data() + transformed_.size()
        );
    }

    ////////////////////////////////////////////////////////////
    /// Changes the transform of the Collider applied to the local space geometry
    ////////////////////////////////////////////////////////////
    void update(const Transform& transform)
    {
        auto it = transformed_.begin();
        for (const Vertex& v : local_)
            *it++ = transform * v;

        boundingBox_ = BoundingBox{transformed_};
    }

private:

    std::vector<Vec2, Alloc> local_;
    std::vector<Vec2, Alloc> transformed_;

    BoundingBox boundingBox_;
    Material    material_;
    Body*       body_;

    friend class World;
    typename ColliderTree::iterator treeLocation_; // only used for removal...
};


class Colliders
{
public:

    typedef Collider::Alloc Alloc;
    Colliders() = default;

    auto begin() const { return colliders_.begin(); }
    auto end() const { return colliders_.end(); }

    bool isEmpty() const { return begin() == end(); }

    void replaceAlloc(const Alloc& alloc)
    {
        replaceAllocator(colliders_, alloc);
        for (Collider& c : colliders_)
            c.replaceAlloc(alloc);
    }

    const MassProperties& properties() const { return properties_; }

    Collider* add(const ColliderDescriptor& descr, Body* owner)
    {
        colliders_.emplace_back(descr, owner, colliders_.get_allocator());

        MassProperties p = colliders_.back().properties();
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
            properties_ = properties_ + c.properties();
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
