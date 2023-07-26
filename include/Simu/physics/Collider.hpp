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
#include "Simu/math/Polygon.hpp"

#include "Simu/physics/BoundingBox.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief Represents geometry that moves
///
/// The Collider holds a local space Polygon and a set of transformed vertices
///     that defines the geometry in world space.
///
/// All of its geometry is always positively oriented.
///
/// The local space geometry does not need to have its centroid at origin,
///     but care should be taken to apply the correct transformations
///     (notably for rotating about the centroid).
///
////////////////////////////////////////////////////////////
class Collider
{
public:

    typedef FreeListAllocator<Vec2> Alloc;

    ////////////////////////////////////////////////////////////
    /// Creates a Collider from a local space polygon and an initial transform.
    ////////////////////////////////////////////////////////////
    Collider(const Polygon& polygon, Mat3 transform, const Alloc& alloc)
        : local_{polygon.begin(), polygon.end(), alloc}, transformed_{alloc}
    {
        transformed_.resize(local_.size());
        update(transform);
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
            transformed_.data(),
            transformed_.data() + transformed_.size()
        );
    }

    ////////////////////////////////////////////////////////////
    /// Changes the transform of the Collider applied to the local space geometry
    ////////////////////////////////////////////////////////////
    void update(const Mat3& transform)
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
};


} // namespace simu
