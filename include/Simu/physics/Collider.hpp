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

// TODO: In time, this can be made abstract to handle concave Polygon
//      with convex decomposition

// TODO: Currently the entire polygon is saved, we can save space by storing
//      only the convex hull since other vertices will never be needed


class Collider
{
public:

    Collider(const Polygon& polygon, Mat3 transform) : local_{polygon}
    {
        // TODO: local_ should have its centroid at {0, 0}
        transformed_.resize(std::distance(local_.begin(), local_.end()));
        update(transform);
    }

    BoundingBox boundingBox() const { return boundingBox_; }

    const Polygon& local() const { return local_; }

    Vertices::iterator       begin() { return transformed_.begin(); }
    Vertices::const_iterator begin() const { return transformed_.begin(); }

    Vertices::iterator       end() { return transformed_.end(); }
    Vertices::const_iterator end() const { return transformed_.end(); }

private:

    friend class PhysicsBody;
    void update(const Mat3& transform)
    {
        auto it = transformed_.begin();
        for (const Vertex& v : local_)
            *it++ = transform * v;

        boundingBox_ = BoundingBox{transformed_};
    }

    Polygon local_;

    Vertices    transformed_;
    BoundingBox boundingBox_;
};


} // namespace simu
