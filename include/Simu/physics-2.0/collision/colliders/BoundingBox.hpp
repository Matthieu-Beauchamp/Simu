////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2025 Matthieu Beauchamp-Boulay
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
#include "Simu/math/Geometry.hpp"

namespace simu
{

/// \brief An axis-aligned bounding box defined by a minimum and maximum point
///
/// If minimum == maximum, then the box is a single point.
///
/// The bounding box is invalid if any coordinates of the minimum are greater than
///     their maximum counterpart.
///
/// An invalid bounding box overlaps nothing and is ignored when combining boxes.
class BoundingBox
{
public:

    /// Creates an invalid bounding box
    BoundingBox() : BoundingBox(Vec2{1, 0}, Vec2{0, 0}) {}

    /// Creates a bounding box from the specified min and max points.
    BoundingBox(Vec2 min, Vec2 max): min_{min}, max_{max} {}

    /// Creates a bounding box that contains all the vertices
    template <VertexIterator2D It>
    BoundingBox(It begin, It end) {
        min_ = max_ = *begin;
        for (auto it = begin + 1; it != end; ++it) {
            min_ = simu::min(min_, *it);
            max_ = simu::max(max_, *it);
        }
    }

    /// Creates a bounding box that contains all the vertices of the geometry
    template <Geometry T>
    BoundingBox(const T& geometry)
        : BoundingBox(geometry.begin(), geometry.end()) {}

    /// true if the bounding box covers a valid area,
    /// \see BoundingBox
    bool isValid() const { return all(max() >= min()); }

    /// The minimum coordinate
    Vec2 min() const { return min_; }

    /// The maximum coordinate
    Vec2 max() const { return max_; }

    /// The center of the box
    Vec2 center() const { return (max() + min()) / 2.f; }

    /// true if this overlaps other, if the borders touch, the boxes overlap.
    /// If this or other is invalid, they never overlap.
    bool overlaps(const BoundingBox& other) const {
        if (!isValid() || !other.isValid())
            return false;
        return all(Interval{min_, max_}.overlaps(Interval{other.min_, other.max_}));
    }

    /// True if this bounding box fully contains other
    bool contains(const BoundingBox& other) const {
        if (!isValid() || !other.isValid())
            return false;

        return this->combined(other) == *this;
    }

    /// Returns a bounding box that covers this and other.
    BoundingBox combined(const BoundingBox& other) const {
        if (!isValid())
            return other;

        if (!other.isValid())
            return *this;

        return BoundingBox(simu::min(min_, other.min()), simu::max(max_, other.max()));
    }

    /// true if both boxes are invalid or they have the same min and max coordinates.
    bool operator==(const BoundingBox& other) const {
        if (!isValid() && !other.isValid())
            return true;

        return all(min() == other.min() && max() == other.max());
    }

    bool operator!=(const BoundingBox& other) const {
        return !(*this == other);
    }

private:

    Vec2 min_;
    Vec2 max_;
};

} // namespace simu
