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
#include "Simu/math/Matrix.hpp"
#include "Simu/math/Geometry.hpp"

namespace simu
{

class BoundingBox
{
public:

    BoundingBox() : BoundingBox(Vec2{1, 0}, Vec2{0, 0}) {}

    BoundingBox(Vec2 min, Vec2 max);

    template <VertexIterator2D It>
    BoundingBox(It begin, It end);

    template <Geometry T>
    BoundingBox(const T& geoemtry);

    bool isValid() const { return all(max() >= min()); }

    Vec2 min() const { return min_; }
    Vec2 max() const { return max_; }

    bool overlaps(const BoundingBox& other) const;

    BoundingBox combined(const BoundingBox& other) const;

    bool operator==(const BoundingBox& other) const
    {
        if (!isValid() && !other.isValid())
            return true;

        return all(min() == other.min() && max() == other.max());
    }

    bool operator!=(const BoundingBox& other) const
    {
        return !(*this == other);
    }

private:

    Vec2 min_;
    Vec2 max_;
};


template <VertexIterator2D It>
BoundingBox::BoundingBox(It begin, It end)
{
    if (begin == end)
        return;

    min_ = max_ = *begin;

    while (++begin != end)
    {
        min_ = std::min(min_, *begin);
        max_ = std::max(max_, *begin);
    }
}


template <Geometry T>
BoundingBox::BoundingBox(const T& geometry)
    : BoundingBox{geometry.begin(), geometry.end()}
{
}

} // namespace simu
