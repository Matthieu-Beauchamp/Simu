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

#include "Simu/physics/BoundingBox.hpp"

namespace simu
{


BoundingBox::BoundingBox(Vec2 min, Vec2 max) : min_{min}, max_{max} {}

BoundingBox BoundingBox::scaled(BoundingBox original, float ratio)
{
    auto transform = [&](Vec2 p) {
        return original.center() + ratio * (p - original.center());
    };

    return BoundingBox{transform(original.min()), transform(original.max())};
}


bool BoundingBox::overlaps(const BoundingBox& other) const
{
    if (!isValid() || !other.isValid())
        return false;

    return all(Interval{min_, max_}.overlaps(Interval{other.min_, other.max_}));
}

bool BoundingBox::contains(const BoundingBox& other) const
{
    if (!isValid() || !other.isValid())
        return false;

    return this->combined(other) == *this;
}


BoundingBox BoundingBox::combined(const BoundingBox& other) const
{
    if (!isValid())
        return other;

    if (!other.isValid())
        return *this;

    return BoundingBox{std::min(min_, other.min_), std::max(max_, other.max_)};
}

} // namespace simu
