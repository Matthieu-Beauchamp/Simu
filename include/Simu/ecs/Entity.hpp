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


#include <cstddef>
#include <functional>

namespace simu
{

class Entity
{
private:

    std::size_t _id;

public:

    explicit Entity(std::size_t id) : _id(id) {}

    std::size_t id() const { return _id; }

    bool operator==(const Entity&) const = default;
};

} // namespace simu

template <>
struct std::hash<simu::Entity>
{
    std::size_t operator()(const simu::Entity& s) const noexcept
    {
        return std::hash<std::size_t>{}(s.id());
    }
};
