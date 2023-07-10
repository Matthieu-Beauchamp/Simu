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

#include <random>
#include <concepts>

#include "Simu/config.hpp"

namespace simu
{

class Random
{
public:

    template <class Dist>
    static auto generate(Dist& dist)
    {
        return dist(engine_);
    }

    template <class Dist, class... Args>
    static auto generate(Args&&... args)
    {
        Dist dist{std::forward<Args>(args)...};
        return generate<Dist>(dist);
    }

    template <std::integral T>
    static T uniform(T min, T max)
        requires(sizeof(T) > 1)
    {
        return generate<std::uniform_int_distribution<T>>(min, max);
    }

    template <std::floating_point T>
    static T uniform(T min, T max)
    {
        return generate<std::uniform_real_distribution<T>>(min, max);
    }

private:

    static std::mt19937_64 engine_;
};


} // namespace simu
