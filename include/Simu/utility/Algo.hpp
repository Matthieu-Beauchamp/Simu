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

#include "Simu/utility/Callable.hpp"

namespace simu
{

template <class T>
inline T clamp(T val, T min, T max)
{
    return std::min(std::max(val, min), max);
}


template <class T>
inline T squared(const T& x)
{
    return x * x;
}

template <std::bidirectional_iterator Iter, Callable<bool(Iter)> GoesLeft>
Iter booleanSort(Iter begin, Iter end, GoesLeft goesLeft)
{
    while (begin != end)
    {
        if (goesLeft(begin))
            ++begin;
        else
            std::swap(*begin, *(--end));
    }

    return begin;
}

} // namespace simu
