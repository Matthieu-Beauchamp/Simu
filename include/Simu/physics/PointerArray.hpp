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

namespace simu
{

template <class T, Uint32 n, bool isConst>
class PointerArray
    : public std::array<std::conditional_t<isConst, const T*, T*>, n>
{
    typedef std::conditional_t<isConst, const T*, T*> value_type;

public:

    PointerArray(const std::initializer_list<value_type>& values)
    {
        Uint32 i = 0;
        for (value_type v : values)
        {
            if (i < n)
                (*this)[i++] = v;
        }
    }

    template <bool otherIsConst>
        requires(isConst || !otherIsConst)
    PointerArray(const PointerArray<T, n, otherIsConst>& other)
    {
        for (Uint32 i = 0; i < n; ++i)
            (*this)[i] = other[i];
    }
};

} // namespace simu
