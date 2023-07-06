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

#include <ranges>

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief Dereferences the element one more time to hide the use of smart pointers
////////////////////////////////////////////////////////////
struct BypassSmartPointer
{
    template <class Value>
    auto& operator()(Value& v) const
    {
        return *v;
    }

    template <class Value>
    const auto& operator()(const Value& v) const
    {
        return *v;
    }
};

template <std::input_iterator Iter, class Sentinel>
auto makeView(Iter begin, Sentinel end)
{
    return std::ranges::subrange<Iter, Sentinel>{begin, end};
}

////////////////////////////////////////////////////////////
/// \brief Creates a view of [begin, end) and applies Fn to every dereferenced element
///
/// The original sequence is not modified.
///
////////////////////////////////////////////////////////////
template <std::input_iterator Iter, class Sentinel, class Fn>
auto makeView(Iter begin, Sentinel end, Fn deref)
{
    return std::ranges::subrange<Iter, Sentinel>{begin, end}
           | std::views::transform(deref);
}

////////////////////////////////////////////////////////////
/// \brief Creates a view of the range and applies Fn to every dereferenced element
///
/// The original range is not modified.
///
////////////////////////////////////////////////////////////
template <std::ranges::range R, class Fn>
auto makeView(R& range, Fn deref)
{
    return makeView(range.begin(), range.end(), deref);
}

template <std::ranges::range R>
auto makeView(R& range)
{
    return makeView(range.begin(), range.end());
}

} // namespace simu
