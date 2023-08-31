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

#include <concepts>

#include "Simu/config.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief Contains various shortcuts for using Allocators
///
////////////////////////////////////////////////////////////
namespace mem
{

template <class A>
using Traits = std::allocator_traits<A>;


template <class A>
using Value = typename A::value_type;

template <class A>
using Pointer = typename Traits<A>::pointer;


template <class A>
using Size = typename Traits<A>::size_type;

template <class A>
using Difference = typename Traits<A>::difference_type;


template <class Alloc, class T>
using ReboundTo = typename std::allocator_traits<Alloc>::template rebind_alloc<T>;


// basic checks for Allocator requirements.
// The following should be respected:
// https://en.cppreference.com/w/cpp/named_req/Allocator
template <class A>
concept Allocator = requires(A alloc, A alloc2, Pointer<A> p, Size<A> n) {
    typename A::value_type;

    // clang-format off
    { alloc.allocate(n) } -> std::same_as<Pointer<A>>;
    { alloc.deallocate(p, n) };

    { alloc == alloc2 } -> std::same_as<bool>;
    { alloc != alloc2 } -> std::same_as<bool>;

    { A(alloc) };
    { A(std::move(alloc)) };
    { alloc = alloc2 } -> std::same_as<A&>;
    // clang-format on
} && std::convertible_to<ReboundTo<A, int>, ReboundTo<A, double>>;


template <class A1, class A2>
concept CompatibleAllocator = Allocator<A1> && Allocator<A2>
                              && std::is_same_v<A1, ReboundTo<A2, Value<A1>>>;


template <class T, class A>
T* allocate(A& alloc, Size<A> n)
{
    ReboundTo<A, T> reboundAlloc{alloc};
    return std::allocator_traits<ReboundTo<A, T>>::allocate(reboundAlloc, n);
}

template <class T, class A>
void destroy(A& alloc, T* p)
{
    ReboundTo<A, T> reboundAlloc{alloc};
    std::allocator_traits<ReboundTo<A, T>>::destroy(reboundAlloc, p);
}

template <class T, class A>
void deallocate(A& alloc, T* p, Size<A> n)
{
    ReboundTo<A, T> reboundAlloc{alloc};
    std::allocator_traits<ReboundTo<A, T>>::deallocate(reboundAlloc, p, n);
}

template <class T, class... Args>
void construct(T* where, Args&&... args)
{
    std::construct_at(where, std::forward<Args>(args)...);
}

} // namespace mem
} // namespace simu
