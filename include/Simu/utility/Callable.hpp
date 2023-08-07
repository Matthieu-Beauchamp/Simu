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

#include <functional>
#include "Simu/config.hpp"

namespace simu
{

namespace details
{

template <class... Args>
struct Pack
{
};

template <class Signature>
struct SignatureDecomposition
{
};

template <class R, class... A>
struct SignatureDecomposition<R(A...)>
{
    typedef R          Return;
    typedef Pack<A...> Args;

    template <class F>
    struct IsCallable : public std::is_invocable_r<R, F, A...>
    {
    };

    template <class F>
    struct IsCallableStrictReturn
        : public std::integral_constant<
              bool,
              std::is_same_v<std::invoke_result_t<F, A...>, R>
                  && std::is_invocable_r_v<R, F, A...>>
    {
    };
};

} // namespace details


template <class F, class Signature>
concept Callable
    = details::SignatureDecomposition<Signature>::template IsCallable<F>::value;

template <class F, class Signature>
concept StrictCallable = details::SignatureDecomposition<
    Signature>::template IsCallableStrictReturn<F>::value;


static_assert(Callable<std::less<int>, bool(int, int)>, "");
static_assert(!Callable<std::less<int>, void(int)>, "bad args");

static_assert(Callable<std::less<int>, void(int, int)>, "Convertible return type");
static_assert(!StrictCallable<std::less<int>, void(int, int)>, "bad return");


} // namespace simu
