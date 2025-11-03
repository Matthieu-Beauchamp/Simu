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

namespace simu
{

namespace internal
{

template <class T, std::size_t i, class... Components>
struct element_index
{
    static_assert(i < sizeof...(Components), "T not in Components");

    static constexpr std::size_t value() {
        if constexpr (std::is_same_v<T, std::tuple_element_t<i, std::tuple<Components...>>>) {
            return i;
        } else {
            return element_index<T, i + 1, Components...>::value();
        }
    }
};

template <class T, class... Components>
constexpr std::size_t index_of() {
    return element_index<T, 0, Components...>::value();
}

template <class Tuple, class Func, std::size_t i = 0>
    requires std::invocable<Func, std::size_t, std::tuple_element_t<i, Tuple>&>
constexpr void forEach(Tuple& tuple, Func&& func) {
    func(i, std::get<i>(tuple));
    if constexpr (i + 1 < std::tuple_size_v<Tuple>)
        forEach<Tuple, Func, i + 1>(tuple, std::forward<Func>(func));
}

template <class T, class... Components>
struct element_of
{
};

template <class T, class First, class... Components>
struct element_of<T, First, Components...>
{
    static constexpr bool value = std::is_same_v<T, First>
                                  || element_of<T, Components...>::value;
};

template <class T, class Component>
struct element_of<T, Component>
{
    static constexpr bool value = std::is_same_v<T, Component>;
};

} // namespace internal

template <class T, class... Components>
concept element_of = internal::element_of<T, Components...>::value;

} // namespace simu
