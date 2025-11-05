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


#include "Simu/entities/ComponentQuery.hpp"
#include "Simu/entities/Entity.hpp"
#include "Simu/entities/JoinQuery.hpp"
#include "Simu/entities/SparseSet.hpp"
#include <memory>
#include <tuple>
#include <type_traits>

namespace simu
{


template <simple_type... Components>
    requires all_different<Components...>
class Entities
{
    template <class Component>
    using SetType = SparseSet<Component>;

    using Storage = std::tuple<SetType<Components>...>;


    Storage                   storage;
    internal::EntityGenerator generator;

public:

    Entity create() { return generator.create(); };

    template <element_of<Components...> T>
    bool add(const Entity& entity, const T& value) {
        return get_component<T>().add(entity, value);
    }

    template <element_of<Components...> T>
    bool remove(const Entity& entity) {
        return get_component<T>().remove(entity);
    }

    void destroy(const Entity& entity) {
        (get_component<Components>().remove(entity), ...);
    }

    template <element_of<Components...> T>
    auto query() {
        return ComponentQuery<T, false>(get_component<T>());
    }

    template <element_of<Components...> T>
    auto query() const {
        return ComponentQuery<T, true>(get_component<T>());
    }

    template <element_of<Components...>... Ts, std::enable_if_t<sizeof...(Ts) >= 2, bool> = false>
    auto query() {
        return JoinQuery<false, Ts...>(
            std::tuple(std::addressof(get_component<Ts>())...)
        );
    }

    template <element_of<Components...>... Ts, std::enable_if_t<sizeof...(Ts) >= 2, bool> = false>
    auto query() const {
        return JoinQuery<true, Ts...>(
            std::tuple(std::addressof(get_component<Ts>())...)
        );
    }

    template <element_of<Components...> T>
    SetType<T>& get_component() {
        return std::get<SetType<T>>(storage);
    }

    template <element_of<Components...> T>
    const SetType<T>& get_component() const {
        return std::get<SetType<T>>(storage);
    }
};

} // namespace simu
