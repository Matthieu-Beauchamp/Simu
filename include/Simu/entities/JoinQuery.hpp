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

#include "Simu/entities/utils.hpp"
#include "Simu/entities/Entity.hpp"
#include "Simu/entities/SparseSet.hpp"
#include <cstddef>
#include <functional>
#include <limits>
#include <tuple>
#include <type_traits>
#include <array>

namespace simu
{

namespace internal
{

template <bool is_const, class... Components>
class JoinIterator
{
    static constexpr std::size_t N = sizeof...(Components);

    template <class T>
    using SparseSetType = std::conditional_t<is_const, const SparseSet<T>, SparseSet<T>>;

    using SetType = std::conditional_t<is_const, const BasicSparseSet, BasicSparseSet>;

    using Sets = std::array<SetType*, N>;

    template <class T>
    using DataType = std::conditional_t<is_const, const T, T>;

    template <class T>
    static constexpr std::size_t index() {
        return index_of<T, Components...>();
    }

public:

    using difference_type = std::ptrdiff_t;
    using reference_type = std::tuple<std::reference_wrapper<DataType<Components>>...>;

    JoinIterator(std::array<SetType*, N> sets, std::size_t ref_set, std::size_t index = 0)
        : _sets(sets), _ref_set(ref_set), _index(index) {
        if (_index < _sets[ref_set]->size() && !has_all_components()) {
            operator++();
        }
    }

    JoinIterator(const JoinIterator&)            = default;
    JoinIterator& operator=(const JoinIterator&) = default;

    reference_type operator*() const {
        Entity entity = get_entity();
        return std::tuple(
            std::ref(
                static_cast<SparseSetType<Components>*>(_sets[index<Components>()])
                    ->get_data(entity)
            )...
        );
    }

    [[nodiscard]] const Entity& get_entity() const {
        return _sets[_ref_set]->get_entity(_index);
    }

    JoinIterator& operator++() {
        while (++_index < _sets[_ref_set]->size() && !has_all_components()) {}
        return *this;
    }

    JoinIterator operator++(int) {
        auto tmp = *this;
        ++*this;
        return tmp;
    }

    auto operator<=>(const JoinIterator& other) const {
        return this->_index <=> other._index;
    }
    bool operator==(const JoinIterator&) const = default;

private:

    [[nodiscard]] bool has_all_components() const {
        Entity entity = _sets[_ref_set]->get_entity(_index);

        // TODO: Skip ref set if faster
        for (std::size_t i = 0; i < N; i++) {
            if (!_sets[i]->has_entity(entity)) {
                return false;
            }
        }

        return true;
    }


    Sets        _sets{};
    std::size_t _ref_set{};
    std::size_t _index{};
};

} // namespace internal

template <bool is_const, class... Components>
class JoinQuery
{
    static constexpr std::size_t N = sizeof...(Components);

    template <class T>
    using SetType = std::conditional_t<is_const, const SparseSet<T>, SparseSet<T>>;
    using BasicSetType
        = std::conditional_t<is_const, const internal::BasicSparseSet, internal::BasicSparseSet>;

    using Sets = std::array<BasicSetType*, N>;

    template <class T>
    using DataType = std::conditional_t<is_const, const T, T>;

    template <class T>
    static constexpr std::size_t index() {
        return internal::index_of<T, Components...>();
    }

public:

    explicit JoinQuery(std::tuple<SetType<Components>*...> sets)
        : _sets(), _ref_set(0) {
        std::size_t min_size = std::numeric_limits<std::size_t>::max();

        forEach(sets, [this, &min_size](std::size_t i, auto* set) {
            _sets[i] = static_cast<internal::BasicSparseSet*>(set);

            if (set->size() < min_size) {
                min_size = set->size();
                _ref_set = i;
            }
        });
    }

    template <std::invocable<DataType<Components>&...> F>
    void each(F&& f) const {
        for (auto it = begin(); it != end(); it++) {
            auto tuple = *it;
            f(std::get<std::reference_wrapper<DataType<Components>>>(tuple)...);
        }
    }

    template <std::invocable<Entity, DataType<Components>&...> F>
    void each(F&& f) const {
        for (auto it = begin(); it != end(); it++) {
            auto tuple = *it;
            f(it.get_entity(),
              std::get<std::reference_wrapper<DataType<Components>>>(tuple)...);
        }
    }

    auto begin() const {
        return internal::JoinIterator<is_const, Components...>(_sets, _ref_set, 0);
    }

    auto end() const {
        return internal::JoinIterator<is_const, Components...>(
            _sets, _ref_set, _sets[_ref_set]->size()
        );
    }

private:

    Sets        _sets{};
    std::size_t _ref_set{};
};

} // namespace simu
