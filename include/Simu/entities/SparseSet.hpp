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
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

#pragma once

#include "Simu/entities/ObjectId.hpp"
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simu
{

namespace internal
{

class BasicSparseSet
{
protected:

    using Ids = std::vector<ObjectId>;

    // FIXME: use vector / custom hashmap to avoid allocations?
    using Sparse = std::unordered_map<ObjectId, std::size_t>;

    Ids    ids{};
    Sparse sparse{};

public:

    const ObjectId& get_entity(std::size_t index) const { return ids[index]; }

    bool has_entity(const ObjectId& entity) const {
        return sparse.contains(entity);
    }

    std::optional<std::size_t> index_of(const ObjectId& entity) const {
        auto iter = sparse.find(entity);
        return iter == sparse.end() ? std::nullopt : std::optional(iter->second);
    }

    std::size_t size() const { return ids.size(); }
};

} // namespace internal

template <class T>
class SparseSet : public internal::BasicSparseSet
{
    using Data = std::vector<T>;

    Data data{};

public:

    template <bool is_const>
    class Iterator;

    T&       get_data(std::size_t index) { return data[index]; }
    const T& get_data(std::size_t index) const { return data[index]; }

    T& get_data(const ObjectId& entity) {
        return data[sparse.find(entity)->second];
    }
    const T& get_data(const ObjectId& entity) const {
        return data[sparse.find(entity)->second];
    }

    std::pair<ObjectId, std::reference_wrapper<T>> get_pair(std::size_t index) {
        return std::make_pair(ids[index], std::ref(data[index]));
    }
    std::pair<ObjectId, std::reference_wrapper<const T>>
    get_pair(std::size_t index) const {
        return std::make_pair(ids[index], std::cref(data[index]));
    }

    bool add(const ObjectId& entity, const T& value) {
        auto result = sparse.emplace(entity, data.size());

        if (result.second) {
            data.emplace_back(value);
            ids.emplace_back(entity);
        }

        return result.second;
    }

    bool remove(const ObjectId& entity) {
        const auto iter = sparse.find(entity);
        if (iter == sparse.end()) {
            return false;
        }

        std::size_t index = iter->second;
        sparse.erase(iter);

        if (index == data.size() - 1) {
            data.pop_back();
            ids.pop_back();
        } else {
            std::swap(data[index], data.back());
            data.pop_back();

            std::swap(ids[index], ids.back());
            ids.pop_back();

            sparse[ids[index]] = index;
        }

        return true;
    }

    auto begin() { return Iterator<false>(this, 0); }
    auto end() { return Iterator<false>(this, size()); }
    auto begin() const { return Iterator<true>(this, 0); }
    auto end() const { return Iterator<true>(this, size()); }
};

template <class T>
template <bool is_const>
class SparseSet<T>::Iterator
{
    using SetType = std::conditional_t<is_const, const SparseSet<T>, SparseSet<T>>;

public:

    using difference_type = std::ptrdiff_t;
    using value_type      = std::conditional_t<is_const, const T, T>;
    using reference_type  = value_type&;

    Iterator() : Iterator(nullptr, 0) {};

    explicit Iterator(SetType& set, std::size_t index = 0)
        : Iterator(std::addressof(set), index) {}

    explicit Iterator(SetType* set, std::size_t index = 0)
        : _set(set), _index(index) {};

    Iterator(const Iterator&)            = default;
    Iterator& operator=(const Iterator&) = default;

    [[nodiscard]] reference_type operator*() const {
        return _set->get_data(_index);
    }

    [[nodiscard]] const ObjectId& get_entity() const {
        return _set->get_entity(_index);
    }

    Iterator& operator++() {
        _index++;
        return *this;
    }

    Iterator operator++(int) {
        auto tmp = *this;
        ++*this;
        return tmp;
    }

    Iterator& operator--() {
        _index--;
        return *this;
    }

    Iterator operator--(int) {
        auto tmp = *this;
        --*this;
        return tmp;
    }

    Iterator& operator+=(std::ptrdiff_t n) {
        _index += n;
        return *this;
    }

    Iterator& operator-=(std::ptrdiff_t n) {
        _index -= n;
        return *this;
    }

    Iterator operator-(std::ptrdiff_t n) const {
        return Iterator(_set, _index - n);
    }

    difference_type operator-(const Iterator& it) const {
        return _index - it._index;
    }

    Iterator operator+(std::ptrdiff_t n) const {
        return Iterator(_set, _index + n);
    }

    friend Iterator operator+(std::ptrdiff_t n, const Iterator& it) {
        return it + n;
    }

    reference_type operator[](std::ptrdiff_t n) const {
        return _set->get_data(_index + n);
    }

    auto operator<=>(const Iterator& other) const {
        return this->_index <=> other._index;
    }
    bool operator==(const Iterator&) const = default;

private:

    SetType*    _set;
    std::size_t _index;
};

} // namespace simu
