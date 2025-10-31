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

#include "Simu/ecs/Entity.hpp"
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simu
{

template <class T>
class SparseSet
{
private:

    using Data = std::vector<T>;
    using Ids  = std::vector<Entity>;

    // FIXME: use vector / custom hashmap to avoid allocations?
    using Sparse = std::unordered_map<Entity, std::size_t>;

    Data   data{};
    Ids    ids{};
    Sparse sparse{};

public:

    T&       get_data(std::size_t index) { return data[index]; }
    const T& get_data(std::size_t index) const { return data[index]; }

    T& get_data(const Entity& entity) {
        return data[sparse.find(entity)->second];
    }
    const T& get_data(const Entity& entity) const {
        return data[sparse.find(entity)->second];
    }

    std::optional<std::size_t> index_of(const Entity& entity) const {
        auto iter = sparse.find(entity);
        return iter == sparse.end() ? std::nullopt : std::optional(iter->second);
    }

    bool has_data(const Entity& entity) const {
        return sparse.contains(entity);
    }

    std::pair<Entity, std::reference_wrapper<T>> get_pair(std::size_t index) {
        return std::make_pair(ids[index], std::ref(data[index]));
    }
    std::pair<Entity, std::reference_wrapper<const T>>
    get_pair(std::size_t index) const {
        return std::make_pair(ids[index], std::cref(data[index]));
    }

    std::size_t size() const { return data.size(); }

    bool add(const Entity& entity, const T& value) {
        auto result = sparse.emplace(entity, data.size());

        if (result.second) {
            data.emplace_back(value);
            ids.emplace_back(entity);
        }

        return result.second;
    }

    bool remove(const Entity& entity) {
        auto iter = sparse.find(entity);
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
};

} // namespace simu
