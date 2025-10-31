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


#include "Simu/ecs/SparseSet.hpp"
#include <cstddef>
#include <memory>
#include <type_traits>

namespace simu
{

template <class T, bool is_const>
class ComponentIterator
{
    using SetType = std::conditional_t<is_const, const SparseSet<T>, SparseSet<T>>;

public:


    using difference_type = std::ptrdiff_t;
    using value_type      = std::conditional_t<is_const, const T, T>;
    using reference_type  = value_type&;

    ComponentIterator(SetType& set, std::size_t index = 0)
        : ComponentIterator(std::addressof(set), index) {}

    ComponentIterator(SetType* set, std::size_t index = 0)
        : _set(set), _index(index) {};

    ComponentIterator(const ComponentIterator&)            = default;
    ComponentIterator& operator=(const ComponentIterator&) = default;

    reference_type operator*() const { return _set->get_data(_index); }

    ComponentIterator& operator++() {
        _index++;
        return *this;
    }

    ComponentIterator operator++(int) {
        auto tmp = *this;
        ++*this;
        return tmp;
    }

    bool operator==(const ComponentIterator&) const = default;

private:

    SetType*    _set;
    std::size_t _index;
};

} // namespace simu
