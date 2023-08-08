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

#include "Simu/utility/View.hpp"

namespace simu
{
namespace details
{

#if !SIMU_HAS_STD_VIEW

template <std::forward_iterator Iter, std::invocable<decltype(*std::declval<Iter>())> Deref>
class TransformView<Iter, Deref>::Iterator
{
public:

    typedef std::remove_reference_t<DerefReturn>        value_type;
    typedef typename std::iterator_traits<Iter>::difference_type difference_type;

    typedef value_type& reference;
    typedef value_type* pointer;

    typedef std::forward_iterator_tag iterator_category;

    Iterator() = default; // semiregular requirements on sentinel..?
    Iterator(Iter it, Deref deref) : it_{it}, deref_{deref} {}

    reference operator*() const { return deref_(*it_); }
    pointer   operator->() const { return &deref_(*it_); }

    Iterator& operator++()
    {
        ++it_;
        return *this;
    }

    Iterator operator++(int) { return Iterator{it_++, deref_}; }

    bool operator==(const Iterator& other) const { return it_ == other.it_; }
    bool operator!=(const Iterator& other) const { return it_ != other.it_; }

private:

    Iter  it_;
    Deref deref_;
};

static_assert(std::input_iterator<TransformView<int*, Identity>::Iterator>, "");
static_assert(
    std::sentinel_for<TransformView<int*, Identity>::Iterator, TransformView<int*, Identity>::Iterator>,
    ""
);
static_assert(std::ranges::range<TransformView<int*, Identity>>, "");


template <std::forward_iterator Iter, std::invocable<decltype(*std::declval<Iter>())> Deref>
typename TransformView<Iter, Deref>::Iterator TransformView<Iter, Deref>::begin() const
{
    return Iterator{begin_, deref_};
}

template <std::forward_iterator Iter, std::invocable<decltype(*std::declval<Iter>())> Deref>
typename TransformView<Iter, Deref>::Iterator TransformView<Iter, Deref>::end() const
{
    return Iterator{end_, deref_};
}

template <std::forward_iterator Iter, std::invocable<decltype(*std::declval<Iter>())> Deref>
bool TransformView<Iter, Deref>::empty() const
{
    return begin() == end();
}

template <std::forward_iterator Iter, std::invocable<decltype(*std::declval<Iter>())> Deref>
std::size_t TransformView<Iter, Deref>::size() const
{
    return std::distance(begin(), end());
}

#endif

} // namespace details
} // namespace simu
