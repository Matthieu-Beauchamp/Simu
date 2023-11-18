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


#if !defined(__clang__) || (__clang_major__ >= 16)
#    define SIMU_HAS_STD_VIEW 1
#    include <ranges>
#else
#    define SIMU_HAS_STD_VIEW 0
#    include <iterator>
#endif


namespace simu
{

#if SIMU_HAS_STD_VIEW
template <class R>
concept Range = std::ranges::range<R>;
#else
template <class R>
concept Range = true;
#endif

namespace details
{


#if !SIMU_HAS_STD_VIEW

template <std::forward_iterator Iter>
class View
{
public:

    View(Iter begin, Iter end) : begin_{begin}, end_{end} {}

    Iter begin() const { return begin_; }
    Iter end() const { return end_; }

    bool        empty() const { return begin() == end(); }
    std::size_t size() const { return std::distance(begin(), end()); }

private:

    Iter begin_;
    Iter end_;
};

////////////////////////////////////////////////////////////
/// \brief Lightweight iterable sequence constructed from a pair of iterators
///
/// The Deref parameter can specialise how the iterator is dereferenced,
///     but unlike std::views::transform, cannot create new objects.
///
/// The typical use is to hide the usage of smart pointers, instead giving access only
///     to raw references/pointers.
///
/// This class covers the needs of Simu for views instead of using the standard library
///     since Clang 15 does not support them well yet (Should be fixed in Clang 16):
///     https://github.com/llvm/llvm-project/issues/44178
///     https://github.com/llvm/llvm-project/issues/59697
///
///
/// \see simu::makeView
////////////////////////////////////////////////////////////
template <std::forward_iterator Iter, std::invocable<decltype(*std::declval<Iter>())> Deref>
class TransformView
{
    typedef decltype(std::declval<Deref>()(
        std::declval<typename std::iterator_traits<Iter>::reference>()
    )) DerefReturn;

    static_assert(
        std::is_reference_v<DerefReturn>,
        "Deref may not create new objets, only redirect on dereference"
    );

public:

    TransformView(Iter begin, Iter end, Deref deref = Deref{})
        : begin_{begin}, end_{end}, deref_{deref}
    {
    }

    class Iterator;

    Iterator begin() const;
    Iterator end() const;

    bool        empty() const;
    std::size_t size() const;

private:

    Iter  begin_;
    Iter  end_;
    Deref deref_;
};

#endif

} // namespace details


////////////////////////////////////////////////////////////
/// \brief Dereferences the element one more time
///
/// Propagates const, suppose a sequence of int*const *
/// then a simple dereference returns int*const &
/// and doubleDereference returns const int&
////////////////////////////////////////////////////////////
struct DoubleDereference
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


////////////////////////////////////////////////////////////
/// \brief Creates a view of [begin, end) and applies Deref(*Iter) to every dereferenced element
///
/// Deref must return a reference, ie it should not create new objects.
////////////////////////////////////////////////////////////
template <std::forward_iterator Iter, class Deref>
auto makeView(Iter begin, Iter end, Deref deref)
{
    typedef decltype(std::declval<Deref>()(
        std::declval<typename std::iterator_traits<Iter>::reference>()
    )) DerefReturn;

    static_assert(
        std::is_reference_v<DerefReturn>,
        "Deref may not create new objets, only redirect on dereference"
    );

#if SIMU_HAS_STD_VIEW
    return std::ranges::subrange<Iter, Iter>{begin, end}
           | std::views::transform(deref);
#else
    return details::TransformView<Iter, Deref>{begin, end, deref};
#endif
}

template <std::forward_iterator Iter>
auto makeView(Iter begin, Iter end)
{
#if SIMU_HAS_STD_VIEW
    return std::ranges::subrange<Iter, Iter>{begin, end};
#else
    return details::View<Iter>{begin, end};
#endif
}


////////////////////////////////////////////////////////////
/// \brief Creates a view of the range and applies Deref to every dereferenced element
///
////////////////////////////////////////////////////////////
template <Range R, class Deref>
auto makeView(R& range, Deref deref)
{
    return makeView(range.begin(), range.end(), deref);
}

template <Range R>
auto makeView(R& range)
{
    return makeView(range.begin(), range.end());
}


template <class Iter>
using ViewType = decltype(simu::makeView(std::declval<Iter>(), std::declval<Iter>()));

} // namespace simu

#include "Simu/utility/View.inl.hpp"
