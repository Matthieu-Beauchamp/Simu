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

#include "Simu/config.hpp"

namespace simu
{

// https://en.cppreference.com/w/cpp/ranges/transform_view/transform_view

struct BasicDereference
{
    template <class Iter>
    auto& dereference(Iter it)
    {
        return *it;
    }

    template <class Iter>
    auto arrow(Iter it)
    {
        return it;
    }
};

struct BypassSmartPointer
{
    template <class Iter>
    auto& dereference(Iter it)
    {
        return **it;
    }

    template <class Iter>
    auto arrow(Iter it)
    {
        return it.operator->();
    }
};

template <class Iter, class Deref = BasicDereference>
class Range
{
public:

    Range(Iter begin, Iter end, Deref deref = Deref{})
        : begin_{begin}, end_{end}, deref_{deref}
    {
    }

    class Iterator;
    Iterator begin() const;
    Iterator end() const;

    class Iterator
    {
    public:

        Iterator(Iter it, Deref deref) : it{it}, deref_{deref} {}

        Iterator& operator++()
        {
            ++it;
            return *this;
        }

        Iterator operator++(int)
        {
            Iterator copy{*this};
            ++*this;
            return copy;
        }

        bool operator==(const Iterator& other) const { return it == other.it; }

        bool operator!=(const Iterator& other) const
        {
            return !(*this == other);
        }

        auto& operator*() { return deref_.dereference(it); }
        auto  operator->() { return deref_.arrow(it); }

    private:

        Iter  it;
        Deref deref_;
    };

private:

    Iter  begin_;
    Iter  end_;
    Deref deref_;
};

template <class Iter, class Deref>
Range<Iter, Deref>::Iterator Range<Iter, Deref>::begin() const
{
    return Iterator{begin_, deref_};
}

template <class Iter, class Deref>
Range<Iter, Deref>::Iterator Range<Iter, Deref>::end() const
{
    return Iterator{end_, deref_};
}

template <class Iter, class Deref = BasicDereference>
Range<Iter, Deref> makeRange(Iter begin, Iter end, Deref deref = Deref{})
{
    return Range<Iter, Deref>{begin, end, deref};
}

} // namespace simu
