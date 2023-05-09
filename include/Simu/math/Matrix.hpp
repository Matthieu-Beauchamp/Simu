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

template <class T, simu::Uint32 mRows_, simu::Uint32 nCols_>
struct MatrixBase
{
    typedef T value_type;
    static constexpr simu::Uint32 mRows = mRows_;
    static constexpr simu::Uint32 nCols = nCols_;

    typedef T * iterator;
    typedef const T * const_iterator;

    iterator
    begin()
    {
        return data[0];
    }
    iterator
    end()
    {
        return data[mRows * nCols - 1];
    }

    const_iterator
    begin() const
    {
        return data[0];
    }
    const_iterator
    end() const
    {
        return data[mRows * nCols - 1];
    }

    T data[mRows * nCols];
};

namespace details
{

template <class T, class U>
struct PromotedImpl
{
    typedef decltype(std::declval<T>() + std::declval<U>()) Type;
};

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
struct PromotedImpl<MatrixBase<T, m, n>, MatrixBase<U, m, n>>
{
    typedef MatrixBase<typename PromotedImpl<T, U>::Type, m, n> Type;
};

template <class T, class U>
using Promoted = typename PromotedImpl<T, U>::Type;

} // namespace details

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
MatrixBase<details::Promoted<T, U>, m, n>
operator+=(MatrixBase<T, m, n> lhs, MatrixBase<U, m, n> rhs)
{
    MatrixBase<details::Promoted<T, U>, m, n> res;
}







template <class T, simu::Uint32 dim>
using Vector = Matrix<T, dim, 1>;

template <class T, simu::Uint32 dim>
using RowVector = Matrix<T, 1, dim>;

} // namespace simu
