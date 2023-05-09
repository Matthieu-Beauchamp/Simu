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

#include "Simu/math/Matrix.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
// Matrix Data
////////////////////////////////////////////////////////////

template <class T, simu::Uint32 m, simu::Uint32 n>
template <class U>
MatrixData<T, m, n>::MatrixData(const MatrixData<U, m, n>& other)
{
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) data[i] = other.data[i];
}

template <class T, simu::Uint32 m, simu::Uint32 n>
T& MatrixData<T, m, n>::operator()(simu::Uint32 row, simu::Uint32 col)
{
    return data[row * nCols + col];
}

template <class T, simu::Uint32 m, simu::Uint32 n>
const T&
MatrixData<T, m, n>::operator()(simu::Uint32 row, simu::Uint32 col) const
{
    return data[row * nCols + col];
}

template <class T, simu::Uint32 m, simu::Uint32 n>
T& MatrixData<T, m, n>::operator[](simu::Uint32 index)
{
    return data[index];
}

template <class T, simu::Uint32 m, simu::Uint32 n>
const T& MatrixData<T, m, n>::operator[](simu::Uint32 index) const
{
    return data[index];
}


////////////////////////////////////////////////////////////
// Comparison facilities
////////////////////////////////////////////////////////////

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator==(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] == rhs[i]; }

    return res;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator!=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] != rhs[i]; }

    return res;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator<(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] < rhs[i]; }

    return res;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator<=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] <= rhs[i]; }

    return res;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator>(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] > rhs[i]; }

    return res;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator>=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] >= rhs[i]; }

    return res;
}

template <simu::Uint32 m, simu::Uint32 n>
bool all(const ComparisonMatrix<m, n>& comp)
{
    for (bool b : comp)
        if (!b)
            return false;

    return true;
}

template <simu::Uint32 m, simu::Uint32 n>
bool any(const ComparisonMatrix<m, n>& comp)
{
    for (bool b : comp)
        if (b)
            return true;

    return false;
}

template <simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator&&(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] && rhs[i]; }

    return res;
}

template <simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator||(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs){
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = lhs[i] || rhs[i]; }

    return res;
}

template <simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n> operator!(const ComparisonMatrix<m, n>& unary){
    ComparisonMatrix<m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) { res[i] = !unary[i]; }

    return res;
}


} // namespace simu
