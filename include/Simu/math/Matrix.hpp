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

#include <initializer_list>

#include "Simu/config.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief Matrix data is stored row-major
///
////////////////////////////////////////////////////////////
template <class T, simu::Uint32 mRows_, simu::Uint32 nCols_>
struct MatrixData
{
    typedef T        value_type;
    typedef T*       iterator;
    typedef const T* const_iterator;

    static constexpr simu::Uint32 mRows = mRows_;
    static constexpr simu::Uint32 nCols = nCols_;

    MatrixData() = default;

    template <class U>
    explicit MatrixData(const MatrixData<U, mRows_, nCols_>& other);

    simu::Uint32 size() const { return mRows * nCols; }

    T&       operator()(simu::Uint32 row, simu::Uint32 col);
    const T& operator()(simu::Uint32 row, simu::Uint32 col) const;

    T&       operator[](simu::Uint32 index);
    const T& operator[](simu::Uint32 index) const;


    iterator begin() { return data; }
    iterator end() { return data + (mRows * nCols - 1); }

    const_iterator begin() const { return data; }
    const_iterator end() const { return data + (mRows * nCols - 1); }

    T data[mRows * nCols];
};


template <class T, simu::Uint32 m, simu::Uint32 n>
struct Matrix : public MatrixData<T, m, n>
{
    typedef Matrix<T, 1, n> Row;
    typedef Matrix<T, m, 1> Col;

    Matrix() = default;

    template <class U>
    explicit Matrix(const Matrix<U, m, n>& other)
        : MatrixData{static_cast<const MatrixData<U, m, n>&>(other)}
    {
    }

    Matrix(const MatrixData<T, m, n>& data) : MatrixData{data} {}

    Matrix(const std::initializer_list<T>& init)
    {
        auto it = begin();
        for (const T& val : init)
        {
            if (it == end())
                return;

            *it++ = val;
        }
    }


    Matrix operator+() const { return *this; }

    Matrix operator-() const
    {
        Matrix null{};
        return null -= *this;
    }


    template <class U>
    Matrix& operator+=(const Matrix<U, m, n>& other)
    {
        for (simu::Uint32 i = 0; i < size(); ++i) { data[i] += other.data[i]; }
        return *this;
    }

    template <class U>
    Matrix& operator-=(const Matrix<U, m, n>& other)
    {
        for (simu::Uint32 i = 0; i < size(); ++i) { data[i] -= other.data[i]; }
        return *this;
    }

    template <class U>
    Matrix& operator*=(U scalar)
    {
        for (simu::Uint32 i = 0; i < size(); ++i) { data[i] *= scalar; }
        return *this;
    }

    template <class U>
    Matrix& operator/=(U scalar)
    {
        for (simu::Uint32 i = 0; i < size(); ++i) { data[i] /= scalar; }
        return *this;
    }
};


namespace details
{

template <class T, class U>
struct PromotedImpl
{
    typedef decltype(std::declval<T>() + std::declval<U>()) Type;
};

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
struct PromotedImpl<MatrixData<T, m, n>, MatrixData<U, m, n>>
{
    typedef MatrixData<typename PromotedImpl<T, U>::Type, m, n> Type;
};

template <class T, class U>
using Promoted = typename PromotedImpl<T, U>::Type;

} // namespace details


template <class T, class U, simu::Uint32 m, simu::Uint32 n>
Matrix<details::Promoted<T, U>, m, n>
operator+(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    Matrix<details::Promoted<T, U>, m, n> res{lhs};
    return res += rhs;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
Matrix<details::Promoted<T, U>, m, n>
operator-(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    Matrix<details::Promoted<T, U>, m, n> res{lhs};
    return res -= rhs;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
Matrix<details::Promoted<T, U>, m, n>
operator*(U scalar, const Matrix<T, m, n>& rhs)
{
    Matrix<details::Promoted<T, U>, m, n> res{rhs};
    return res *= scalar;
}

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
Matrix<details::Promoted<T, U>, m, n>
operator/(U scalar, const Matrix<T, m, n>& rhs)
{
    Matrix<details::Promoted<T, U>, m, n> res{rhs};
    return res /= scalar;
}

template <class T, class U, simu::Uint32 mLeft, simu::Uint32 nLeft, simu::Uint32 nRight>
Matrix<details::Promoted<T, U>, mLeft, nRight>
operator*(const Matrix<T, mLeft, nLeft>& lhs, const Matrix<U, nLeft, nRight>& rhs)
{
    Matrix<details::Promoted<T, U>, mLeft, nRight> res{};
    for (simu::Uint32 row = 0; row < lhs.mRows; ++row)
    {
        for (simu::Uint32 col = 0; col < rhs.nCols; ++col)
        {
            for (simu::Uint32 k = 0; k < lhs.nCols; ++k)
            {
                res(row, col) += lhs(row, k) * rhs(k, col);
            }
        }
    }
}

template <class T, simu::Uint32 m, simu::Uint32 n>
Matrix<T, n, m> transpose(const Matrix<T, m, n>& original)
{
    Matrix<T, n, m> res;
    for (simu::Uint32 row = 0; row < m; ++row)
    {
        for (simu::Uint32 col = 0; col < n; ++col)
        {
            res(row, col) = original(col, row);
        }
    }
}


template <class T, simu::Uint32 dim>
using Vector = Matrix<T, dim, 1>;

template <class T, simu::Uint32 dim>
using RowVector = Matrix<T, 1, dim>;


typedef Matrix<float, 2, 2> Mat22;
typedef Matrix<float, 3, 3> Mat33;
typedef Matrix<float, 4, 4> Mat44;


////////////////////////////////////////////////////////////
// Comparison facilities
////////////////////////////////////////////////////////////

template <simu::Uint32 m, simu::Uint32 n>
using ComparisonMatrix = MatrixData<bool, m, n>;

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator==(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator!=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator<(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator<=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator>(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator>=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);



template <simu::Uint32 m, simu::Uint32 n>
bool all(const ComparisonMatrix<m, n>& comp);

template <simu::Uint32 m, simu::Uint32 n>
bool any(const ComparisonMatrix<m, n>& comp);

template <simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator&&(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs);

template <simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator||(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs);

template <simu::Uint32 m, simu::Uint32 n>
ComparisonMatrix<m, n>
operator!(const ComparisonMatrix<m, n>& unary);



} // namespace simu


#include "Simu/math/Matrix.inl.hpp"
