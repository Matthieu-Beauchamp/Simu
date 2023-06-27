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

template <class T, Uint32 m, Uint32 n>
template <class U>
MatrixData<T, m, n>::MatrixData(const MatrixData<U, m, n>& other)
{
    for (Uint32 i = 0; i < size(); ++i)
        data[i] = other.data[i];
}

template <class T, Uint32 m, Uint32 n>
T& MatrixData<T, m, n>::operator()(Uint32 row, Uint32 col)
{
    return data[row * nCols + col];
}

template <class T, Uint32 m, Uint32 n>
const T& MatrixData<T, m, n>::operator()(Uint32 row, Uint32 col) const
{
    return data[row * nCols + col];
}

template <class T, Uint32 m, Uint32 n>
T& MatrixData<T, m, n>::operator[](Uint32 index)
{
    return data[index];
}

template <class T, Uint32 m, Uint32 n>
const T& MatrixData<T, m, n>::operator[](Uint32 index) const
{
    return data[index];
}


////////////////////////////////////////////////////////////
// Matrix
////////////////////////////////////////////////////////////

template <class T, Uint32 dim>
Matrix<T, dim, dim> SpecialConstructors<T, dim, dim, true>::identity()
{
    Matrix<T, dim, dim> ident{};
    for (Uint32 i = 0; i < dim; ++i)
        ident(i, i) = 1;

    return ident;
}

template <class T, Uint32 dim>
Matrix<T, dim, dim>
SpecialConstructors<T, dim, dim, true>::diagonal(const Vector<T, dim>& elements)
{
    Matrix<T, dim, dim> diag{};
    for (Uint32 i = 0; i < dim; ++i)
        diag(i, i) = elements[i];

    return diag;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::i()
{
    Vector<T, dim> vec{};
    if constexpr (0 < dim)
        vec[0] = 1;

    return vec;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::j()
{
    Vector<T, dim> vec{};
    if constexpr (1 < dim)
        vec[1] = 1;

    return vec;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::k()
{
    Vector<T, dim> vec{};
    if constexpr (2 < dim)
        vec[2] = 1;

    return vec;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::w()
{
    Vector<T, dim> vec{};
    if constexpr (3 < dim)
        vec[3] = 1;

    return vec;
}


template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>::Matrix(const Matrix<U, m, n>& other)
    : MatrixData<T, m, n>{static_cast<const MatrixData<U, m, n>&>(other)}
{
}


template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n>::Matrix(const MatrixData<T, m, n>& data)
    : MatrixData<T, m, n>{data}
{
}


template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n>::Matrix(const std::initializer_list<T>& init)
{
    auto it = this->begin();
    for (const T& val : init)
    {
        if (it == this->end())
            return;

        *it++ = val;
    }
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>
Matrix<T, m, n>::fromRows(const std::initializer_list<Vector<U, n>>& rows)
{
    return fromRows(Vector<Vector<U, n>, m>{rows});
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n> Matrix<T, m, n>::fromRows(const Vector<Vector<U, n>, m>& rows)
{
    Matrix mat{};

    auto it = mat.begin();
    for (const auto& row : rows)
        for (const auto& val : row)
            *it++ = val;

    return mat;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>
Matrix<T, m, n>::fromCols(const std::initializer_list<Vector<U, m>>& cols)
{
    return transpose(Matrix<T, n, m>::fromRows(cols));
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n> Matrix<T, m, n>::fromCols(const Vector<Vector<U, m>, n>& cols)
{
    return transpose(Matrix<T, n, m>::fromRows(cols));
}

template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n> Matrix<T, m, n>::filled(T val)
{
    Matrix<T, m, n> mat{};
    for (auto& elem : mat)
        elem = val;

    return mat;
}

template <class T, Uint32 m, Uint32 n>
Vector<Vector<T, n>, m> Matrix<T, m, n>::asRows() const
{
    Vector<Vector<T, n>, m> rows{};

    auto it = this->begin();
    for (Vector<T, n>& row : rows)
        for (T& val : row)
            val = *it++;

    return rows;
}

template <class T, Uint32 m, Uint32 n>
Vector<Vector<T, m>, n> Matrix<T, m, n>::asCols() const
{
    return transpose(*this).asRows();
}


template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n> Matrix<T, m, n>::operator+() const
{
    return *this;
}

template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n> Matrix<T, m, n>::operator-() const
{
    Matrix null{};
    return null -= *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator+=(const Matrix<U, m, n>& other)
{
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] += other.data[i];

    return *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator-=(const Matrix<U, m, n>& other)
{
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] -= other.data[i];

    return *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator*=(U scalar)
{
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] *= scalar;

    return *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator/=(U scalar)
{
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] /= scalar;

    return *this;
}


template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n>
operator+(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    Matrix<Promoted<T, U>, m, n> res{lhs};
    return res += rhs;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n>
operator-(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    return lhs + (-rhs);
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator*(U scalar, const Matrix<T, m, n>& mat)
{
    Matrix<Promoted<T, U>, m, n> res{mat};
    return res *= scalar;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator*(const Matrix<T, m, n>& mat, U scalar)
{
    return scalar * mat;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator/(const Matrix<T, m, n>& mat, U scalar)
{
    Matrix<Promoted<T, U>, m, n> res{mat};
    return res /= scalar;
}

template <class T, class U, Uint32 mLeft, Uint32 nLeft, Uint32 nRight>
Matrix<Promoted<T, U>, mLeft, nRight>
operator*(const Matrix<T, mLeft, nLeft>& lhs, const Matrix<U, nLeft, nRight>& rhs)
{
    Matrix<Promoted<T, U>, mLeft, nRight> res{};
    for (Uint32 row = 0; row < lhs.mRows; ++row)
    {
        for (Uint32 col = 0; col < rhs.nCols; ++col)
        {
            for (Uint32 k = 0; k < lhs.nCols; ++k)
            {
                res(row, col) += lhs(row, k) * rhs(k, col);
            }
        }
    }

    return res;
}


template <class T, Uint32 m, Uint32 n>
Matrix<T, n, m> transpose(const Matrix<T, m, n>& original)
{
    Matrix<T, n, m> res;
    for (Uint32 row = 0; row < m; ++row)
    {
        for (Uint32 col = 0; col < n; ++col)
        {
            res(col, row) = original(row, col);
        }
    }

    return res;
}


template <class T, class U, Uint32 n>
Vector<Promoted<T, U>, n> solve(const Matrix<T, n, n>& A, const Vector<U, n>& b)
{
    return Solver{A}.solve(b);
}

template <class T, Uint32 n>
Solver<T, n>::Solver(const Matrix<T, n, n>& A) : R_{}
{
    // modified Gram-Schmidt for QR decomposition
    // https://www.math.uci.edu/~ttrogdon/105A/html/Lecture23.html

    auto Q = A.asCols();

    for (Uint32 col = 0; col < n; ++col)
    {
        R_(col, col) = norm(Q[col]);

        isValid_ = isValid_ && R_(col, col) != 0.f;
        if (!isValid_)
            return;

        Q[col] /= R_(col, col);

        for (Uint32 nextCol = col + 1; nextCol < n; ++nextCol)
        {
            R_(col, nextCol) = dot(Q[col], Q[nextCol]);
            Q[nextCol] -= R_(col, nextCol) * Q[col];
        }
    }

    QT_ = transpose(Matrix<T, n, n>::fromCols(Q));
}

template <class T, Uint32 n>
template <class U>
Vector<Promoted<T, U>, n> Solver<T, n>::solve(const Vector<U, n>& b) const
{
    SIMU_ASSERT(
        isValid_,
        "Solver invalid, ensure the original matrix has full rank"
    );

    Vector<Promoted<T, U>, n> c = QT_ * b;
    Vector<Promoted<T, U>, n> x{};

    for (Uint32 row = n; row > 0; --row)
    {
        for (Uint32 col = row; col < n; ++col)
        {
            c[row - 1] -= R_(row - 1, col) * x[col];
        }

        x[row - 1] = c[row - 1] / R_(row - 1, row - 1);
    }

    return x;
}


template <class T, Uint32 n>
Matrix<T, n, n> invert(const Matrix<T, n, n>& mat)
{
    Solver<T, n> solver{mat};

    auto inverse = Matrix<T, n, n>::identity().asCols();
    for (auto& col : inverse)
        col = solver.solve(col);

    return Matrix<T, n, n>::fromCols(inverse);
}


template <class T, class U, Uint32 n, std::invocable<Vector<Promoted<T, U>, n>> Proj>
Vector<Promoted<T, U>, n> solveInequalities(
    const Matrix<T, n, n>&    A,
    Vector<U, n>              b,
    Proj                      proj,
    Vector<Promoted<T, U>, n> initialGuess,
    float                     epsilon
)
{
    b = -b; // Ax - b >= 0

    Vector<Promoted<T, U>, n> x = initialGuess;

    float eps = 1.f + epsilon;
    while (eps > epsilon)
    {
        eps = 0.f;
        for (Uint32 row = 0; row < n; ++row)
        {
            float delX = 0.f;

            for (Uint32 col = 0; col < row; ++col)
                delX += A(row, col) * x[col];

            for (Uint32 col = row + 1; col < n; ++col)
                delX += A(row, col) * x[col];

            delX = -(delX + b[row]) / A(row, row);

            Vector<Promoted<T, U>, n> newX{x};
            newX[row] = delX;
            delX      = proj(newX)[row];

            eps    = std::max(eps, std::abs(delX - x[row]));
            x[row] = delX;
        }
    }

    return x;
}

////////////////////////////////////////////////////////////
// Vector operations
////////////////////////////////////////////////////////////

template <class T, class U, Uint32 dim>
Promoted<T, U> dot(const Vector<T, dim>& lhs, const Vector<U, dim>& rhs)
{
    Promoted<T, U> sum{};
    for (Uint32 i = 0; i < dim; ++i)
        sum += lhs[i] * rhs[i];

    return sum;
}

template <class T, Uint32 dim>
T normSquared(const Vector<T, dim>& v)
{
    return dot(v, v);
}

template <class T, Uint32 dim>
T norm(const Vector<T, dim>& v)
{
    return std::sqrt(normSquared(v));
}

template <class T, Uint32 dim>
Vector<T, dim> normalized(const Vector<T, dim>& v)
{
    return v / norm(v);
}

template <class T>
Vector<T, 2> perp(const Vector<T, 2>& v, bool clockwise)
{
    return clockwise ? -perp(v, false) : Vector<T, 2>{-v[1], v[0]};
}

template <class T, class U>
Vector<Promoted<T, U>, 3> cross(const Vector<T, 3>& lhs, const Vector<U, 3>& rhs)
{
    return Vector<Promoted<T, U>, 3>{
        cross(Vector<T, 2>{lhs[1], lhs[2]}, Vector<U, 2>{rhs[1], rhs[2]}),
        -cross(Vector<T, 2>{lhs[0], lhs[2]}, Vector<U, 2>{rhs[0], rhs[2]}),
        cross(Vector<T, 2>{lhs[0], lhs[1]}, Vector<U, 2>{rhs[0], rhs[1]})};
}

template <class T, class U>
Promoted<T, U> cross(const Vector<T, 2>& lhs, const Vector<U, 2>& rhs)
{
    return lhs[0] * rhs[1] - lhs[1] * rhs[0];
}

template <class T, class U, Uint32 dim>
Vector<Promoted<T, U>, dim>
projection(const Vector<T, dim>& ofThis, const Vector<U, dim>& onThat)
{
    return onThat * dot(ofThis, onThat) / normSquared(onThat);
}


////////////////////////////////////////////////////////////
// Comparison facilities
////////////////////////////////////////////////////////////

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator==(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = lhs[i] == rhs[i];
    }

    return res;
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator!=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    return !(lhs == rhs);
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator<(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = lhs[i] < rhs[i];
    }

    return res;
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator<=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    return !(lhs > rhs);
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator>(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = lhs[i] > rhs[i];
    }

    return res;
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator>=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs)
{
    return !(lhs < rhs);
}

template <Uint32 m, Uint32 n>
bool all(const ComparisonMatrix<m, n>& comp)
{
    for (bool b : comp)
        if (!b)
            return false;

    return true;
}

template <Uint32 m, Uint32 n>
bool any(const ComparisonMatrix<m, n>& comp)
{
    for (bool b : comp)
        if (b)
            return true;

    return false;
}

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator&&(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = lhs[i] && rhs[i];
    }

    return res;
}

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator||(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs)
{
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = lhs[i] || rhs[i];
    }

    return res;
}

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n> operator!(const ComparisonMatrix<m, n>& unary)
{
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < res.size(); ++i)
    {
        res[i] = !unary[i];
    }

    return res;
}


} // namespace simu


////////////////////////////////////////////////////////////
// std overloads
////////////////////////////////////////////////////////////

namespace std
{

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n> abs(const simu::Matrix<T, m, n>& mat)
{
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < mat.size(); ++i)
    {
        res[i] = std::abs(mat[i]);
    }

    return res;
}

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n> round(const simu::Matrix<T, m, n>& mat)
{
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < mat.size(); ++i)
    {
        res[i] = std::round(mat[i]);
    }

    return res;
}

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n>
min(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs)
{
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = std::min(lhs[i], rhs[i]);
    }

    return res;
}

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n>
max(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs)
{
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i)
    {
        res[i] = std::max(lhs[i], rhs[i]);
    }

    return res;
}

} // namespace std
