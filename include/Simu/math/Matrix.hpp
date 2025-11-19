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
#include <type_traits>
#include <cmath>
#include <optional>

#include "Simu/config.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \ingroup math
/// \defgroup LinearAlgebra
////////////////////////////////////////////////////////////

template <class, Uint32, Uint32>
struct Matrix;

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \brief Vector class
///
/// Vectors are always column vectors,
///     use transpose(vec) to obtain a row vector when needed.
////////////////////////////////////////////////////////////
template <class T, Uint32 dim>
using Vector = Matrix<T, dim, 1>;

template <class T, Uint32 m, Uint32 n, bool isSquare = (m == n)>
struct SpecialConstructors
{
};

template <class T, Uint32 dim>
struct SpecialConstructors<T, dim, dim, true>
{
    static inline Matrix<T, dim, dim> identity();
    static inline Matrix<T, dim, dim> diagonal(const Vector<T, dim>& elements);
};

template <class T, Uint32 dim>
struct SpecialConstructors<T, dim, 1, false>
{
    static inline Vector<T, dim> i();
    static inline Vector<T, dim> j();
    static inline Vector<T, dim> k();
    static inline Vector<T, dim> w();
};

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \brief Matrix class for small matrices
///
/// Most operations are provided as global functions
/// \see operations
////////////////////////////////////////////////////////////
template <class T, Uint32 m, Uint32 n>
struct Matrix : public SpecialConstructors<T, m, n>
{
    typedef T        value_type;
    typedef T*       iterator;
    typedef const T* const_iterator;

    static constexpr Uint32 mRows = m;
    static constexpr Uint32 nCols = n;

    Matrix() = default;

    template <class U>
    explicit inline Matrix(const Matrix<U, m, n>& other);

    explicit inline Matrix(const std::initializer_list<T>& init);

    template <std::convertible_to<T>... Args, std::enable_if_t<(sizeof...(Args) == n * m), int> = 0>
    explicit inline Matrix(Args... values)
        : Matrix<T, m, n>{static_cast<T>(values)...} {}

    static inline Matrix filled(T val);

    template <class U>
    static inline Matrix fromRows(const std::initializer_list<Vector<U, n>>& rows);

    template <class U>
    static inline Matrix fromRows(const Vector<Vector<U, n>, m>& rows);

    template <class U>
    static inline Matrix fromCols(const std::initializer_list<Vector<U, m>>& cols);

    template <class U>
    static inline Matrix fromCols(const Vector<Vector<U, m>, n>& cols);

    inline Vector<Vector<T, n>, m> asRows() const;
    inline Vector<Vector<T, m>, n> asCols() const;

    constexpr static Uint32 size() { return mRows * nCols; }

    inline T&       operator()(Uint32 row, Uint32 col);
    inline const T& operator()(Uint32 row, Uint32 col) const;

    inline T&       operator[](Uint32 index);
    inline const T& operator[](Uint32 index) const;

    iterator begin() { return data; }
    iterator end() { return data + size(); }

    const_iterator begin() const { return data; }
    const_iterator end() const { return data + size(); }

    inline Matrix operator+() const;
    inline Matrix operator-() const;

    template <class U>
    inline Matrix& operator+=(const Matrix<U, m, n>& other);

    template <class U>
    inline Matrix& operator-=(const Matrix<U, m, n>& other);

    template <class U>
    inline Matrix& operator*=(U scalar);

    template <class U>
    inline Matrix& operator/=(U scalar);

    T data[mRows * nCols];
};

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \defgroup operations
/// \{
////////////////////////////////////////////////////////////

template <class T, class U>
using Promoted = typename std::common_type<T, U>::type;


template <class T, class U, Uint32 m, Uint32 n>
inline Matrix<Promoted<T, U>, m, n>
operator+(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline Matrix<Promoted<T, U>, m, n>
operator-(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline Matrix<Promoted<T, U>, m, n> operator*(U scalar, const Matrix<T, m, n>& mat);

template <class T, class U, Uint32 m, Uint32 n>
inline Matrix<Promoted<T, U>, m, n> operator*(const Matrix<T, m, n>& mat, U scalar);

template <class T, class U, Uint32 m, Uint32 n>
inline Matrix<Promoted<T, U>, m, n> operator/(const Matrix<T, m, n>& mat, U scalar);

template <class T, class U, Uint32 mLeft, Uint32 nLeft, Uint32 nRight>
inline Matrix<Promoted<T, U>, mLeft, nRight>
operator*(const Matrix<T, mLeft, nLeft>& lhs, const Matrix<U, nLeft, nRight>& rhs);


template <class T, Uint32 m, Uint32 n>
inline Matrix<T, n, m> transpose(const Matrix<T, m, n>& original);


////////////////////////////////////////////////////////////
/// \brief Return x such that Ax = b
///
////////////////////////////////////////////////////////////
template <class T, class U, Uint32 n>
inline Vector<Promoted<T, U>, n>
solve(const Matrix<T, n, n>& A, const Vector<U, n>& b);

template <class T, Uint32 n>
class Solver
{
public:

    inline Solver(const Matrix<T, n, n>& A);

    template <class U>
    inline Vector<Promoted<T, U>, n> solve(const Vector<U, n>& b) const;

    Matrix<T, n, n> original() const { return transpose(QT_) * R_; }

    bool isValid() const { return isValid_; }

private:

    Matrix<T, n, n> QT_;
    Matrix<T, n, n> R_;
    bool            isValid_ = true;
};


template <class T, Uint32 n>
inline Matrix<T, n, n> invert(const Matrix<T, n, n>& mat);


////////////////////////////////////////////////////////////
/// \brief Return x such that Ax >= b with bounds on x
///
/// proj must project x onto its valid bounds (ie clamp its values). With the following signature:
///     T proj(const Vector<T, n>& x, Uint32 index)
/// where x[index] must be clamped and returned.
///
/// when the absolute change of components of x drops below epsilon, iteration terminates.
///
/// Uses projected Gauss-Seidel to solve the MLCP,
/// See A. Enzenhofer's master thesis (McGill): Numerical Solutions of MLCP
////////////////////////////////////////////////////////////
template <class T, Uint32 n, std::invocable<Vector<T, n>, Uint32> Proj>
inline Vector<T, n> solveInequalities(
    const Matrix<T, n, n>& A,
    Vector<T, n>           b,
    Proj                   proj,
    Vector<T, n>           initialGuess = Vector<T, n>{},
    float                  epsilon      = simu::EPSILON
);

////////////////////////////////////////////////////////////
/// \brief Return x such that Ax >= b with bounds on x
///
/// proj must project x onto its valid bounds (ie clamp its values).
///
/// when the absolute change of components of x drops below epsilon, iteration terminates.
///
/// Uses projected Gauss-Seidel to solve the MLCP,
/// See A. Enzenhofer's master thesis (McGill): Numerical Solutions of MLCP
////////////////////////////////////////////////////////////
template <class T, Uint32 n, std::invocable<Vector<T, n>> Proj>
inline Vector<T, n> solveInequalities(
    const Matrix<T, n, n>& A,
    Vector<T, n>           b,
    Proj                   proj,
    Vector<T, n>           initialGuess = Vector<T, n>{},
    float                  epsilon      = simu::EPSILON
);


////////////////////////////////////////////////////////////
/// \brief Solves the LCP Ax >= b with x >= 0
///
/// This gives Ax - b = w with the residuals w >= 0,
///     the complementarity condition is dot(x, w) = 0
///     (xi = 0 or wi = 0 for each index i)
///
/// Uses total enumeration, taken from box2d's contact solver.
/// If no value is returned, the LCP had no solution.
///
////////////////////////////////////////////////////////////
template <class T>
inline Vector<T, 2> solveLcp(const Matrix<T, 2, 2>& A, const Vector<T, 2>& b);

template <class T>
class LcpSolver;


////////////////////////////////////////////////////////////
// Vector operations
////////////////////////////////////////////////////////////

template <class T, class U, Uint32 dim>
inline Vector<Promoted<T, U>, dim>
elementWiseMul(const Vector<T, dim>& lhs, const Vector<U, dim>& rhs) {
    Vector<Promoted<T, U>, dim> res;
    for (Uint32 i = 0; i < dim; ++i)
        res[i] = lhs[i] * rhs[i];

    return res;
}

template <class T, class U, Uint32 dim>
inline Promoted<T, U> dot(const Vector<T, dim>& lhs, const Vector<U, dim>& rhs);

template <class T, class U>
inline Vector<Promoted<T, U>, 3>
cross(const Vector<T, 3>& lhs, const Vector<U, 3>& rhs);

template <class T, class U>
inline Promoted<T, U> cross(const Vector<T, 2>& lhs, const Vector<U, 2>& rhs);

template <class T, Uint32 dim>
inline T normSquared(const Vector<T, dim>& v);

template <class T, Uint32 dim>
inline T norm(const Vector<T, dim>& v);

template <class T, Uint32 dim>
inline Vector<T, dim> normalized(const Vector<T, dim>& v);

template <class T, class U, Uint32 dim>
inline Vector<Promoted<T, U>, dim>
projection(const Vector<T, dim>& ofThis, const Vector<U, dim>& onThat);


////////////////////////////////////////////////////////////
/// \brief rotates v by 90 degrees
///
/// If clockwise is false, then this is (k x v)
/// If clockwise is true, then this is  (v x k)
///
////////////////////////////////////////////////////////////
template <class T>
inline Vector<T, 2> perp(const Vector<T, 2>& v, bool clockwise = false);

/// \}


////////////////////////////////////////////////////////////
// Aliases
////////////////////////////////////////////////////////////

typedef Vector<float, 2> Vec2;
typedef Vector<float, 3> Vec3;
typedef Vector<float, 4> Vec4;

typedef Vector<Int32, 2> Vec2i;
typedef Vector<Int32, 3> Vec3i;
typedef Vector<Int32, 4> Vec4i;

typedef Matrix<float, 2, 2> Mat2;
typedef Matrix<float, 3, 3> Mat3;
typedef Matrix<float, 4, 4> Mat4;

typedef Matrix<Int32, 2, 2> Mat2i;
typedef Matrix<Int32, 3, 3> Mat3i;
typedef Matrix<Int32, 4, 4> Mat4i;


////////////////////////////////////////////////////////////
// Comparison facilities
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \defgroup comparison
/// \{
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
/// \brief Comparison Matrix returned by comparisons of matrices
///
/// This is not convertible to bool, use the functions all() and any()
///     to evaluate to a bool.
///
/// All matrix comparisons are done element-wise.
///
/// element-wise boolean operators are also provided for ComparisonMatrix
////////////////////////////////////////////////////////////
template <Uint32 m, Uint32 n>
using ComparisonMatrix = Matrix<bool, m, n>;


template <class T, class U, Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator==(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator!=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator<(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator<=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator>(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator>=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);


////////////////////////////////////////////////////////////
/// \brief true if all elements are true
///
////////////////////////////////////////////////////////////
template <Uint32 m, Uint32 n>
inline bool all(const ComparisonMatrix<m, n>& comp);

////////////////////////////////////////////////////////////
/// \brief true if any element is true
///
////////////////////////////////////////////////////////////
template <Uint32 m, Uint32 n>
inline bool any(const ComparisonMatrix<m, n>& comp);

template <Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator&&(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs);

template <Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n>
operator||(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs);

template <Uint32 m, Uint32 n>
inline ComparisonMatrix<m, n> operator!(const ComparisonMatrix<m, n>& unary);

/// \}

} // namespace simu


////////////////////////////////////////////////////////////
// std overloads
////////////////////////////////////////////////////////////

namespace std
{

////////////////////////////////////////////////////////////
/// \ingroup operations
/// \{
////////////////////////////////////////////////////////////

template <class T, class U, simu::Uint32 m, simu::Uint32 n>
struct common_type<simu::Matrix<T, m, n>, simu::Matrix<U, m, n>>
{
    typedef simu::Matrix<typename std::common_type<T, U>::type, m, n> type;
};

template <class T, simu::Uint32 m, simu::Uint32 n>
inline simu::Matrix<T, m, n> abs(const simu::Matrix<T, m, n>& mat);

template <class T, simu::Uint32 m, simu::Uint32 n>
inline simu::Matrix<T, m, n> round(const simu::Matrix<T, m, n>& mat);

template <class T, simu::Uint32 m, simu::Uint32 n>
inline simu::Matrix<T, m, n>
min(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs);

template <class T, simu::Uint32 m, simu::Uint32 n>
inline simu::Matrix<T, m, n>
max(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs);

/// \}

} // namespace std


namespace simu
{

////////////////////////////////////////////////////////////
// Matrix
////////////////////////////////////////////////////////////

template <class T, Uint32 dim>
Matrix<T, dim, dim> SpecialConstructors<T, dim, dim, true>::identity() {
    Matrix<T, dim, dim> ident{};
    for (Uint32 i = 0; i < dim; ++i)
        ident(i, i) = 1;

    return ident;
}

template <class T, Uint32 dim>
Matrix<T, dim, dim>
SpecialConstructors<T, dim, dim, true>::diagonal(const Vector<T, dim>& elements) {
    Matrix<T, dim, dim> diag{};
    for (Uint32 i = 0; i < dim; ++i)
        diag(i, i) = elements[i];

    return diag;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::i() {
    Vector<T, dim> vec{};
    if constexpr (0 < dim)
        vec[0] = 1;

    return vec;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::j() {
    Vector<T, dim> vec{};
    if constexpr (1 < dim)
        vec[1] = 1;

    return vec;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::k() {
    Vector<T, dim> vec{};
    if constexpr (2 < dim)
        vec[2] = 1;

    return vec;
}

template <class T, Uint32 dim>
Vector<T, dim> SpecialConstructors<T, dim, 1, false>::w() {
    Vector<T, dim> vec{};
    if constexpr (3 < dim)
        vec[3] = 1;

    return vec;
}


template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>::Matrix(const Matrix<U, m, n>& other) {
    for (Uint32 i = 0; i < size(); ++i)
        data[i] = static_cast<T>(other.data[i]);
}

template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n>::Matrix(const std::initializer_list<T>& init) {
    SIMU_ASSERT(init.size() == this->size(), "Incorrect number of arguments in initializer list");

    for (Uint32 i = 0; i < this->size(); ++i) {
        this->data[i] = init.begin()[i];
    }
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>
Matrix<T, m, n>::fromRows(const std::initializer_list<Vector<U, n>>& rows) {
    return fromRows(Vector<Vector<U, n>, m>{rows});
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n> Matrix<T, m, n>::fromRows(const Vector<Vector<U, n>, m>& rows) {
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
Matrix<T, m, n>::fromCols(const std::initializer_list<Vector<U, m>>& cols) {
    return transpose(Matrix<T, n, m>::fromRows(cols));
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n> Matrix<T, m, n>::fromCols(const Vector<Vector<U, m>, n>& cols) {
    return transpose(Matrix<T, n, m>::fromRows(cols));
}

template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n> Matrix<T, m, n>::filled(T val) {
    Matrix<T, m, n> mat{};
    for (auto& elem : mat)
        elem = val;

    return mat;
}

template <class T, Uint32 m, Uint32 n>
Vector<Vector<T, n>, m> Matrix<T, m, n>::asRows() const {
    Vector<Vector<T, n>, m> rows{};

    auto it = this->begin();
    for (Vector<T, n>& row : rows)
        for (T& val : row)
            val = *it++;

    return rows;
}

template <class T, Uint32 m, Uint32 n>
Vector<Vector<T, m>, n> Matrix<T, m, n>::asCols() const {
    return transpose(*this).asRows();
}

template <class T, Uint32 m, Uint32 n>
T& Matrix<T, m, n>::operator()(Uint32 row, Uint32 col) {
    return data[row * nCols + col];
}

template <class T, Uint32 m, Uint32 n>
const T& Matrix<T, m, n>::operator()(Uint32 row, Uint32 col) const {
    return data[row * nCols + col];
}

template <class T, Uint32 m, Uint32 n>
T& Matrix<T, m, n>::operator[](Uint32 index) {
    return data[index];
}

template <class T, Uint32 m, Uint32 n>
const T& Matrix<T, m, n>::operator[](Uint32 index) const {
    return data[index];
}


template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n> Matrix<T, m, n>::operator+() const {
    return *this;
}

template <class T, Uint32 m, Uint32 n>
Matrix<T, m, n> Matrix<T, m, n>::operator-() const {
    Matrix<T, m, n> res{*this};
    for (T& x : res)
        x = -x;

    return res;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator+=(const Matrix<U, m, n>& other) {
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] += other.data[i];

    return *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator-=(const Matrix<U, m, n>& other) {
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] -= other.data[i];

    return *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator*=(U scalar) {
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] *= scalar;

    return *this;
}

template <class T, Uint32 m, Uint32 n>
template <class U>
Matrix<T, m, n>& Matrix<T, m, n>::operator/=(U scalar) {
    for (Uint32 i = 0; i < this->size(); ++i)
        this->data[i] /= scalar;

    return *this;
}


template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n>
operator+(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    Matrix<Promoted<T, U>, m, n> res{lhs};
    return res += rhs;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n>
operator-(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    Matrix<Promoted<T, U>, m, n> res{lhs};
    return res -= rhs;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator*(U scalar, const Matrix<T, m, n>& mat) {
    Matrix<Promoted<T, U>, m, n> res{mat};
    return res *= scalar;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator*(const Matrix<T, m, n>& mat, U scalar) {
    return scalar * mat;
}

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator/(const Matrix<T, m, n>& mat, U scalar) {
    Matrix<Promoted<T, U>, m, n> res{mat};
    return res /= scalar;
}

template <class T, class U, Uint32 mLeft, Uint32 nLeft, Uint32 nRight>
Matrix<Promoted<T, U>, mLeft, nRight>
operator*(const Matrix<T, mLeft, nLeft>& lhs, const Matrix<U, nLeft, nRight>& rhs) {
    Matrix<Promoted<T, U>, mLeft, nRight> res{};
    for (Uint32 row = 0; row < lhs.mRows; ++row) {
        for (Uint32 col = 0; col < rhs.nCols; ++col) {
            for (Uint32 k = 0; k < lhs.nCols; ++k) {
                res(row, col) += lhs(row, k) * rhs(k, col);
            }
        }
    }

    return res;
}


template <class T, Uint32 m, Uint32 n>
Matrix<T, n, m> transpose(const Matrix<T, m, n>& original) {
    Matrix<T, n, m> res;
    for (Uint32 row = 0; row < m; ++row) {
        for (Uint32 col = 0; col < n; ++col) {
            res(col, row) = original(row, col);
        }
    }

    return res;
}


template <class T>
class Solver<T, 2>
{
public:

    Solver(const Matrix<T, 2, 2>& A) : A_{A} {
        invDet_  = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
        isValid_ = (invDet_ != 0.f);
        if (isValid_)
            invDet_ = 1.f / invDet_;
    }

    template <class U>
    Vector<Promoted<T, U>, 2> solve(const Vector<U, 2>& b) const {
        SIMU_ASSERT(isValid_, "Solver invalid, ensure the original matrix has full rank");

        // use cramer's rule
        return invDet_
               * Vector<Promoted<T, U>, 2>{
                   b[0] * A_(1, 1) - A_(0, 1) * b[1], A_(0, 0) * b[1] - b[0] * A_(1, 0)
               };
    }

    Matrix<T, 2, 2> original() const { return A_; }

    bool isValid() const { return isValid_; }

private:

    template <class U>
    friend class LcpSolver;

    Matrix<T, 2, 2> A_;
    T               invDet_;
    bool            isValid_;
};

template <class T, class U, Uint32 n>
Vector<Promoted<T, U>, n> solve(const Matrix<T, n, n>& A, const Vector<U, n>& b) {
    return Solver{A}.solve(b);
}

template <class T, Uint32 n>
Solver<T, n>::Solver(const Matrix<T, n, n>& A) : R_{} {
    // modified Gram-Schmidt for QR decomposition
    // https://www.math.uci.edu/~ttrogdon/105A/html/Lecture23.html

    auto Q = A.asCols();

    for (Uint32 col = 0; col < n; ++col) {
        R_(col, col) = norm(Q[col]);

        isValid_ = isValid_ && R_(col, col) != 0.f;
        if (!isValid_)
            return;

        Q[col] /= R_(col, col);

        for (Uint32 nextCol = col + 1; nextCol < n; ++nextCol) {
            R_(col, nextCol) = dot(Q[col], Q[nextCol]);
            Q[nextCol] -= R_(col, nextCol) * Q[col];
        }
    }

    QT_ = transpose(Matrix<T, n, n>::fromCols(Q));
}

template <class T, Uint32 n>
template <class U>
Vector<Promoted<T, U>, n> Solver<T, n>::solve(const Vector<U, n>& b) const {
    SIMU_ASSERT(isValid_, "Solver invalid, ensure the original matrix has full rank");

    Vector<Promoted<T, U>, n> c = QT_ * b;
    Vector<Promoted<T, U>, n> x{};

    for (Uint32 row = n; row > 0; --row) {
        for (Uint32 col = row; col < n; ++col) {
            c[row - 1] -= R_(row - 1, col) * x[col];
        }

        x[row - 1] = c[row - 1] / R_(row - 1, row - 1);
    }

    return x;
}


template <class T, Uint32 n>
Matrix<T, n, n> invert(const Matrix<T, n, n>& mat) {
    Solver<T, n> solver{mat};

    auto inverse = Matrix<T, n, n>::identity().asCols();
    for (auto& col : inverse)
        col = solver.solve(col);

    return Matrix<T, n, n>::fromCols(inverse);
}


template <class T, Uint32 n, std::invocable<Vector<T, n>, Uint32> Proj>
Vector<T, n>
solveInequalities(const Matrix<T, n, n>& A, Vector<T, n> b, Proj proj, Vector<T, n> initialGuess, float epsilon) {
    b = -b; // Ax - b >= 0

    Vector<T, n> x = initialGuess;

    float eps = 1.f + epsilon;
    while (eps > epsilon) {
        eps = 0.f;
        for (Uint32 row = 0; row < n; ++row) {
            float delX = 0.f;

            for (Uint32 col = 0; col < row; ++col)
                delX += A(row, col) * x[col];

            for (Uint32 col = row + 1; col < n; ++col)
                delX += A(row, col) * x[col];

            delX = -(delX + b[row]) / A(row, row);

            Vector<T, n> newX{x};
            newX[row] = delX;
            delX      = proj(newX, row);

            eps    = std::max(eps, std::abs(delX - x[row]));
            x[row] = delX;
        }
    }

    return x;
}


template <class T, Uint32 n, std::invocable<Vector<T, n>> Proj>
Vector<T, n>
solveInequalities(const Matrix<T, n, n>& A, Vector<T, n> b, Proj proj, Vector<T, n> initialGuess, float epsilon) {
    return solveInequalities(
        A, b, [=](const Vector<T, n>& x, Uint32 i) { return proj(x)[i]; }, initialGuess, epsilon
    );
}


template <class T>
class LcpSolver
{
public:

    LcpSolver(const Matrix<T, 2, 2>& A)
        : solver{A}, invA11{1.f / A(0, 0)}, invA22{1.f / A(1, 1)} {
        solver.isValid_ = solver.isValid() && A(0, 0) != 0.f && A(1, 1) != 0.f;
    }

    template <class U>
    Vector<Promoted<T, U>, 2> solve(const Vector<U, 2>& b) const {
        Vector<T, 2> x = solver.solve(b);
        if (all(x >= Vector<T, 2>::filled(0.f)))
            return x;

        x = Vector<T, 2>{b[0] * invA11, 0.f};
        if ((x[0] >= 0.f) && (x[0] * solver.A_(1, 0) - b[1] >= 0))
            return x;

        x = Vector<T, 2>{0.f, b[1] * invA22};
        if ((x[1] >= 0.f) && (x[1] * solver.A_(0, 1) - b[0] >= 0))
            return x;

        x = Vector<T, 2>{};
        if (all(-b >= Vector<T, 2>::filled(0.f)))
            return x;

        return x; // all null
    }

    Matrix<T, 2, 2> original() const { return solver.original(); }
    bool            isValid() const { return solver.isValid(); }

private:

    Solver<T, 2> solver;
    float        invA11;
    float        invA22;
};

template <class T>
Vector<T, 2> solveLcp(const Matrix<T, 2, 2>& A, const Vector<T, 2>& b) {
    return LcpSolver{A}.solve(b);
}


////////////////////////////////////////////////////////////
// Vector operations
////////////////////////////////////////////////////////////

template <class T, class U, Uint32 dim>
Promoted<T, U> dot(const Vector<T, dim>& lhs, const Vector<U, dim>& rhs) {
    Promoted<T, U> sum{};
    for (Uint32 i = 0; i < dim; ++i)
        sum += lhs[i] * rhs[i];

    return sum;
}

template <class T, Uint32 dim>
T normSquared(const Vector<T, dim>& v) {
    return dot(v, v);
}

template <class T, Uint32 dim>
T norm(const Vector<T, dim>& v) {
    return std::sqrt(normSquared(v));
}

template <class T, Uint32 dim>
Vector<T, dim> normalized(const Vector<T, dim>& v) {
    return v / norm(v);
}

template <class T>
Vector<T, 2> perp(const Vector<T, 2>& v, bool clockwise) {
    return clockwise ? -perp(v, false) : Vector<T, 2>{-v[1], v[0]};
}

template <class T, class U>
Vector<Promoted<T, U>, 3> cross(const Vector<T, 3>& lhs, const Vector<U, 3>& rhs) {
    return Vector<Promoted<T, U>, 3>{
        cross(Vector<T, 2>{lhs[1], lhs[2]}, Vector<U, 2>{rhs[1], rhs[2]}),
        -cross(Vector<T, 2>{lhs[0], lhs[2]}, Vector<U, 2>{rhs[0], rhs[2]}),
        cross(Vector<T, 2>{lhs[0], lhs[1]}, Vector<U, 2>{rhs[0], rhs[1]})
    };
}

template <class T, class U>
Promoted<T, U> cross(const Vector<T, 2>& lhs, const Vector<U, 2>& rhs) {
    return lhs[0] * rhs[1] - lhs[1] * rhs[0];
}

template <class T, class U, Uint32 dim>
Vector<Promoted<T, U>, dim>
projection(const Vector<T, dim>& ofThis, const Vector<U, dim>& onThat) {
    return onThat * dot(ofThis, onThat) / normSquared(onThat);
}


////////////////////////////////////////////////////////////
// Comparison facilities
////////////////////////////////////////////////////////////

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator==(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = lhs[i] == rhs[i];
    }

    return res;
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator!=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    return !(lhs == rhs);
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator<(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = lhs[i] < rhs[i];
    }

    return res;
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator<=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    return !(lhs > rhs);
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator>(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = lhs[i] > rhs[i];
    }

    return res;
}

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator>=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs) {
    return !(lhs < rhs);
}

template <Uint32 m, Uint32 n>
bool all(const ComparisonMatrix<m, n>& comp) {
    for (bool b : comp)
        if (!b)
            return false;

    return true;
}

template <Uint32 m, Uint32 n>
bool any(const ComparisonMatrix<m, n>& comp) {
    for (bool b : comp)
        if (b)
            return true;

    return false;
}

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator&&(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs) {
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = lhs[i] && rhs[i];
    }

    return res;
}

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator||(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs) {
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = lhs[i] || rhs[i];
    }

    return res;
}

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n> operator!(const ComparisonMatrix<m, n>& unary) {
    ComparisonMatrix<m, n> res;
    for (Uint32 i = 0; i < res.size(); ++i) {
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
simu::Matrix<T, m, n> abs(const simu::Matrix<T, m, n>& mat) {
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < mat.size(); ++i) {
        res[i] = std::abs(mat[i]);
    }

    return res;
}

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n> round(const simu::Matrix<T, m, n>& mat) {
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < mat.size(); ++i) {
        res[i] = std::round(mat[i]);
    }

    return res;
}

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n>
min(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs) {
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = std::min(lhs[i], rhs[i]);
    }

    return res;
}

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n>
max(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs) {
    simu::Matrix<T, m, n> res;
    for (simu::Uint32 i = 0; i < lhs.size(); ++i) {
        res[i] = std::max(lhs[i], rhs[i]);
    }

    return res;
}

} // namespace std
