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

#include "Simu/config.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \ingroup math
/// \defgroup LinearAlgebra
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \brief Matrix data, stored row-major
///
/// Two methods for random access are provided,
///     For matrices,  mat(i, j) is recommended
///     For vectors,   vec[i]    is recommended
///
/// Direct access to the data member is allowed, but for matrices
///     it is preferable to use operator() to avoid errors
////////////////////////////////////////////////////////////
template <class T, Uint32 mRows_, Uint32 nCols_>
struct MatrixData
{
    typedef T        value_type;
    typedef T*       iterator;
    typedef const T* const_iterator;

    static constexpr Uint32 mRows = mRows_;
    static constexpr Uint32 nCols = nCols_;

    MatrixData() = default;

    template <class U>
    explicit MatrixData(const MatrixData<U, mRows_, nCols_>& other);

    constexpr static Uint32 size() { return mRows * nCols; }

    T&       operator()(Uint32 row, Uint32 col);
    const T& operator()(Uint32 row, Uint32 col) const;

    T&       operator[](Uint32 index);
    const T& operator[](Uint32 index) const;


    iterator begin() { return data; }
    iterator end() { return data + size(); }

    const_iterator begin() const { return data; }
    const_iterator end() const { return data + size(); }

    T data[mRows * nCols];
};


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
    static Matrix<T, dim, dim> identity();
    static Matrix<T, dim, dim> diagonal(const Vector<T, dim>& elements);
};

template <class T, Uint32 dim>
struct SpecialConstructors<T, dim, 1, false>
{
    static Vector<T, dim> i();
    static Vector<T, dim> j();
    static Vector<T, dim> k();
    static Vector<T, dim> w();
};

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \brief Matrix class for small matrices
///
/// Most operations are provided as global functions
/// \see operations
////////////////////////////////////////////////////////////
template <class T, Uint32 m, Uint32 n>
struct Matrix : public MatrixData<T, m, n>, public SpecialConstructors<T, m, n>
{
    Matrix() = default;

    template <class U>
    explicit Matrix(const Matrix<U, m, n>& other);

    explicit Matrix(const MatrixData<T, m, n>& data);

    explicit Matrix(const std::initializer_list<T>& init);

    static Matrix filled(T val);

    template <class U>
    static Matrix fromRows(const std::initializer_list<Vector<U, n>>& rows);

    template <class U>
    static Matrix fromRows(const Vector<Vector<U, n>, m>& rows);

    template <class U>
    static Matrix fromCols(const std::initializer_list<Vector<U, m>>& cols);

    template <class U>
    static Matrix fromCols(const Vector<Vector<U, m>, n>& cols);


    Vector<Vector<T, n>, m> asRows() const;

    Vector<Vector<T, m>, n> asCols() const;


    Matrix operator+() const;
    Matrix operator-() const;

    template <class U>
    Matrix& operator+=(const Matrix<U, m, n>& other);

    template <class U>
    Matrix& operator-=(const Matrix<U, m, n>& other);

    template <class U>
    Matrix& operator*=(U scalar);

    template <class U>
    Matrix& operator/=(U scalar);
};

////////////////////////////////////////////////////////////
/// \ingroup LinearAlgebra
/// \defgroup operations
/// \{
////////////////////////////////////////////////////////////

template <class T, class U>
using Promoted = typename std::common_type<T, U>::type;


template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n>
operator+(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n>
operator-(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator*(U scalar, const Matrix<T, m, n>& mat);

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator*(const Matrix<T, m, n>& mat, U scalar);

template <class T, class U, Uint32 m, Uint32 n>
Matrix<Promoted<T, U>, m, n> operator/(const Matrix<T, m, n>& mat, U scalar);

template <class T, class U, Uint32 mLeft, Uint32 nLeft, Uint32 nRight>
Matrix<Promoted<T, U>, mLeft, nRight>
operator*(const Matrix<T, mLeft, nLeft>& lhs, const Matrix<U, nLeft, nRight>& rhs);


template <class T, Uint32 m, Uint32 n>
Matrix<T, n, m> transpose(const Matrix<T, m, n>& original);


////////////////////////////////////////////////////////////
/// \brief Return x such that Ax = b
///
////////////////////////////////////////////////////////////
template <class T, class U, Uint32 n>
Vector<Promoted<T, U>, n> solve(const Matrix<T, n, n>& A, const Vector<U, n>& b);

template <class T, Uint32 n>
class Solver
{
public:

    Solver(const Matrix<T, n, n>& A);

    template <class U>
    Vector<Promoted<T, U>, n> solve(const Vector<U, n>& b) const;

    Matrix<T, n, n> original() const { return transpose(QT_) * R_; }

    bool isValid() const { return isValid_; }

private:

    Matrix<T, n, n> QT_;
    Matrix<T, n, n> R_;
    bool            isValid_ = true;
};

template <class T, Uint32 n>
Matrix<T, n, n> invert(const Matrix<T, n, n>& mat);


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
Vector<T, n> solveInequalities(
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
Vector<T, n> solveInequalities(
    const Matrix<T, n, n>& A,
    Vector<T, n>           b,
    Proj                   proj,
    Vector<T, n>           initialGuess = Vector<T, n>{},
    float                  epsilon      = simu::EPSILON
);


////////////////////////////////////////////////////////////
// Vector operations
////////////////////////////////////////////////////////////

template <class T, class U, Uint32 dim>
Promoted<T, U> dot(const Vector<T, dim>& lhs, const Vector<U, dim>& rhs);

template <class T, class U>
Vector<Promoted<T, U>, 3>
cross(const Vector<T, 3>& lhs, const Vector<U, 3>& rhs);

template <class T, class U>
Promoted<T, U> cross(const Vector<T, 2>& lhs, const Vector<U, 2>& rhs);

template <class T, Uint32 dim>
T normSquared(const Vector<T, dim>& v);

template <class T, Uint32 dim>
T norm(const Vector<T, dim>& v);

template <class T, Uint32 dim>
Vector<T, dim> normalized(const Vector<T, dim>& v);

template <class T, class U, Uint32 dim>
Vector<Promoted<T, U>, dim>
projection(const Vector<T, dim>& ofThis, const Vector<U, dim>& onThat);


////////////////////////////////////////////////////////////
/// \brief rotates v by 90 degrees
////////////////////////////////////////////////////////////
template <class T>
Vector<T, 2> perp(const Vector<T, 2>& v, bool clockwise = false);

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
using ComparisonMatrix = MatrixData<bool, m, n>;


template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator==(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator!=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator<(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator<=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator>(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);

template <class T, class U, Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator>=(const Matrix<T, m, n>& lhs, const Matrix<U, m, n>& rhs);


////////////////////////////////////////////////////////////
/// \brief true if all elements are true
///
////////////////////////////////////////////////////////////
template <Uint32 m, Uint32 n>
bool all(const ComparisonMatrix<m, n>& comp);

////////////////////////////////////////////////////////////
/// \brief true if any element is true
///
////////////////////////////////////////////////////////////
template <Uint32 m, Uint32 n>
bool any(const ComparisonMatrix<m, n>& comp);

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator&&(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs);

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n>
operator||(const ComparisonMatrix<m, n>& lhs, const ComparisonMatrix<m, n>& rhs);

template <Uint32 m, Uint32 n>
ComparisonMatrix<m, n> operator!(const ComparisonMatrix<m, n>& unary);

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
simu::Matrix<T, m, n> abs(const simu::Matrix<T, m, n>& mat);

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n> round(const simu::Matrix<T, m, n>& mat);

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n>
min(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs);

template <class T, simu::Uint32 m, simu::Uint32 n>
simu::Matrix<T, m, n>
max(const simu::Matrix<T, m, n>& lhs, const simu::Matrix<T, m, n>& rhs);

/// \}

} // namespace std


#include "Simu/math/Matrix.inl.hpp"
