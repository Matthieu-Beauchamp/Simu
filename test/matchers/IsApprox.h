#pragma once
#include <catch2/matchers/catch_matchers.hpp>
#include <cmath>
#include "Simu/math/Interval.hpp"
#include "Simu/math/Matrix.hpp"

namespace simu
{

template<class T, Uint32 m, Uint32 n>
class IsApprox: public Catch::Matchers::MatcherBase<Matrix<T, m, n>>
{
public:
    using MatrixType = Matrix<T, m, n>;

    explicit IsApprox(MatrixType target, float eps) : target(target), epsilon(eps) {}

    bool match(const MatrixType& vec) const override {
        return all(approx(target, Vec2::filled(epsilon)).contains(vec));
    }

    std::string describe() const override {
        return "is approximately equal to " + to_string(target) + " (within ε = " + std::to_string(epsilon) + ")";
    }

private:

    MatrixType target;
    float epsilon;
};

template<class T, Uint32 m, Uint32 n>
inline auto isApprox(Matrix<T, m, n> matrix, float eps = EPSILON) {
    return IsApprox(matrix, eps);
}

} // namespace simu
