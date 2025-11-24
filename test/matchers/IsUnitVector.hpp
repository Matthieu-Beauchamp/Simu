#pragma once
#include <catch2/matchers/catch_matchers.hpp>
#include <cmath>
#include "Simu/math/Matrix.hpp"

namespace simu
{

class IsUnitVectorMatcher : public Catch::Matchers::MatcherBase<Vec2>
{
public:

    explicit IsUnitVectorMatcher(float eps = 1e-5f) : epsilon(eps) {}

    bool match(const Vec2& vec) const override {
        if (std::isnan(vec[0]) || std::isnan(vec[1])) {
            return false;
        }

        float len = norm(vec);
        return std::abs(len - 1.0f) < epsilon;
    }

    std::string describe() const override {
        return "is a unit vector (|v| == 1 within ε = " + std::to_string(epsilon) + ")";
    }

private:

    float epsilon;
};

inline IsUnitVectorMatcher isUnitVector(float eps = EPSILON) {
    return IsUnitVectorMatcher(eps);
}

} // namespace simu
