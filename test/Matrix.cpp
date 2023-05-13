#include <numbers>

#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Matrix.hpp"
#include "Simu/math/Interval.hpp"

using namespace simu;

TEST_CASE("Matrix")
{
    SECTION("Matrix Data")
    {
        Mat2 null{};
        REQUIRE(all(null == Mat2{0, 0, 0, 0}));
    }

    SECTION("Construct from vectors")
    {
        Matrix<int, 2, 4> ref{1, 2, 3, 4, 5, 6, 7, 8};

        REQUIRE(
            all(ref
                == Matrix<int, 2, 4>::fromRows({
                    Vec4i{1, 2, 3, 4},
                    Vec4i{5, 6, 7, 8}
        }))
        );

        REQUIRE(
            all(ref
                == Matrix<int, 2, 4>::fromCols({
                    Vec2i{1, 5},
                    Vec2i{2, 6},
                    Vec2i{3, 7},
                    Vec2i{4, 8},
        }))
        );
    }

    SECTION("Filled")
    {
        Mat2i ref{1, 1, 1, 1};
        REQUIRE(all(ref == Mat2i::filled(1)));
    }

    SECTION("Special constructors")
    {
        REQUIRE(all(
            Mat4::identity()
            == Mat4{
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1,
            }
        ));

        REQUIRE(
            all(Mat4::identity()
                == Mat4::fromCols({Vec4::i(), Vec4::j(), Vec4::k(), Vec4::w()}))
        );

        REQUIRE(
            all(Mat4::identity()
                == Mat4::fromRows({Vec4::i(), Vec4::j(), Vec4::k(), Vec4::w()}))
        );

        // may also be made to not compile
        REQUIRE(all(Vec3i::w() == Vec3i{}));
    }

    SECTION("Unary operators")
    {
        Mat2i ident = Mat2i::identity();
        REQUIRE(all(+ident == ident));
        REQUIRE(all(-ident + ident == Mat2i{}));
        REQUIRE(all(+-+-ident == ident));
    }

    SECTION("Arithmetic assignement")
    {
        Mat2 ones = Mat2::filled(1.f);

        ones *= 2;
        REQUIRE(all(ones == Mat2::filled(2.f)));

        ones /= 2;
        REQUIRE(all(ones == Mat2::filled(1.f)));

        ones += ones;
        REQUIRE(all(ones == Mat2::filled(2.f)));

        ones -= ones;
        REQUIRE(all(ones == Mat2{}));
    }

    SECTION("Linear combination (non-member operators)")
    {
        // Binary operators returning by value must promote the return type
        auto combination = Vec3i::i() * 0.5f - Vec3i::i() + 0.5f * Vec3i::j()
                           + Vec3i::k() / 2.f;

        REQUIRE(all(combination == Vec3{-0.5f, 0.5f, 0.5f}));
    }

    SECTION("Matrix multiplication")
    {
        Mat2 ident = Mat2::identity();
        REQUIRE(all(ident * ident == ident));

        REQUIRE(all(transpose(Vec2::i()) * Vec2::j() == Vector<int, 1>{0}));

        REQUIRE(all(Vec2{1, 2} * transpose(Vec2{2, 1}) == Mat2{2, 1, 4, 2}));

        float theta = std::numbers::pi / 4;
        Mat2  rot{
            std::cos(theta), -std::sin(theta),
            std::sin(theta),  std::cos(theta)
        };

        REQUIRE(all(approx(normalized(Vec2{1, 1}), Vec2::filled(1e-6f))
                        .contains(rot * Vec2::i())));
    }

    SECTION("Vector operations")
    {
        REQUIRE(all(cross(Vec3::i(), Vec3::j()) == Vec3::k()));
        REQUIRE(all(cross(Vec3::j(), Vec3::i()) == -Vec3::k()));

        REQUIRE(all(cross(Vec3::j(), Vec3::k()) == Vec3::i()));
        REQUIRE(all(cross(Vec3::k(), Vec3::j()) == -Vec3::i()));

        REQUIRE(all(cross(Vec3::k(), Vec3::i()) == Vec3::j()));
        REQUIRE(all(cross(Vec3::i(), Vec3::k()) == -Vec3::j()));

        REQUIRE(all(cross(Vec3::i(), Vec3::i()) == Vec3{}));
    }

    SECTION("std and comparison matrices")
    {
        Vec3 v = Vec3::filled(-1.25f);

        REQUIRE(!any(v == std::abs(v)));
        REQUIRE(all(v != std::abs(v)));

        Vec3 absV = std::abs(v);
        REQUIRE(all(v < absV));
        REQUIRE(all(v <= absV));

        REQUIRE(all(absV > std::round(absV)));
        REQUIRE(all(absV >= std::round(absV)));
    }
}
