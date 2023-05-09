#include "catch2/catch_test_macros.hpp"
#include "Simu/math/Matrix.hpp"

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
        Matrix<int, 2, 4> ref{
            1, 2, 3, 4, 
            5, 6, 7, 8
        };

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
    }

    SECTION("Unary operators")
    {
        Mat2i ident = Mat2i::identity();
        REQUIRE(all(+ident == ident));
        REQUIRE(all(-ident + ident == Mat2i{}));
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

    }
}