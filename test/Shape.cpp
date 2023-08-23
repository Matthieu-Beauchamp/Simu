#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Shape.hpp"
#include "Simu/math/Circle.hpp"

using namespace simu;

struct TestCircle : public Shape, public Circle
{
    static constexpr Uint32 type = Shape::Type::BEGIN_USER_TYPE;

    TestCircle(float r, Vec2 c) : Shape{type}, Circle{r, c} {}

    Properties properties() const override { return Properties{}; }
};

CollisionManifold collideCircle(const Shape& A, const Shape& B)
{
    auto cA = static_cast<const TestCircle*>(&A);
    auto cB = static_cast<const TestCircle*>(&B);

    CollisionManifold mani;

    // TODO: Edge case when n == Vec2{};
    Vec2  n = cA->center() - cB->center();
    float d = norm(n);

    mani.normal  = normalized(n);
    mani.tangent = perp(mani.normal);
    if (d < cA->radius() + cB->radius())
    {
        mani.nContacts    = 1;
        mani.contactsA[0] = cA->center() - mani.normal * cA->radius();
        mani.contactsB[0] = cB->center() + mani.normal * cB->radius();
    }

    return mani;
}


TEST_CASE("Shape")
{
    SECTION("Proof of concept")
    {
        ShapeCollider<std::allocator<int>> collider{};

        collider.registerCollisionCallback<TestCircle::type, TestCircle::type, collideCircle>(
        );

        auto mani = collider.collide(TestCircle{1.f, Vec2{}}, TestCircle{1.f, Vec2{0.5f, 0.f}});
        REQUIRE(all(mani.normal == Vec2{-1.f, 0.f}));
    }
}