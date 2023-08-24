#include "catch2/catch_test_macros.hpp"

#include "Simu/math/ShapeCollision.hpp"
#include "Simu/math/Polygon.hpp"

using namespace simu;

// TODO: Adapt to Shape collisions

typedef CollisionManifold Manifold;

bool inContacts(auto contacts, Vec2 v)
{
    for (const Vec2& c : contacts)
    {
        if (all(c == v))
            return true;
    }

    return false;
}

void checkIsInvert(Manifold mani, Manifold inv)
{
    REQUIRE(mani.nContacts == inv.nContacts);
    REQUIRE(all(mani.normal == -inv.normal));
    REQUIRE(all(mani.tangent == -inv.tangent));

    for (Uint32 i = 0; i < mani.nContacts; ++i)
    {
        REQUIRE(inContacts(inv.contactsB, mani.contactsA[i]));
        REQUIRE(inContacts(inv.contactsA, mani.contactsB[i]));
    }
}

Manifold makeManifold(const Polygon& first, const Polygon& second)
{
    static ShapeCollider<std::allocator<int>> collider(std::allocator<int>{});

    Manifold manifold = collider.collide(first, second);

    Manifold inv = collider.collide(second, first);
    checkIsInvert(manifold, inv);

    return manifold;
}


TEST_CASE("Contact manifolds")
{
    SECTION("Vertex to Vertex")
    {
        // never happens, unless the bodies are only touching -> nothing to do
    }

    SECTION("Vertex to Edge")
    {
        Polygon bottom{
            Vec2{0, 0},
            Vec2{2, 0},
            Vec2{2, 2},
            Vec2{0, 2}
        };

        Polygon top{
            Vec2{1, 1.5f},
            Vec2{2, 3   },
            Vec2{0, 3   }
        };

        {
            auto manifold = makeManifold(bottom, top);


            Vec2 n = manifold.normal;
            REQUIRE(
                all(approx(Vec2{0, -1}, Vec2::filled(simu::EPSILON)).contains(n))
            );
            REQUIRE(manifold.nContacts == 1);
            REQUIRE(all(manifold.contactsB[0] == Vec2{1, 1.5f}));
        }
        {
            auto manifold = makeManifold(top, bottom);


            Vec2 n = manifold.normal;
            REQUIRE(all(approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON)).contains(n)));
            REQUIRE(manifold.nContacts == 1);
            REQUIRE(all(manifold.contactsA[0] == Vec2{1, 1.5f}));
        }
    }

    SECTION("Edge to Edge")
    {
        SECTION("Flush sides")
        {
            Polygon bottom{
                Vec2{0, 0},
                Vec2{2, 0},
                Vec2{2, 2},
                Vec2{0, 2}
            };

            Polygon top{
                Vec2{0, 1},
                Vec2{2, 1},
                Vec2{2, 3},
                Vec2{0, 3}
            };


            auto manifold = makeManifold(bottom, top);
            REQUIRE(manifold.nContacts == 2);


            REQUIRE(all(approx(Vec2{0, -1}, Vec2::filled(simu::EPSILON))
                            .contains(manifold.normal)));

            REQUIRE(inContacts(manifold.contactsA, Vec2{0, 2}));
            REQUIRE(inContacts(manifold.contactsA, Vec2{2, 2}));
        }

        SECTION("Uneven sides")
        {
            Polygon bottom{
                Vec2{0, 0},
                Vec2{2, 0},
                Vec2{2, 2},
                Vec2{0, 2}
            };

            Polygon top{
                Vec2{0.5f, 1.5f},
                Vec2{2.5f, 1.5f},
                Vec2{2.5f, 3.5f},
                Vec2{0.5f, 3.5f}
            };

            auto manifold = makeManifold(bottom, top);

            REQUIRE(manifold.nContacts == 2);


            REQUIRE(all(approx(Vec2{0, -1}, Vec2::filled(simu::EPSILON))
                            .contains(manifold.normal)));

            REQUIRE(inContacts(manifold.contactsA, Vec2{0.5f, 2})); // clipped
            REQUIRE(inContacts(manifold.contactsA, Vec2{2, 2}));
        }
    }

    SECTION("Leaning inside")
    {
        Polygon bottom{
            Vec2{0, 0},
            Vec2{2, 0},
            Vec2{2, 2},
            Vec2{0, 2}
        };

        Polygon top{
            Vec2{0.25f, 1.75f},
            Vec2{2.f,   2.f  },
            Vec2{1.75f, 3.f  },
            Vec2{0.f,   2.75f},
        };

        auto manifold = makeManifold(top, bottom);

        REQUIRE(all(
            approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON)).contains(manifold.normal)
        ));

        REQUIRE(manifold.nContacts == 2);

        REQUIRE(inContacts(manifold.contactsA, Vec2{0.25f, 1.75f}));
        REQUIRE(inContacts(manifold.contactsA, Vec2{2.f, 2.f}));

        REQUIRE(inContacts(manifold.contactsB, Vec2{0.25f, 2.f}));
        REQUIRE(inContacts(manifold.contactsB, Vec2{2.f, 2.f}));
    }

    SECTION("Edge distance to origin")
    {
        // hard to find a test that will encounter this situation,
        //  but did happen in practice

        std::array<Vec2, 3> v{
            Vec2{-1,    1e-8f},
            Vec2{1e-6f, 1e-8f},
            Vec2{1,     1e-8f},
        };

        auto it = v.begin();

        Edges<std::array<Vec2, 3>>::Edge e1{it, it + 1};
        Edges<std::array<Vec2, 3>>::Edge e2{it + 1, it + 2};

        REQUIRE(e1.distanceSquaredToOrigin() < e2.distanceSquaredToOrigin());
    }
}
