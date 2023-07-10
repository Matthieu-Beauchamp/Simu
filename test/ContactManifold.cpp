#include "catch2/catch_test_macros.hpp"

#include "Simu/physics/ContactManifold.hpp"
#include "Simu/math/Gjk.hpp"

using namespace simu;

template <Geometry T>
ContactManifold<T> makeContactManifold(std::array<T*, 2> bodies)
{
    Gjk<T> gjk{*bodies[0], *bodies[1]};
    return ContactManifold<T>{bodies[0], bodies[1], gjk.penetration()};
}

bool inContacts(auto contacts, Vertex v)
{
    for (const Vertex& c : contacts)
    {
        if (all(c == v))
            return true;
    }

    return false;
}

TEST_CASE("Contact manifolds")
{
    typedef std::array<Polygon*, 2> Polys;

    SECTION("Vertex to Vertex")
    {
        // never happens, unless the bodies are only touching -> nothing to do
    }

    SECTION("Vertex to Edge")
    {
        Polygon bottom{
            Vertex{0, 0},
            Vertex{2, 0},
            Vertex{2, 2},
            Vertex{0, 2}
        };

        Polygon top{
            Vertex{1, 1.5f},
            Vertex{2, 3   },
            Vertex{0, 3   }
        };

        {
            auto manifold = makeContactManifold(Polys{&bottom, &top});

            REQUIRE(manifold.referenceIndex() == 0);
            REQUIRE(manifold.incidentIndex() == 1);

            REQUIRE(all(approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON))
                            .contains(normalized(manifold.contactNormal))));
            REQUIRE(manifold.nContacts == 1);
            REQUIRE(all(
                manifold.contacts[manifold.incidentIndex()][0] == Vertex{1, 1.5f}
            ));
        }
        {
            auto manifold = makeContactManifold(Polys{&top, &bottom});

            REQUIRE(manifold.referenceIndex() == 1);
            REQUIRE(manifold.incidentIndex() == 0);

            REQUIRE(all(approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON))
                            .contains(normalized(manifold.contactNormal))));
            REQUIRE(manifold.nContacts == 1);
            REQUIRE(all(
                manifold.contacts[manifold.incidentIndex()][0] == Vertex{1, 1.5f}
            ));
        }
    }

    SECTION("Edge to Edge")
    {
        SECTION("Flush sides")
        {
            Polygon bottom{
                Vertex{0, 0},
                Vertex{2, 0},
                Vertex{2, 2},
                Vertex{0, 2}
            };

            Polygon top{
                Vertex{0, 1},
                Vertex{2, 1},
                Vertex{2, 3},
                Vertex{0, 3}
            };


            auto manifold = makeContactManifold(Polys{&bottom, &top});

            REQUIRE(manifold.nContacts == 2);
            if (manifold.referenceIndex() == 0)
            {
                REQUIRE(all(approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON))
                                .contains(normalized(manifold.contactNormal))));

                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{0, 1}
                ));
                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{2, 1}
                ));
            }
            else
            {
                REQUIRE(all(approx(Vec2{0, -1}, Vec2::filled(simu::EPSILON))
                                .contains(normalized(manifold.contactNormal))));

                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{0, 2}
                ));
                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{2, 2}
                ));
            }
        }

        SECTION("Uneven sides")
        {
            Polygon bottom{
                Vertex{0, 0},
                Vertex{2, 0},
                Vertex{2, 2},
                Vertex{0, 2}
            };

            Polygon top{
                Vertex{0.5f, 1.5f},
                Vertex{2.5f, 1.5f},
                Vertex{2.5f, 3.5f},
                Vertex{0.5f, 3.5f}
            };

            auto manifold = makeContactManifold(Polys{&bottom, &top});

            REQUIRE(manifold.nContacts == 2);
            if (manifold.referenceIndex() == 0)
            {
                REQUIRE(all(approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON))
                                .contains(normalized(manifold.contactNormal))));

                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{0.5f, 1.5f}
                ));
                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{2.f, 1.5f}
                )); // clipped
            }
            else
            {
                REQUIRE(all(approx(Vec2{0, -1}, Vec2::filled(simu::EPSILON))
                                .contains(normalized(manifold.contactNormal))));

                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{0.5f, 2}
                )); // clipped
                REQUIRE(inContacts(
                    manifold.contacts[manifold.incidentIndex()],
                    Vertex{2, 2}
                ));
            }
        }
    }

    SECTION("Leaning inside")
    {
        Polygon bottom{
            Vertex{0, 0},
            Vertex{2, 0},
            Vertex{2, 2},
            Vertex{0, 2}
        };

        Polygon top{
            Vertex{0.25f, 1.75f},
            Vertex{2.f,   2.f  },
            Vertex{1.75f, 3.f  },
            Vertex{0.f,   2.75f},
        };

        auto manifold = makeContactManifold(Polys{&top, &bottom});

        REQUIRE(manifold.referenceIndex() == 1);
        REQUIRE(manifold.incidentIndex() == 0);

        REQUIRE(all(approx(Vec2{0, 1}, Vec2::filled(simu::EPSILON))
                        .contains(normalized(manifold.contactNormal))));

        REQUIRE(manifold.nContacts == 2);

        REQUIRE(inContacts(manifold.contacts[0], Vertex{0.25f, 1.75f}));
        REQUIRE(inContacts(manifold.contacts[0], Vertex{2.f, 2.f}));

        REQUIRE(inContacts(manifold.contacts[1], Vertex{0.25f, 2.f}));
        REQUIRE(inContacts(manifold.contacts[1], Vertex{2.f, 2.f}));
    }

    SECTION("Edge distance to origin")
    {
        // hard to find a test that will encounter this situation,
        //  but did happen in practice

        Vertices v{
            Vertex{-1,   1e-8},
            Vertex{1e-6, 1e-8},
            Vertex{1,    1e-8},
        };

        auto it = v.begin();

        Edges<Vertices>::Edge e1{it, it + 1};
        Edges<Vertices>::Edge e2{it + 1, it + 2};

        REQUIRE(e1.distanceToOrigin() < e2.distanceToOrigin());
    }
}
