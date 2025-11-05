#include "Simu/entities/Entities.hpp"
#include "Simu/entities/Entity.hpp"
#include "catch2/catch_test_macros.hpp"

using namespace simu;


TEST_CASE("Entities") {
    SECTION("Single component") {
        Entities<int> entities;

        Entity a = entities.create();
        Entity b = entities.create();
        Entity c = entities.create();

        entities.add(a, 1);
        entities.add(b, 2);
        entities.add(c, 3);

        SECTION("query") {
            int current = 1;
            for (int& component : entities.query<int>()) {
                REQUIRE(current++ == component);
            }
            REQUIRE(current == 4);

            const Entities<int>& c_entities = entities;
            current                         = 1;
            for (const int& component : c_entities.query<int>()) {
                REQUIRE(current++ == component);
            }
            REQUIRE(current == 4);
        };

        SECTION("callback iteration") {
            int current = 1;
            entities.query<int>().each([&](int& x) { REQUIRE(current++ == x); });
            REQUIRE(current == 4);

            const Entities<int>& c_entities = entities;
            current                         = 1;
            c_entities.query<int>().each([&](const int& x) {
                REQUIRE(current++ == x);
            });
            REQUIRE(current == 4);
        };

        SECTION("callback zipped iteration") {
            int current = 1;
            entities.query<int>().each([&](Entity e, int& x) {
                switch (current++) {
                    case 1:
                        REQUIRE(x == 1);
                        REQUIRE(e == a);
                        break;
                    case 2:
                        REQUIRE(x == 2);
                        REQUIRE(e == b);
                        break;
                    case 3:
                        REQUIRE(x == 3);
                        REQUIRE(e == c);
                        break;
                }
            });
            REQUIRE(current == 4);
        };
    }

    SECTION("Multi component") {
        struct X
        {
            int x;
        };
        struct Y
        {
            int y;
        };

        Entities<X, Y> entities;

        Entity a = entities.create();
        Entity b = entities.create();
        Entity c = entities.create();

        entities.add(a, X{1});

        entities.add(b, X{2});
        entities.add(b, Y{2});

        entities.add(c, X{3});

        SECTION("destroy") {
            entities.destroy(b);
            REQUIRE_FALSE(entities.get_component<X>().has_entity(b));
            REQUIRE_FALSE(entities.get_component<Y>().has_entity(b));
        }

        SECTION("callback iteration") {
            auto query = entities.query<X, Y>();
            query.each([](X& x, Y& y) {
                REQUIRE(x.x == 2);
                REQUIRE(y.y == 2);
            });

            entities.remove<Y>(b);
            entities.add<Y>(a, Y{1});

            query = entities.query<X, Y>();
            query.each([](X& x, Y& y) {
                REQUIRE(x.x == 1);
                REQUIRE(y.y == 1);
            });
        }

        SECTION("callback zipped iteration") {
            entities.add<Y>(a, Y{1});
            entities.add<Y>(c, Y{3});

            int current = 1;
            entities.query<X, Y>().each([&](Entity e, X& x, Y& y) {
                switch (current++) {
                    case 1:
                        REQUIRE(y.y == 1);
                        REQUIRE(x.x == 1);
                        REQUIRE(e == a);
                        break;
                    case 2:
                        REQUIRE(y.y == 2);
                        REQUIRE(x.x == 2);
                        REQUIRE(e == b);
                        break;
                    case 3:
                        REQUIRE(y.y == 3);
                        REQUIRE(x.x == 3);
                        REQUIRE(e == c);
                        break;
                }
            });

            REQUIRE(current == 4);
        };
    }
}
