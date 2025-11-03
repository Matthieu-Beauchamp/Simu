#include "Simu/entities/Entities.hpp"
#include "Simu/entities/Entity.hpp"
#include "catch2/catch_test_macros.hpp"

using namespace simu;


TEST_CASE("Entities") {
    Entities<int> entities;

    Entity a = entities.create();
    Entity b = entities.create();
    Entity c = entities.create();

    entities.add(a, 1);
    entities.add(b, 2);
    entities.add(c, 3);

    SECTION("Single component query") {
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
