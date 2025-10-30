#include "Simu/ecs/Entity.hpp"
#include "catch2/catch_test_macros.hpp"
#include <cstddef>

#include "Simu/ecs/SparseSet.hpp"

using namespace simu;

TEST_CASE("SparseSet") {
    using Set = SparseSet<int>;
    Entity a{10};
    Entity b{11};
    Entity c{12};
    Entity d{13};

    SECTION("Single element") {
        Set s;
        s.add(a, 1);

        REQUIRE(s.size() == 1);
        REQUIRE(s.get_data(a) == 1);
        REQUIRE(s.has_data(a));

        std::optional<std::size_t> index = s.get_index(a);
        REQUIRE(index);
        auto pair = s.get_pair(*index);
        REQUIRE(pair.first == a);
        REQUIRE(pair.second == 1);

        pair.second.get() = 2;
        REQUIRE(s.get_data(a) == 2);

        s.get_data(a) = 3;
        REQUIRE(s.get_data(a) == 3);
    };

    SECTION("Removal") {
        Set s;
        s.add(a, 1);
        s.add(b, 2);
        s.add(c, 3);
        s.add(d, 4);

        REQUIRE(s.size() == 4);

        s.remove(a);
        REQUIRE(s.size() == 3);
        REQUIRE(s.get_index(d) == 0);
        REQUIRE(!s.get_index(a));

        s.remove(b);
        REQUIRE(s.size() == 2);
        REQUIRE(s.get_index(c) == 1);
        REQUIRE(!s.get_index(b));

        auto d_pair = s.get_pair(0);
        auto c_pair = s.get_pair(1);
        REQUIRE(d_pair.first == d);
        REQUIRE(d_pair.second == 4);
        REQUIRE(c_pair.first == c);
        REQUIRE(c_pair.second == 3);
    };
}
