#include "Simu/entities/Entity.hpp"
#include "catch2/catch_test_macros.hpp"
#include <cstddef>

#include "Simu/entities/SparseSet.hpp"

using namespace simu;

TEST_CASE("SparseSet") {
    using Set = SparseSet<int>;
    internal::EntityGenerator gen;

    Entity a = gen.create();
    Entity b = gen.create();
    Entity c = gen.create();
    Entity d = gen.create();

    SECTION("Single element") {
        Set s;
        s.add(a, 1);

        REQUIRE(s.size() == 1);
        REQUIRE(s.get_data(a) == 1);
        REQUIRE(s.has_entity(a));

        std::optional<std::size_t> index = s.index_of(a);
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
        REQUIRE(s.index_of(d) == 0);
        REQUIRE(!s.index_of(a));

        s.remove(b);
        REQUIRE(s.size() == 2);
        REQUIRE(s.index_of(c) == 1);
        REQUIRE(!s.index_of(b));

        auto d_pair = s.get_pair(0);
        auto c_pair = s.get_pair(1);
        REQUIRE(d_pair.first == d);
        REQUIRE(d_pair.second == 4);
        REQUIRE(c_pair.first == c);
        REQUIRE(c_pair.second == 3);
    };

    SECTION("Iterator") {
        Set s;
        s.add(a, 1);
        s.add(b, 2);
        s.add(c, 3);
        s.add(d, 4);

        auto it  = s.begin();
        auto end = s.end();

        REQUIRE(it.get_entity() == a);
        REQUIRE(*it == 1);
        ++it;

        REQUIRE(it.get_entity() == b);
        REQUIRE(*it == 2);
        ++it;

        REQUIRE(it.get_entity() == c);
        REQUIRE(*it == 3);
        ++it;

        REQUIRE(it.get_entity() == d);
        REQUIRE(*it == 4);
        ++it;

        REQUIRE(it == end);
    }

    SECTION("Iterator requirements") {
        STATIC_REQUIRE(std::random_access_iterator<SparseSet<int>::Iterator<false>>);

        // STATIC_REQUIRE( std::contiguous_iterator<SparseSet<int>::ZippedIterator<false>>);
    }
}
