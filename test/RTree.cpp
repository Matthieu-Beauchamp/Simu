#include "catch2/catch_test_macros.hpp"

#include "Simu/physics/RTree.hpp"
#include "Simu/math/Random.hpp"

using namespace simu;

typedef RTree<int> Tree;

template <class T>
void testEmpty(const RTree<T>& t)
{
    REQUIRE(t.size() == 0);
    REQUIRE(t.isEmpty());
    REQUIRE(t.bounds() == BoundingBox{});
}

struct TestType
{
    bool hit = false;
};

typedef RTree<TestType> TestTree;


void hit(TestTree::iterator it) { it->hit = true; }

TEST_CASE("R-Tree")
{
    SECTION("Empty tree")
    {
        Tree t{};
        REQUIRE(t.begin() == t.end());
        REQUIRE(t.height() == 0);
    }

    SECTION("Iterator conversions")
    {
        Tree        t{};
        const Tree& ct = static_cast<const Tree&>(t);

        Tree::iterator       ii{t.begin()};
        Tree::const_iterator cici{ct.begin()};
        Tree::const_iterator cii{t.begin()};

        REQUIRE(ii == cici);
        REQUIRE(ii == cii);

        STATIC_REQUIRE_FALSE(
            std::is_constructible<Tree::iterator, Tree::const_iterator>::value
        );
    }

    SECTION("Insertion")
    {
        RTree<TestType*> t{};

        // must be a power of 2 for height checks
        constexpr Uint32 size = 16;

        std::vector<TestType> v{size};

        for (Uint32 i = 0; i < size; ++i)
            t.insert(BoundingBox{Vec2::filled((float)i), Vec2::filled(i + 1.f)}, &v[i]);


        for (TestType* test : t)
            test->hit = true;

        for (const TestType& test : v)
            REQUIRE(test.hit);

        REQUIRE_FALSE(t.isEmpty());
        REQUIRE(t.size() == size);
        REQUIRE(t.bounds() == BoundingBox{Vec2{0,0}, Vec2{size, size}});

        REQUIRE(t.height() < size - 1);
    }

    SECTION("Removal")
    {
        Tree          t{};
        constexpr int size = 16;

        for (int i = 0; i < size; ++i)
            t.insert(BoundingBox{Vec2::filled((float)i), Vec2::filled(i + 1.f)}, i);

        REQUIRE(t.bounds() == BoundingBox{Vec2::filled(0), Vec2::filled(size)});

        {
            auto it = t.begin();
            while (it != t.end())
            {
                if (*it & 1)
                    it = t.erase(it);
                else
                    ++it;
            }

            REQUIRE(t.size() == size / 2);
            REQUIRE(
                t.bounds() == BoundingBox{Vec2::filled(0), Vec2::filled(size - 1)}
            );
        }
        {
            auto it = t.begin();
            while (it != t.end())
                it = t.erase(it);

            testEmpty(t);
        }

        for (int i = 0; i < size; ++i)
            t.insert(BoundingBox{Vec2::filled((float)i), Vec2::filled(i + 1.f)}, i);

        t.clear();
        testEmpty(t);
    }

    SECTION("Box queries")
    {
        TestTree t{};

        // clang-format off
        auto topleft  = t.insert(BoundingBox{Vec2{-2,  1}, Vec2{-1,  2}}, {});
        auto topright = t.insert(BoundingBox{Vec2{ 1,  1}, Vec2{ 2,  2}}, {});
        auto botleft  = t.insert(BoundingBox{Vec2{-2, -2}, Vec2{-1, -1}}, {});
        auto botright = t.insert(BoundingBox{Vec2{ 1, -2}, Vec2{ 2, -1}}, {});
        // clang-format on


        auto testHits = [&](bool tl, bool tr, bool bl, bool br) {
            REQUIRE(topleft->hit == tl);
            REQUIRE(topright->hit == tr);
            REQUIRE(botleft->hit == bl);
            REQUIRE(botright->hit == br);
        };

        t.forEachAt(Vec2{0, 0}, hit);
        testHits(false, false, false, false);

        t.forEachAt(Vec2{-1.5f, 1.5f}, hit);
        testHits(true, false, false, false);
        topleft->hit = false;

        t.forEachAt(Vec2{1.5f, 1.5f}, hit);
        testHits(false, true, false, false);
        topright->hit = false;

        t.forEachAt(Vec2{-1.5f, -1.5f}, hit);
        testHits(false, false, true, false);
        botleft->hit = false;

        t.forEachAt(Vec2{1.5f, -1.5f}, hit);
        testHits(false, false, false, true);
        botright->hit = false;


        t.forEachIn(t.bounds(), hit);
        testHits(true, true, true, true);
    }

    SECTION("Update")
    {
        TestTree t{};

        BoundingBox b{
            Vec2{0, 0},
            Vec2{1, 1}
        };

        // clang-format off
        auto left   = t.insert(BoundingBox{Vec2{0, 0}, Vec2{1, 1}}, {});
        auto center = t.insert(BoundingBox{Vec2{1.1f, 1.1f}, Vec2{1.9f, 1.9f}}, {});
        auto right  = t.insert(BoundingBox{Vec2{2, 2}, Vec2{3, 3}}, {});
        // clang-format on

        REQUIRE(t.bounds() == BoundingBox{Vec2{0, 0}, Vec2{3, 3}});

        t.update(left, center.bounds());
        t.update(right, center.bounds());
        REQUIRE(t.bounds() == center.bounds());
        t.forEachIn(center.bounds(), hit);
        REQUIRE(left->hit);
        REQUIRE(center->hit);
        REQUIRE(right->hit);
    }

    SECTION("Batch operations")
    {
        // TODO: 
    }
}