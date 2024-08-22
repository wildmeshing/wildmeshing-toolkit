#include <stdlib.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/Tuple.hpp>

using namespace wmtk;

TEST_CASE("tuple_is_null", "[tuple]")
{
    Tuple a(0, 0, 0, 0);
    CHECK(!a.is_null());

    Tuple b(0, 0, 0, -1);
    CHECK(b.is_null());

    Tuple c(-1, -1, -1, -1);
    CHECK(c.is_null());

    Tuple d(-1, -1, -1, 0);
    CHECK(!d.is_null());

    Tuple e(-1, -1, -1, -2);
    CHECK(!e.is_null());
}
TEST_CASE("tuple_comparison", "[tuple]")
{
    { // check that equals works
        Tuple a(0, 0, 0, 0);
        Tuple b(0, 0, 0, 0);
        CHECK(a == b);
        CHECK(a.same_ids(b));
    }
    { // check that diff ids are equivalent
        {
            Tuple a(1, 0, 0, 0);
            Tuple b(0, 0, 0, 0);
            CHECK(a != b);
            CHECK(!a.same_ids(b));
        }
        {
            Tuple a(0, 1, 0, 0);
            Tuple b(0, 0, 0, 0);
            CHECK(a != b);
            CHECK(!a.same_ids(b));
        }
        {
            Tuple a(0, 0, 1, 0);
            Tuple b(0, 0, 0, 0);
            CHECK(a != b);
            CHECK(!a.same_ids(b));
        }
        {
            Tuple a(0, 0, 0, 1);
            Tuple b(0, 0, 0, 0);
            CHECK(a != b);
            CHECK(!a.same_ids(b));
        }
    }
}
