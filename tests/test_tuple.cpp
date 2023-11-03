#include <stdlib.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/Tuple.hpp>

using namespace wmtk;

TEST_CASE("tuple_comparison", "[tuple]")
{
    { // check that equals are detected properly
        Tuple a(0, 0, 0, 0, 0);
        Tuple b(0, 0, 0, 0, 0);
        CHECK(a == b);
        CHECK(a.same_ids(b));
    }
    {// check that diff ids are equivalent
     {Tuple a(1, 0, 0, 0, 0);
    Tuple b(0, 0, 0, 0, 0);
    CHECK(a != b);
    CHECK(!a.same_ids(b));
}
{
    Tuple a(0, 1, 0, 0, 0);
    Tuple b(0, 0, 0, 0, 0);
    CHECK(a != b);
    CHECK(!a.same_ids(b));
}
{
    Tuple a(0, 0, 1, 0, 0);
    Tuple b(0, 0, 0, 0, 0);
    CHECK(a != b);
    CHECK(!a.same_ids(b));
}
{
    Tuple a(0, 0, 0, 1, 0);
    Tuple b(0, 0, 0, 0, 0);
    CHECK(a != b);
    CHECK(!a.same_ids(b));
}
}
{
    Tuple a(0, 0, 0, 0, 1);
    Tuple b(0, 0, 0, 0, 0);
    CHECK(a != b);
    CHECK(a.same_ids(b));
}
}
