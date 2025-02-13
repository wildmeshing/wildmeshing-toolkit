#include <stdlib.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <sstream>
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

TEST_CASE("tuple_ids", "[tuple]")
{
    Tuple a(0, 1, 2, 3);
    CHECK(a.local_vid() == 0);
    CHECK(a.local_eid() == 1);
    CHECK(a.local_fid() == 2);
    CHECK(a.global_cid() == 3);

    Tuple b;
    CHECK(b.local_vid() == -1);
    CHECK(b.local_eid() == -1);
    CHECK(b.local_fid() == -1);
    CHECK(b.global_cid() == -1);
}

TEST_CASE("tuple_as_string", "[tuple]")
{
    const Tuple t(0, 1, 2, 3);
    const std::string expected = "(gid 3 : lids[v0,e1,f2])";

    CHECK(t.as_string() == expected);
    CHECK(static_cast<std::string>(t) == expected);

    std::stringstream ss;
    ss << t;
    CHECK(ss.str() == expected);
}