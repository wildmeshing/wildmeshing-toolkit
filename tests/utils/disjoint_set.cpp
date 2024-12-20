#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/DisjointSet.hpp>


TEST_CASE("disjoint_set", "[disjoint_set]")
{
    wmtk::utils::DisjointSet ds(5);

    CHECK(ds.roots() == std::vector<size_t>{0,1,2,3,4});

    ds.merge(0,3);
    CHECK(ds.roots() == std::vector<size_t>{0,1,2,4});

    CHECK(ds.get_root(0) == 0);
    CHECK(ds.get_root(1) == 1);
    CHECK(ds.get_root(2) == 2);
    CHECK(ds.get_root(3) == 0);
    CHECK(ds.get_root(4) == 4);

    ds.merge(2,3);
    CHECK(ds.roots() == std::vector<size_t>{0,1,4});
    CHECK(ds.get_root(0) == 0);
    CHECK(ds.get_root(1) == 1);
    CHECK(ds.get_root(2) == 0);
    CHECK(ds.get_root(3) == 0);
    CHECK(ds.get_root(4) == 4);

    ds.merge(4,1);
    CHECK(ds.roots() == std::vector<size_t>{0,1});
    CHECK(ds.get_root(0) == 0);
    CHECK(ds.get_root(1) == 1);
    CHECK(ds.get_root(2) == 0);
    CHECK(ds.get_root(3) == 0);
    CHECK(ds.get_root(4) == 1);
}
