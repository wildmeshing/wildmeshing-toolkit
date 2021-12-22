#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include <iostream>

using namespace wmtk;

TEST_CASE("edge_splitting", "[test_operation]")
{
    // TODO
}

TEST_CASE("rollback_split_operation", "[test_operation]")
{
    class NoSplitMesh : public TetMesh
    {
    public:
        bool split_after(const std::vector<TetMesh::Tuple>& locs) override { return false; };
    };
    auto mesh = NoSplitMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE_FALSE(mesh.split_edge(tuple, dummy));
    REQUIRE(tuple.is_valid(mesh));
}