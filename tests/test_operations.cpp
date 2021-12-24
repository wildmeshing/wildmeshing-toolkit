#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include <iostream>

using namespace wmtk;

TEST_CASE("edge_splitting", "[test_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE(mesh.split_edge(tuple, dummy));
    REQUIRE_FALSE(tuple.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("rollback_split_operation", "[test_operation]")
{
    class NoSplitMesh : public TetMesh
    {
    public:
        bool split_after(const TetMesh::Tuple& locs) override { return false; };
    };
    auto mesh = NoSplitMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE_FALSE(mesh.split_edge(tuple, dummy));
    REQUIRE(tuple.is_valid(mesh));
}