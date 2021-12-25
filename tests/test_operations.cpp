#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>

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

TEST_CASE("tet_mesh_swap", "[test_operation]")
{
    TetMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});

    // ("3-2 swap")
    {
        const auto edges = mesh.get_edges();

        REQUIRE(edges.size() == 10);
        auto cnt_swap = 0;
        for (auto e : edges) {
            if (!e.is_valid(mesh)) continue;
            if (mesh.swap_edge(e)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
            }
        }
        REQUIRE(mesh.check_mesh_connectivity_validity());
        auto temp = mesh.get_edges();
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.get_edges().size() == 9);
    }
    //
    // ("2-3 swap")
    {
        const auto faces = mesh.get_faces();
        REQUIRE(faces.size() == 7);
        auto cnt_swap = 0;
        for (auto e : faces) {
            if (!e.is_valid(mesh)) continue;
            if (mesh.swap_face(e)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
            }
        }
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.check_mesh_connectivity_validity());
    }
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