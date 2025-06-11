#include <wmtk/TetMesh.h>
#include <catch2/catch_test_macros.hpp>
#include "wmtk/utils/Logger.hpp"

using namespace wmtk;

TEST_CASE("edge_splitting", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE(mesh.split_edge(tuple, dummy));
    REQUIRE_FALSE(tuple.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("edge_collapsing_impossible", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;

    // Cannot collapse the edge
    REQUIRE_FALSE(mesh.collapse_edge(tuple, dummy));
    REQUIRE(tuple.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("edge_collapsing", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> new_edge, dummy;
    REQUIRE(mesh.split_edge(tuple, new_edge));
    REQUIRE_FALSE(tuple.is_valid(mesh));

    // Cannot collapse the edge
    REQUIRE(mesh.collapse_edge(new_edge[1], dummy));
    for (const auto& e : new_edge) REQUIRE_FALSE(e.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("edge_collapsing_invalid", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(6, {{{0, 1, 2, 3}}, {{1, 2, 3, 4}}, {{1, 3, 4, 5}}});
    const auto tuple = mesh.tuple_from_edge(0, 1);
    std::vector<TetMesh::Tuple> new_edge, dummy;
    // Cannot collapse the edge
    REQUIRE(!mesh.collapse_edge(tuple, dummy));
}

TEST_CASE("tet_mesh_swap", "[tuple_operation]")
{
    TetMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});

    // SECTION ("3-2 swap")
    {
        const auto edges = mesh.get_edges();

        REQUIRE(edges.size() == 10);
        auto cnt_swap = 0;
        for (auto e : edges) {
            if (!e.is_valid(mesh)) continue;
            std::vector<TetMesh::Tuple> newt;
            if (mesh.swap_edge(e, newt)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
                REQUIRE(newt.front().is_valid(mesh));
            }
        }
        REQUIRE(mesh.check_mesh_connectivity_validity());
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.get_edges().size() == 9);
        REQUIRE(mesh.get_tets().size() == 2);
    }
    //
    // SECTION ("2-3 swap")
    {
        const auto faces = mesh.get_faces();
        REQUIRE(faces.size() == 7);
        auto cnt_swap = 0;
        for (auto e : faces) {
            if (!e.is_valid(mesh)) continue;
            std::vector<TetMesh::Tuple> newt;
            if (mesh.swap_face(e, newt)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
                REQUIRE(newt.front().is_valid(mesh));
            }
        }
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.check_mesh_connectivity_validity());
        REQUIRE(mesh.get_tets().size() == 3);
    }

    REQUIRE(mesh.tet_capacity() == 4);
    mesh.consolidate_mesh();
    REQUIRE(mesh.tet_capacity() == 3);
}

TEST_CASE("rollback_operation", "[tuple_operation]")
{
    class NoOperationMesh : public TetMesh
    {
    public:
        bool split_edge_after(const TetMesh::Tuple& locs) override { return false; };
        bool collapse_edge_after(const TetMesh::Tuple& locs) override { return false; };
        bool swap_edge_after(const TetMesh::Tuple& locs) override { return false; };
        bool swap_face_after(const TetMesh::Tuple& locs) override { return false; };
    };
    auto mesh = NoOperationMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    SECTION("split")
    {
        REQUIRE_FALSE(mesh.split_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
    }
    SECTION("swap")
    {
        REQUIRE_FALSE(mesh.swap_face(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
        REQUIRE_FALSE(mesh.swap_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
    }
    SECTION("collapse")
    {
        REQUIRE_FALSE(mesh.collapse_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
    }
}

TEST_CASE("forbidden-face-swap", "[tuple_operation]")
{
    /// https://i.imgur.com/aVCsOvf.png and 0,2,3 should not be swapped.
    /// Visualize V as
    //   [[ 0,  0, -1],
    //    [ 0,  0,  1],
    //    [ 1,  1,  0],
    //    [ 1, -1,  0],
    //    [ 2,  0,  0]]
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 3, 2, 4}}, {{1, 2, 3, 4}}, {{0, 1, 2, 3}}});
    auto t = mesh.tuple_from_tet(0);
    REQUIRE(t.vid(mesh) == 0);
    auto oppo = t.switch_vertex(mesh);
    REQUIRE(oppo.vid(mesh) == 3);
    REQUIRE(oppo.switch_edge(mesh).switch_vertex(mesh).vid(mesh) == 2);
    std::vector<TetMesh::Tuple> new_t;
    mesh.swap_face(t, new_t);
    REQUIRE(t.is_valid(mesh)); // operation rejected.
    REQUIRE(mesh.check_mesh_connectivity_validity());
}


TEST_CASE("tet_mesh_swap44", "[tuple_operation]")
{
    TetMesh mesh;
    // 01 is the common edge
    mesh.init(6, {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 2}}});

    {
        const auto edges = mesh.get_edges();

        REQUIRE(edges.size() == 13);
        auto cnt_swap = 0;
        for (auto e : edges) {
            if (!e.is_valid(mesh)) continue;
            std::vector<TetMesh::Tuple> newt;
            if (mesh.swap_edge_44(e, newt)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
                REQUIRE(newt.front().is_valid(mesh));
            }
        }
        REQUIRE(mesh.check_mesh_connectivity_validity());
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.get_edges().size() == 13);
        REQUIRE(mesh.get_tets().size() == 4);
    }
}

TEST_CASE("divide-tet-customized")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_tet(0);
    std::vector<TetMesh::Tuple> new_tups;

    struct DivideTet : public TetMesh::OperationBuilder
    {
        const TetMesh& m;
        TetMesh::Tuple tet;

        DivideTet(TetMesh& _m)
            : m(_m)
        {}
        std::vector<size_t> removed_tids(const TetMesh::Tuple& t)
        {
            tet = t;
            return {t.tid(m)};
        }
        int request_vert_slots() { return 1; }
        std::vector<std::array<size_t, 4>> replacing_tets(const std::vector<size_t>& slots)
        {
            assert(slots.size() == 1);
            auto ux = slots.front();

            std::array<size_t, 4> t_conn;
            auto vs = m.oriented_tet_vertices(tet);
            for (auto i = 0; i < 4; i++) t_conn[i] = vs[i].vid(m);
            auto new_tets = std::vector<std::array<size_t, 4>>(4, t_conn);
            for (auto i = 0; i < 4; i++) new_tets[i][i] = ux;

            return new_tets;
        }
    };
    auto op = DivideTet(mesh);
    mesh.customized_operation(op, tuple, new_tups);
    CHECK(mesh.tet_size() == 5);
}