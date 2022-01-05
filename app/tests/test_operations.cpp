#include <TetWild.h>

#include <wmtk/TetMesh.h>

#include <catch2/catch.hpp>
#include "Logger.hpp"
#include "spdlog/common.h"


using namespace wmtk;
using namespace tetwild;

TEST_CASE("edge_splitting", "[test_operation]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3f(0, 0, 0), Vector3f(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);

    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3f(0, 0, 0);
    vertices[1].m_posf = Vector3f(1, 0, 0);
    vertices[2].m_posf = Vector3f(0, 1, 0);
    vertices[3].m_posf = Vector3f(0, 0, 1);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 3, 2}}};
    std::vector<TetAttributes> tet_attrs(1);

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    tetwild.split_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());

    tetwild.swap_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());
    tetwild.swap_all_faces();

    REQUIRE(tetwild.check_mesh_connectivity_validity());
}


TEST_CASE("edge_collapsing", "[test_operation]")
{
    Parameters params;
    params.lr = 1 / 20.;
    params.init(Vector3f(0, 0, 0), Vector3f(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);
    {
        std::vector<VertexAttributes> vertices(4);
        vertices[0].m_posf = Vector3f(0, 0, 0);
        vertices[1].m_posf = Vector3f(1, 0, 0);
        vertices[2].m_posf = Vector3f(0, 1, 0);
        vertices[3].m_posf = Vector3f(0, 0, 1);
        std::vector<std::array<size_t, 4>> tets = {{{0, 1, 3, 2}}};
        std::vector<TetAttributes> tet_attrs(1);

        tetwild.init(vertices.size(), tets);
        tetwild.create_mesh_attributes(vertices, tet_attrs);
    }

    tetwild.split_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());

    tetwild.collapse_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());

    REQUIRE(tetwild.tet_capacity() == 5617);
    for (auto& t : tetwild.get_tets()) {
        REQUIRE_FALSE(tetwild.is_inverted(t));
    }
    tetwild.consolidate_mesh_connectivity();
    auto n_tet_after = tetwild.get_tets().size();
    auto n_verts_after = tetwild.get_vertices().size();
    REQUIRE(n_tet_after == 4809);
    REQUIRE(tetwild.tet_capacity() == n_tet_after);
    REQUIRE(tetwild.m_tet_attribute.size() == n_tet_after);
    REQUIRE(tetwild.check_mesh_connectivity_validity());
}
