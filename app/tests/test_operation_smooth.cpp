#include <TetWild.h>
#include <common.h>
#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include "spdlog/spdlog.h"

#include <spdlog/fmt/ostr.h>
using namespace wmtk;

TEST_CASE("smooth_in_single_tet", "[tetwild_operation]")
{
    using namespace tetwild;

    Parameters params;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    TetWild tetwild(params, envelope);
    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3d(0.1, 0, 0);
    vertices[1].m_posf = Vector3d(1, 0, 0);
    vertices[2].m_posf = Vector3d(0, 1, 0);
    vertices[3].m_posf = Vector3d(0, 0, 1);
    for (auto& v : vertices) {
        v.m_is_rounded = true;
        v.m_pos = tetwild::to_rational(v.m_posf);
    }
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    std::vector<TetAttributes> tet_attrs(1);

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    tetwild.smooth_all_vertices();
    tetwild.smooth_all_vertices();
    auto quality = tetwild.m_tet_attribute.m_attributes.front().m_quality;
    REQUIRE(quality == Approx(27.0));
}

TEST_CASE("smooth_double_tet", "[tetwild_operation]")
{
    using namespace tetwild;

    Parameters params;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    wmtk::Envelope envelope;
    TetWild tetwild(params, envelope);
    std::vector<VertexAttributes> vertices(5);
    vertices[0].m_posf = Vector3d(0.1, 0, 0);
    vertices[1].m_posf = Vector3d(1, 0, 0);
    vertices[2].m_posf = Vector3d(0, 1, 0);
    vertices[3].m_posf = Vector3d(0, 0, 1);
    vertices[4].m_posf = Vector3d(1, 1, 1);
    for (auto& v : vertices) v.m_is_rounded = true;
    std::vector<std::array<size_t, 4>> tets;
    tets.emplace_back(std::array<size_t, 4>{{0, 1, 2, 3}});
    tets.emplace_back(std::array<size_t, 4>{{1, 2, 3, 4}});
    std::vector<TetAttributes> tet_attrs(tets.size());

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    tetwild.smooth_all_vertices();
    tetwild.smooth_all_vertices();
    tetwild.smooth_all_vertices();
    auto quality = tetwild.m_tet_attribute.m_attributes.front().m_quality;
    REQUIRE(quality == Approx(27.0));
    auto quality2 = tetwild.m_tet_attribute.m_attributes[tetwild.tet_capacity()-1].m_quality;
    REQUIRE(quality2 == Approx(27.0));
}