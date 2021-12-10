#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include "../app/TetWild.h"

#include <spdlog/fmt/ostr.h>
using namespace wmtk;

TEST_CASE("smooth_vertex", "[test_operation]")
{
    using namespace tetwild;

    Parameters params;
    params.init();

    Envelope envelope;
    TetWild tetwild(params, envelope);
    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3f(0.1, 0, 0);
    vertices[1].m_posf = Vector3f(1, 0, 0);
    vertices[2].m_posf = Vector3f(0, 1, 0);
    vertices[3].m_posf = Vector3f(0, 0, 1);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 3, 2}}};
    std::vector<TetAttributes> tet_attrs(1);

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    tetwild.smooth_all_vertices();
    auto quality = tetwild.m_tet_attribute.front().m_qualities;
    REQUIRE(quality == Approx(3.0));
}