#include <catch2/catch.hpp>

#include <TetWild.h>

#include <wmtk/TetMesh.h>


using namespace wmtk;
using namespace tetwild;

TEST_CASE("tetwild_file_write", "[tetwild_operation]")
{
    Parameters params;
    params.lr = 1 / 5.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);

    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(1, 0, 0);
    vertices[2].m_posf = Vector3d(0, 1, 0);
    vertices[3].m_posf = Vector3d(0, 0, 1);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v:vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);
    tetwild.split_all_edges();
    tetwild.collapse_all_edges();
    tetwild.output_mesh("test_save.msh");
}