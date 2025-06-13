#include <catch2/catch_test_macros.hpp>
#include "sec/envelope/SampleEnvelope.hpp"

#include <wmtk/components/tetwild/IncrementalTetWild.h>

#include <wmtk/TetMesh.h>
#include <wmtk/components/tetwild/common.h>

using namespace wmtk;
using namespace tetwild;

TEST_CASE("tetwild_file_write", "[tetwild_operation][.]")
{
    // Parameters params;
    //params.lr = 1 / 5.;
    // params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
    //
    // wmtk::Envelope envelope;
    // TetWild tetwild(params, envelope);
    //
    // std::vector<VertexAttributes> vertices(4);
    // vertices[0].m_posf = Vector3d(0, 0, 0);
    // vertices[1].m_posf = Vector3d(1, 0, 0);
    // vertices[2].m_posf = Vector3d(0, 1, 0);
    // vertices[3].m_posf = Vector3d(0, 0, 1);
    // std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    // std::vector<TetAttributes> tet_attrs(1);
    // for (auto& v : vertices) {
    //     v.m_is_rounded = true;
    //     v.m_pos = tetwild::to_rational(v.m_posf);
    // }
    //
    // tetwild.init(vertices.size(), tets);
    // tetwild.create_mesh_attributes(vertices, tet_attrs);
    // tetwild.output_mesh("test_save.msh");
}