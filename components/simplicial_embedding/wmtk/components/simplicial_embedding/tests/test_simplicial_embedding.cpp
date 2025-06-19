#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simplicial_embedding/SimplicialEmbeddingTriMesh.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/io/TriVTUWriter.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::components::simplicial_embedding;
using namespace wmtk::utils::examples::tri;

TEST_CASE("write", "")
{
    TriMeshVF VF = wmtk::utils::examples::tri::edge_region();

    SimplicialEmbeddingTriMesh m;
    m.init(VF.F);
    m.set_positions(VF.V);

    io::TriVTUWriter writer(m);
    writer.add_vertex_positions([&m](int i) { return m.vertex_attrs[i].pos; });
    writer.add_vertex_attribute("position", [&m](int i) { return m.vertex_attrs[i].pos; });
    writer.add_vertex_attribute("vid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.add_triangle_attribute("fid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.add_edge_attribute("eid", [&m](int i) { return VectorXd::Constant(1, i); });
    writer.write_triangles("simplicial_embedding_f_out.vtu");
    writer.write_edges("simplicial_embedding_e_out.vtu");
}