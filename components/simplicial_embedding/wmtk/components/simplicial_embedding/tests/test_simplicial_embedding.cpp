#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simplicial_embedding/SimplicialEmbeddingTriMesh.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/io/VTUWriter.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::components::simplicial_embedding;
using namespace wmtk::utils::examples::tri;

TEST_CASE("write", "")
{
    TriMeshVF VF = wmtk::utils::examples::tri::edge_region();

    SimplicialEmbeddingTriMesh m;
    m.create_mesh(VF.F);
    m.set_positions(VF.V);

    // m.write_mesh("simplicial_embedding_out.vtu");

    io::VTUWriter writer(m);
    writer.write_triangles("simplicial_embedding_out.vtu");
}