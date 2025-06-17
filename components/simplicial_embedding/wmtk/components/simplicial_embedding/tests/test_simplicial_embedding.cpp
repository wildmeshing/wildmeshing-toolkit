#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simplicial_embedding/SimplicialEmbeddingTriMesh.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::components::simplicial_embedding;
using namespace wmtk::utils::examples::tri;

TEST_CASE("tetwild_file_write", "[tetwild_operation][.]")
{
    TriMeshVF VF = wmtk::utils::examples::tri::edge_region();

    SimplicialEmbeddingTriMesh m;
    m.create_mesh(VF.F);
    m.set_positions(VF.V);
}