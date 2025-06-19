#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simplicial_embedding/SimplicialEmbeddingTriMesh.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/io/TriVTUWriter.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::components::simplicial_embedding;
using namespace wmtk::utils::examples::tri;


TEST_CASE("write", "[simplicial_embedding]")
{
    using Tuple = TriMesh::Tuple;

    TriMeshVF VF = wmtk::utils::examples::tri::edge_region();

    SimplicialEmbeddingTriMesh m;
    m.init(VF.F);
    m.set_positions(VF.V);

    const Tuple f0 = m.tuple_from_edge(3, 4, 0);
    m.set_face_tag(f0, 1);

    const Tuple f4 = m.tuple_from_edge(5, 6, 4);
    m.set_edge_tag(f4, 1);

    const Tuple f7 = m.tuple_from_edge(8, 4, 7);
    m.set_vertex_tag(f7, 1);

    m.write("simplicial_emedding");
}

TEST_CASE("tri_single_triangle", "[simplicial_embedding][TriMesh]")
{
    using Tuple = TriMesh::Tuple;

    TriMeshVF VF = single_triangle();

    SimplicialEmbeddingTriMesh m;
    m.init(VF.F);
    m.set_positions(VF.V);

    const Tuple t = m.tuple_from_edge(0, 1, 0);
    std::vector<Tuple> dummy;

    SECTION("tagged_edge")
    {
        m.set_edge_tag(t, 1);

        // m.write("tri_single_triangle_0");
        REQUIRE(m.split_edge(t, dummy));
        // m.write("tri_single_triangle_1");
        CHECK(m.vertex_attrs[0].tag == 1);
        CHECK(m.vertex_attrs[1].tag == 1);
        CHECK(m.vertex_attrs[2].tag == 0);
        CHECK(m.vertex_attrs[3].tag == 1);
        CHECK(m.edge_attrs[m.tuple_from_vids(0, 3, 2).eid(m)].tag == 1); // spine left
        CHECK(m.edge_attrs[m.tuple_from_vids(3, 1, 2).eid(m)].tag == 1); // spine right
        CHECK(m.edge_attrs[m.tuple_from_vids(2, 0, 3).eid(m)].tag == 0); // left
        CHECK(m.edge_attrs[m.tuple_from_vids(2, 1, 3).eid(m)].tag == 0); // right
        CHECK(m.edge_attrs[m.tuple_from_vids(3, 2, 0).eid(m)].tag == 0); // mid
        CHECK(m.face_attrs[0].tag == 0);
        CHECK(m.face_attrs[1].tag == 0);
    }
    SECTION("untagged_edge")
    {
        m.set_edge_tag(m.tuple_from_edge(1, 2, 0), 1);
        m.set_edge_tag(m.tuple_from_edge(2, 0, 0), 1);
        m.write("tri_single_triangle_0");
        REQUIRE(m.split_edge(t, dummy));
        m.write("tri_single_triangle_1");
        CHECK(m.vertex_attrs[0].tag == 1);
        CHECK(m.vertex_attrs[1].tag == 1);
        CHECK(m.vertex_attrs[2].tag == 1);
        CHECK(m.vertex_attrs[3].tag == 0);
        CHECK(m.edge_attrs[m.tuple_from_vids(0, 3, 2).eid(m)].tag == 0); // spine left
        CHECK(m.edge_attrs[m.tuple_from_vids(3, 1, 2).eid(m)].tag == 0); // spine right
        CHECK(m.edge_attrs[m.tuple_from_vids(2, 0, 3).eid(m)].tag == 1); // left
        CHECK(m.edge_attrs[m.tuple_from_vids(2, 1, 3).eid(m)].tag == 1); // right
        CHECK(m.edge_attrs[m.tuple_from_vids(3, 2, 0).eid(m)].tag == 0); // mid
        CHECK(m.face_attrs[0].tag == 0);
        CHECK(m.face_attrs[1].tag == 0);
    }
}