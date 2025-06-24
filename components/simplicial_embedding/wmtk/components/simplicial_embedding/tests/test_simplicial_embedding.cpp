#include <catch2/catch_test_macros.hpp>

#include <random>
#include <wmtk/components/simplicial_embedding/SimplicialEmbeddingTriMesh.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/io/TriVTUWriter.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::components::simplicial_embedding;
using namespace wmtk::utils::examples::tri;


TEST_CASE("write", "[simplicial_embedding][.]")
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

TEST_CASE("tri_single_triangle_tags", "[simplicial_embedding][TriMesh]")
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
        // m.write("tri_single_triangle_1");
    }
    SECTION("untagged_edge")
    {
        m.set_edge_tag(m.tuple_from_edge(1, 2, 0), 1);
        m.set_edge_tag(m.tuple_from_edge(2, 0, 0), 1);
        // m.write("tri_single_triangle_0");
        REQUIRE(m.split_edge(t, dummy));
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
        // m.write("tri_single_triangle_1");
    }
}

TEST_CASE("tri_two_triangles_tags", "[simplicial_embedding][TriMesh]")
{
    using Tuple = TriMesh::Tuple;

    TriMeshVF VF = two_triangles();


    SimplicialEmbeddingTriMesh m;
    m.init(VF.F);
    m.set_positions(VF.V);

    const Tuple t = m.tuple_from_vids(0, 2, 1);
    std::vector<Tuple> dummy;

    SECTION("boundary_tagged")
    {
        auto check_tags = [&m]() {
            for (const Tuple& e : m.get_edges()) {
                const size_t eid = e.eid(m);
                if (m.is_boundary_edge(e)) {
                    CHECK(m.edge_attrs[eid].tag == 1);
                } else {
                    CHECK(m.edge_attrs[eid].tag == 0);
                }
            }
        };

        for (const Tuple& e : m.get_edges()) {
            if (m.is_boundary_edge(e)) {
                m.set_edge_tag(e, 1);
            }
        }
        // m.write("tri_single_triangle_0");
        REQUIRE(m.split_edge(t, dummy));
        check_tags();

        // randomly split edges
        std::random_device random_device;
        std::mt19937 engine{random_device()};
        for (int i = 0; i < 10; ++i) {
            const auto es = m.get_edges();

            std::uniform_int_distribution<int> dist(0, es.size() - 1);
            const Tuple rnd_edge = es[dist(engine)];

            REQUIRE(m.split_edge(rnd_edge, dummy));
            check_tags();
        }
        // m.write("tri_single_triangle_1");
    }
    SECTION("face_tagged")
    {
        m.set_face_tag(m.tuple_from_tri(0), 1);
        // m.write("tri_single_triangle_0");
        REQUIRE(m.split_edge(t, dummy));
        CHECK(m.face_attrs[m.tuple_from_vids(0, 1, 4).fid(m)].tag == 1);
        CHECK(m.face_attrs[m.tuple_from_vids(2, 1, 4).fid(m)].tag == 1);
        CHECK(m.face_attrs[m.tuple_from_vids(0, 3, 4).fid(m)].tag == 0);
        CHECK(m.face_attrs[m.tuple_from_vids(2, 3, 4).fid(m)].tag == 0);
        // m.write("tri_single_triangle_1");
    }
}

TEST_CASE("tri_face_split_tags", "[simplicial_embedding][TriMesh]")
{
    using Tuple = TriMesh::Tuple;

    SimplicialEmbeddingTriMesh m;
    {
        TriMeshVF VF = edge_region();
        m.init(VF.F);
        m.set_positions(VF.V);

        const Tuple f0 = m.tuple_from_edge(3, 4, 0);
        m.set_face_tag(f0, 1);

        const Tuple f4 = m.tuple_from_edge(5, 6, 4);
        m.set_edge_tag(f4, 1);

        const Tuple f7 = m.tuple_from_edge(8, 4, 7);
        m.set_vertex_tag(f7, 1);

        m.set_edge_tag(m.tuple_from_vids(3, 7, 4), 1);
        m.set_edge_tag(m.tuple_from_vids(4, 7, 3), 1);
    }
    // m.write("tri_simplicial_emedding_0");
    std::vector<Tuple> new_tris;
    for (int i = 0; i < 10; ++i) {
        m.split_face(m.tuple_from_tri(i), new_tris);
    }
    REQUIRE(m.get_vertices().size() == 20);
    std::vector<int64_t> v_expected_tags(30, 0);
    v_expected_tags[0] = 1;
    v_expected_tags[3] = 1;
    v_expected_tags[4] = 1;
    v_expected_tags[5] = 1;
    v_expected_tags[6] = 1;
    v_expected_tags[7] = 1;
    v_expected_tags[8] = 1;
    v_expected_tags[10] = 1;
    for (int i = 0; i < 20; ++i) {
        CHECK(m.vertex_attrs[i].tag == v_expected_tags[i]);
    }

    REQUIRE(m.get_faces().size() == 30);
    std::vector<int64_t> f_expected_tags(30, 0);
    f_expected_tags[0] = 1;
    f_expected_tags[10] = 1;
    f_expected_tags[11] = 1;
    for (int i = 0; i < 30; ++i) {
        CHECK(m.face_attrs[i].tag == f_expected_tags[i]);
    }

    // m.write("tri_simplicial_emedding_1");
}

TEST_CASE("tri_simplicial_embedding", "[simplicial_embedding][TriMesh][.]")
{
    using Tuple = TriMesh::Tuple;

    SimplicialEmbeddingTriMesh m;
    {
        TriMeshVF VF = edge_region();
        m.init(VF.F);
        m.set_positions(VF.V);

        const Tuple f0 = m.tuple_from_edge(3, 4, 0);
        m.set_face_tag(f0, 1);

        const Tuple f4 = m.tuple_from_edge(5, 6, 4);
        m.set_edge_tag(f4, 1);

        const Tuple f7 = m.tuple_from_edge(8, 4, 7);
        m.set_vertex_tag(f7, 1);

        m.set_edge_tag(m.tuple_from_vids(3, 7, 4), 1);
        m.set_edge_tag(m.tuple_from_vids(4, 7, 3), 1);
    }
    // for (const Tuple& t : m.get_edges()) {
    //     m.set_edge_tag(t, 1);
    // }

    m.write("tri_simplicial_emedding_0");
    m.edge_split_simplicial_embedding();
    m.write("tri_simplicial_emedding_1");
    m.face_split_simplicial_embedding();
    m.write("tri_simplicial_emedding_2");
}