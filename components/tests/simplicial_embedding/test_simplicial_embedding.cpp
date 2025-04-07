#include <catch2/catch_test_macros.hpp>

#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk;
using namespace components;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE(
    "simplicial_embedding_component_tri",
    "[components][simplicial_embedding][trimesh][2D][scheduler]")
{
    tests::DEBUG_TriMesh m = wmtk::tests::hex_plus_two_with_position();

    SimplicialEmbeddingOptions options;
    options.value = 1;
    options.tag_attributes[PrimitiveType::Vertex] =
        m.register_attribute<int64_t>("vertex_tag", wmtk::PrimitiveType::Vertex, 1);
    options.tag_attributes[PrimitiveType::Edge] =
        m.register_attribute<int64_t>("edge_tag", wmtk::PrimitiveType::Edge, 1);
    options.tag_attributes[PrimitiveType::Triangle] =
        m.register_attribute<int64_t>("face_tag", wmtk::PrimitiveType::Triangle, 1);
    options.pass_through_attributes.emplace_back(
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));

    SECTION("points_in_2d_case")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /  .
        //    7---8
        // set 0 1 4 5 6
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            attribute::Accessor<int64_t> acc_vertex_tag =
                m.create_accessor<int64_t>(options.tag_attributes[PrimitiveType::Vertex]);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[6]) = options.value;
        }

        simplicial_embedding::simplicial_embedding(m, options);

        CHECK(m.get_all(PrimitiveType::Vertex).size() == 15);

        if (false) {
            wmtk::io::ParaviewWriter writer(
                data_dir / "simplicial_embedding_result_0d_case",
                "vertices",
                m,
                true,
                true,
                true,
                false);
            m.serialize(writer);
        }
    }
    SECTION("edges_in_2d_case")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /  .
        //    7---8
        // set vertex 0 1 4 5 6 7
        // set edge 4-5 5-1 1-4 4-7 7-3
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            attribute::Accessor<int64_t> acc_vertex_tag =
                m.create_accessor<int64_t>(options.tag_attributes[PrimitiveType::Vertex]);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[6]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[7]) = options.value;
            attribute::Accessor<int64_t> acc_edge_tag =
                m.create_accessor<int64_t>(options.tag_attributes[PrimitiveType::Edge]);
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(4, 5, 2)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(5, 1, 2)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(1, 4, 2)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(7, 4, 5)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(7, 3, 5)) = options.value;
        }

        simplicial_embedding::simplicial_embedding(m, options);

        CHECK(m.get_all(PrimitiveType::Triangle).size() == 17);
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 15);

        if (false) {
            ParaviewWriter writer(
                data_dir / "simplicial_embedding_result_1d_case",
                "vertices",
                m,
                true,
                true,
                true,
                false);
            m.serialize(writer);
        }
    }
}

TEST_CASE(
    "simplicial_embedding_component_tet",
    "[components][simplicial_embedding][tetmesh][3D][scheduler][.]")
{
    using namespace tests_3d;
    //        0 ---------- 4
    //       / \\        // \ .
    //      /   \ \     //   \ .
    //     /     \  \  //     \ .
    //    /       \   \3       \ .
    //  1 --------- 2/ -------- 5   tuple edge 2-3
    //    \       /  /\ \      / .
    //     \     / /   \\     / .
    //      \   //      \\   / .
    //       \ //        \  / .
    //        6 -----------7
    //
    DEBUG_TetMesh m = six_cycle_tets();

    SimplicialEmbeddingOptions options;
    options.value = 1;
    options.tag_attributes[PrimitiveType::Vertex] =
        m.register_attribute<int64_t>("vertex_tag", wmtk::PrimitiveType::Vertex, 1);
    options.tag_attributes[PrimitiveType::Edge] =
        m.register_attribute<int64_t>("edge_tag", wmtk::PrimitiveType::Edge, 1);
    options.tag_attributes[PrimitiveType::Triangle] =
        m.register_attribute<int64_t>("face_tag", wmtk::PrimitiveType::Triangle, 1);
    options.pass_through_attributes.emplace_back(
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));

    SECTION("points_in_3d_case")
    {
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            attribute::Accessor<int64_t> acc_vertex_tag =
                m.create_accessor<int64_t>(options.tag_attributes[PrimitiveType::Vertex]);
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = options.value;
        }
        simplicial_embedding::simplicial_embedding(m, options);

        CHECK(false); // TODO add real checks

        if (false) {
            ParaviewWriter writer(
                data_dir / "simplicial_embedding_result_points_3d_case",
                "vertices",
                m,
                true,
                true,
                true,
                true);
            m.serialize(writer);
        }
    }
    SECTION("edges_in_3d_case")
    {
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            attribute::Accessor<int64_t> acc_vertex_tag =
                m.create_accessor<int64_t>(options.tag_attributes[PrimitiveType::Vertex]);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = options.value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[7]) = options.value;
            attribute::Accessor<int64_t> acc_edge_tag =
                m.create_accessor<int64_t>(options.tag_attributes[PrimitiveType::Edge]);
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(0, 1, 0)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(0, 2, 0)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(0, 3, 0)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(1, 2, 0)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(2, 3, 0)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(3, 1, 0)) = options.value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(2, 5, 2)) = options.value;
        }
        simplicial_embedding::simplicial_embedding(m, options);

        CHECK(false); // TODO add real checks

        if (false) {
            ParaviewWriter writer(
                data_dir / "simplicial_embedding_result_edges_3d_case",
                "vertices",
                m,
                true,
                true,
                true,
                true);
            m.serialize(writer);
        }
    }
}
