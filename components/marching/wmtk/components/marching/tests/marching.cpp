#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/components/marching/marching.hpp>

using namespace wmtk;


TEST_CASE("marching_component_tri", "[components][marching]")
{
    const int64_t input_tag_value_0 = 0;
    const int64_t input_tag_value_1 = 1;
    const int64_t isosurface_tag_value = 2;

    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
    //    7---8
    tests::DEBUG_TriMesh m = tests::hex_plus_two_with_position();

    components::MarchingOptions options;
    options.position_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    options.label_handles[PrimitiveType::Vertex] = m.register_attribute<int64_t>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        input_tag_value_0);
    options.label_handles[PrimitiveType::Edge] =
        m.register_attribute<int64_t>("marching_edge_tag", PrimitiveType::Edge, 1);

    options.input_values = {input_tag_value_0, input_tag_value_1};
    options.output_value = isosurface_tag_value;

    int64_t expected_isosurface_vertex_num = 0;
    int64_t expected_isosurface_edge_num = 0;

    SECTION("4")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;

        expected_isosurface_vertex_num = 6;
        expected_isosurface_edge_num = 6;
    }
    SECTION("4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 9;
        expected_isosurface_edge_num = 8;
    }
    SECTION("0-4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 10;
        expected_isosurface_edge_num = 8;
    }
    SECTION("0-4-5-with-filter")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        attribute::MeshAttributeHandle filter =
            m.register_attribute<int64_t>("edge_filter", PrimitiveType::Edge, 1);
        options.edge_filter_handles.emplace_back(filter, 1);

        attribute::Accessor<int64_t> acc_filter = m.create_accessor<int64_t>(filter);
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(1, 4)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(1, 5)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(2, 5)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(5, 6)) = 1;

        expected_isosurface_vertex_num = 5;
        expected_isosurface_edge_num = 4;
    }

    int64_t expected_vertex_num =
        m.get_all(PrimitiveType::Vertex).size() + expected_isosurface_vertex_num;

    components::marching(m, options);

    const auto& vertices = m.get_all(PrimitiveType::Vertex);
    attribute::Accessor<int64_t> acc_vertex_tag =
        m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
    // vertex number should be correct
    {
        CHECK(vertices.size() == expected_vertex_num);

        int64_t isosurface_vertex_num = 0;
        for (const Tuple& v : vertices) {
            if (acc_vertex_tag.scalar_attribute(v) == isosurface_tag_value) {
                isosurface_vertex_num++;
            }
        }
        CHECK(isosurface_vertex_num == expected_isosurface_vertex_num);
    }

    const auto& edges = m.get_all(PrimitiveType::Edge);
    wmtk::attribute::Accessor<int64_t> acc_edge_tag =
        m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Edge]);
    // edge number should be correct
    {
        int64_t isosurface_edge_num = 0;
        for (const Tuple& e : edges) {
            if (acc_edge_tag.scalar_attribute(e) == isosurface_tag_value) {
                isosurface_edge_num++;
            }
        }
        CHECK(isosurface_edge_num == expected_isosurface_edge_num);
    }

    // should be manifold
    {
        for (const Tuple& v : vertices) {
            if (acc_vertex_tag.scalar_attribute(v) == isosurface_tag_value) {
                std::vector<Tuple> one_ring = simplex::link(m, simplex::Simplex::vertex(m, v))
                                                  .simplex_vector_tuples(PrimitiveType::Vertex);

                int64_t tagged_neighbors = 0;
                for (const Tuple& neigh : one_ring) {
                    if (acc_vertex_tag.scalar_attribute(neigh) == isosurface_tag_value) {
                        ++tagged_neighbors;
                    }
                }

                if (m.is_boundary_vertex(v)) {
                    CHECK(tagged_neighbors == 1);
                } else {
                    CHECK(tagged_neighbors == 2);
                }
            }
        }
    }

    if (false) {
        wmtk::io::ParaviewWriter
            writer("marching_2d_result", "vertices", m, true, true, true, false);
        m.serialize(writer);
    }
}

TEST_CASE("marching_component_tet", "[components][marching]")
{
    const int64_t input_tag_value_0 = 0;
    const int64_t input_tag_value_1 = 1;
    const int64_t isosurface_tag_value = 2;

    //        0 ---------- 4
    //       / \\        // \ .
    //      /   \ \     //   \ .
    //     /     \  \  //     \  .
    //    /       \   \3       \ .
    //  1 --------- 2/ -------- 5   tuple edge 2-3
    //

    tests_3d::DEBUG_TetMesh m = tests_3d::three_incident_tets_with_positions();

    components::MarchingOptions options;
    options.position_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    options.label_handles[PrimitiveType::Vertex] = m.register_attribute<int64_t>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        input_tag_value_0);
    options.label_handles[PrimitiveType::Edge] =
        m.register_attribute<int64_t>("marching_edge_tag", PrimitiveType::Edge, 1);
    options.label_handles[PrimitiveType::Triangle] =
        m.register_attribute<int64_t>("marching_face_tag", PrimitiveType::Triangle, 1);

    options.input_values = {input_tag_value_0, input_tag_value_1};
    options.output_value = isosurface_tag_value;


    int64_t expected_isosurface_vertex_num = 0;
    int64_t expected_isosurface_face_num = 0;

    SECTION("2-3")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = input_tag_value_1;

        expected_isosurface_vertex_num = 8;
        expected_isosurface_face_num = 6;
    }
    SECTION("4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        wmtk::attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 5;
        expected_isosurface_face_num = 3;
    }
    SECTION("1-0-4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        wmtk::attribute::Accessor<int64_t> acc_vertex_tag =
            m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
        acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 8;
        expected_isosurface_face_num = 6;
    }

    int64_t expected_vertex_num =
        m.get_all(PrimitiveType::Vertex).size() + expected_isosurface_vertex_num;


    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    pass_through_attributes.emplace_back(
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));

    components::marching(m, options);

    const auto& vertices = m.get_all(PrimitiveType::Vertex);
    attribute::Accessor<int64_t> acc_vertex_tag =
        m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Vertex]);
    // vertex number should be correct
    {
        CHECK(vertices.size() == expected_vertex_num);

        int64_t isosurface_vertex_num = 0;
        for (const Tuple& v : vertices) {
            if (acc_vertex_tag.scalar_attribute(v) == isosurface_tag_value) {
                isosurface_vertex_num++;
            }
        }
        CHECK(isosurface_vertex_num == expected_isosurface_vertex_num);
    }

    // face number should be correct
    const auto& faces = m.get_all(PrimitiveType::Triangle);
    wmtk::attribute::Accessor<int64_t> acc_face_tag =
        m.create_accessor<int64_t>(options.label_handles[PrimitiveType::Triangle]);
    {
        int64_t isosurface_face_num = 0;
        for (const Tuple& f : faces) {
            if (acc_face_tag.scalar_attribute(f) == isosurface_tag_value) {
                isosurface_face_num++;
            }
        }
        CHECK(isosurface_face_num == expected_isosurface_face_num);
    }

    if (false) {
        wmtk::io::ParaviewWriter
            writer("marching_3d_result", "vertices", m, true, true, true, true);
        m.serialize(writer);
    }
}
