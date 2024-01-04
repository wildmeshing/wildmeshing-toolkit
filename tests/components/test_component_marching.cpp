#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/marching/internal/Marching.hpp>
#include <wmtk_components/marching/internal/MarchingOptions.hpp>
#include <wmtk_components/marching/marching.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;


TEST_CASE("component_marching_options", "[components][marching]")
{
    using namespace components::internal;

    json o = {
        {"type", "marching"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"input_tags", {"vertex_tag", 0, 1}},
        {"output_tags", {"vertex_tag", 2}},
        {"edge_filter_tags", json::array({})}};

    CHECK_NOTHROW(o.get<MarchingOptions>());

    o["type"] = "something else";
    CHECK_THROWS(o.get<MarchingOptions>());
}

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

    attribute::TypedAttributeHandle<int64_t> vertex_tag_handle = m.register_attribute<int64_t>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        input_tag_value_0).as<int64_t>();

    std::tuple<attribute::TypedAttributeHandle<int64_t>, int64_t, int64_t> vertex_tags =
        std::make_tuple(vertex_tag_handle, input_tag_value_0, input_tag_value_1);

    std::tuple<std::string, int64_t> output_tags =
        std::make_tuple("vertex_tag", isosurface_tag_value);

    std::vector<std::tuple<attribute::TypedAttributeHandle<int64_t>, int64_t>> filter_tag;


    int64_t expected_isosurface_vertex_num = 0;

    SECTION("4")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;

        expected_isosurface_vertex_num = 6;
    }
    SECTION("4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 9;
    }
    SECTION("0-4-5")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        expected_isosurface_vertex_num = 10;
    }
    SECTION("0-4-5-with-filter")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;

        attribute::TypedAttributeHandle<int64_t> filter =
            m.register_attribute<int64_t>("edge_filter", PrimitiveType::Edge, 1).as<int64_t>();
        const int64_t filter_val = 1;
        filter_tag.emplace_back(std::make_tuple(filter, filter_val));

        Accessor<int64_t> acc_filter = m.create_accessor(filter);
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(1, 4)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(1, 5)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(2, 5)) = 1;
        acc_filter.scalar_attribute(m.edge_tuple_from_vids(5, 6)) = 1;

        expected_isosurface_vertex_num = 5;
    }

    int64_t expected_vertex_num =
        m.get_all(PrimitiveType::Vertex).size() + expected_isosurface_vertex_num;


    components::internal::Marching mc(m, vertex_tags, output_tags, filter_tag);
    mc.process();

    const auto& vertices = m.get_all(PrimitiveType::Vertex);
    Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
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

    // should be manifold
    {
        for (const Tuple& v : vertices) {
            if (acc_vertex_tag.scalar_attribute(v) == isosurface_tag_value) {
                std::vector<Tuple> one_ring = simplex::link(m, simplex::Simplex::vertex(v))
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
            writer("marching_2d_result", "vertices", m, true, false, true, false);
        m.serialize(writer);
    }
}

TEST_CASE("marching_component_tet", "[components][marching][.]")
{
    REQUIRE(false); // TODO test when wmtk code is ready on TetMesh
    const int64_t input_tag_value_0 = 0;
    const int64_t input_tag_value_1 = 1;
    const int64_t isosurface_tag_value = 2;

    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
    //    7---8

    tests_3d::DEBUG_TetMesh m = tests_3d::three_incident_tets_with_positions();

    attribute::TypedAttributeHandle<int64_t> vertex_tag_handle = m.register_attribute<int64_t>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        input_tag_value_0).as<int64_t>();

    std::tuple<attribute::TypedAttributeHandle<int64_t>, int64_t, int64_t> vertex_tags =
        std::make_tuple(vertex_tag_handle, input_tag_value_0, input_tag_value_1);

    std::tuple<std::string, int64_t> output_tags =
        std::make_tuple("vertex_tag", isosurface_tag_value);

    std::vector<std::tuple<attribute::TypedAttributeHandle<int64_t>, int64_t>> filter_tag;


    int64_t expected_isosurface_vertex_num = 0;

    SECTION("2-3")
    {
        const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
        Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
        acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = input_tag_value_1;
        acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = input_tag_value_1;

        expected_isosurface_vertex_num = 6;
    }
    // SECTION("4-5")
    //{
    //     const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
    //     Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
    //     acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value_1;
    //     acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value_1;
    //
    //     expected_isosurface_vertex_num = 9;
    // }

    int64_t expected_vertex_num =
        m.get_all(PrimitiveType::Vertex).size() + expected_isosurface_vertex_num;


    components::internal::Marching mc(m, vertex_tags, output_tags, filter_tag);
    mc.process();

    const auto& vertices = m.get_all(PrimitiveType::Vertex);
    Accessor<int64_t> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
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

    if (true) {
        wmtk::io::ParaviewWriter
            writer("marching_3d_result", "vertices", m, true, true, true, true);
        m.serialize(writer);
    }
}
