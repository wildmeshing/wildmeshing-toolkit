#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/regular_space/internal/RegularSpace.hpp>
#include <wmtk_components/regular_space/regular_space.hpp>
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("regular_space_file_reading", "[components][regular_space][.]")
{
    std::map<std::string, std::filesystem::path> files;
    std::map<std::string, long> tags_value;

    json regular_space_jason = {
        {"type", "regular space"},
        {"input", "inputdir"}, /*input dir*/
        {"output", "outputdir"}, /*output dir*/
        {"demension", 1}, /*0 for vertex, 1 for edge, 2 for face, 3 for tet*/
        {"tags_value", tags_value},
        {"split_tag_value"}};

    // TODO
    // upload embedding result .hdf5 file and use regular_space API
    REQUIRE(false);
}

TEST_CASE("regular_space_component", "[components][regular_space][scheduler]")
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2;
    tests::DEBUG_TriMesh m = wmtk::tests::hex_plus_two_with_position();
    MeshAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
        "vertex_tag",
        wmtk::PrimitiveType::Vertex,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>(
        "edge_tag",
        wmtk::PrimitiveType::Edge,
        1,
        false,
        embedding_tag_value);
    SECTION("0d_case")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /  .
        //    7---8
        // set 0 1 4 5 6
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[6]) = input_tag_value;
        }

        components::internal::RegularSpace rs(
            m,
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            input_tag_value,
            embedding_tag_value,
            split_tag_value,
            0);
        rs.process();

        CHECK(m.get_all(PrimitiveType::Vertex).size() == 15);

        Accessor<long> acc_todo = m.create_accessor(m.get_attribute_handle<long>(
            std::string("todo_edgesplit_same_tag"),
            PrimitiveType::Edge));
        int todo_num = 0;
        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            // spdlog::info("{}", acc_todo.scalar_attribute(t));
            if (acc_todo.scalar_attribute(t) == 1) {
                todo_num++;
            }
        }
        CHECK(todo_num == 0);

        if (false) {
            ParaviewWriter writer(
                data_dir / "regular_space_result_0d_case",
                "position",
                m,
                true,
                true,
                true,
                false);
            m.serialize(writer);
        }
    }
    SECTION("1d_case")
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
            Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[6]) = input_tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[7]) = input_tag_value;
            Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(4, 5, 2)) = input_tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(5, 1, 2)) = input_tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 4, 2)) = input_tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(7, 4, 5)) = input_tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(7, 3, 5)) = input_tag_value;
        }

        components::internal::RegularSpace rs(
            m,
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            input_tag_value,
            embedding_tag_value,
            split_tag_value,
            1);
        rs.process();

        Accessor<long> acc_todo_edge = m.create_accessor(m.get_attribute_handle<long>(
            std::string("todo_edgesplit_same_tag"),
            PrimitiveType::Edge));
        Accessor<long> acc_todo_face = m.create_accessor(
            m.get_attribute_handle<long>(std::string("todo_facesplit_tag"), PrimitiveType::Face));
        int todo_num = 0;
        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            // spdlog::info("{}", acc_todo.scalar_attribute(t));
            if (acc_todo_edge.scalar_attribute(t) == 1) {
                todo_num++;
            }
        }
        for (const Tuple& t : m.get_all(PrimitiveType::Face)) {
            // spdlog::info("{}", acc_todo.scalar_attribute(t));
            if (acc_todo_face.scalar_attribute(t) == 1) {
                todo_num++;
            }
        }
        CHECK(todo_num == 0);
        CHECK(m.get_all(PrimitiveType::Face).size() == 17);
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 15);
        if (false) {
            ParaviewWriter writer(
                data_dir / "regular_space_result_1d_case",
                "position",
                m,
                true,
                true,
                true,
                false);
            m.serialize(writer);
        }
    }
    SECTION("2d_case")
    {
        components::internal::RegularSpace rs(
            m,
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            input_tag_value,
            embedding_tag_value,
            split_tag_value,
            2);
        CHECK_THROWS(rs.process());
    }
    SECTION("unknown_dimension_case")
    {
        components::internal::RegularSpace rs(
            m,
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            input_tag_value,
            embedding_tag_value,
            split_tag_value,
            -7);
        CHECK_THROWS(rs.process());
    }
}