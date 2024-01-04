#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/regular_space/internal/RegularSpace.hpp>
#include <wmtk_components/regular_space/internal/RegularSpaceOptions.hpp>
#include <wmtk_components/regular_space/regular_space.hpp>
#include "wmtk/../../tests/tools/DEBUG_TetMesh.hpp"
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TetMesh_examples.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("regular_space_file_reading", "[components][regular_space]")
{
    using namespace components::internal;

    std::map<std::string, std::filesystem::path> files;
    std::map<std::string, int64_t> tags_value;

    json o = {
        {"type", "regular_space"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"tags", json::array({})}};

    CHECK_NOTHROW(o.get<RegularSpaceOptions>());

    const int64_t tag_value = 1;
    std::vector<std::tuple<std::string, int64_t, int64_t>> tags;
    tags.emplace_back(
        std::make_tuple("face_tag", get_primitive_type_id(PrimitiveType::Face), tag_value));
    tags.emplace_back(
        std::make_tuple("edge_tag", get_primitive_type_id(PrimitiveType::Edge), tag_value));
    tags.emplace_back(
        std::make_tuple("vertex_tag", get_primitive_type_id(PrimitiveType::Vertex), tag_value));

    o["tags"] = tags;
    CHECK_NOTHROW(o.get<RegularSpaceOptions>());

    o["type"] = "something else";
    CHECK_THROWS(o.get<RegularSpaceOptions>());
}

TEST_CASE("regular_space_component_tri", "[components][regular_space][trimesh][2D][scheduler]")
{
    const int64_t tag_value = 1;
    tests::DEBUG_TriMesh m = wmtk::tests::hex_plus_two_with_position();
    wmtk::attribute::MeshAttributeHandle vertex_tag_handle =
        m.register_attribute<int64_t>("vertex_tag", wmtk::PrimitiveType::Vertex, 1);
    wmtk::attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", wmtk::PrimitiveType::Edge, 1);
    wmtk::attribute::MeshAttributeHandle face_tag_handle =
        m.register_attribute<int64_t>("face_tag", wmtk::PrimitiveType::Face, 1);

    std::vector<std::tuple<std::string, int64_t, int64_t>> tags;
    tags.emplace_back(
        std::make_tuple("face_tag", get_primitive_type_id(PrimitiveType::Face), tag_value));
    tags.emplace_back(
        std::make_tuple("edge_tag", get_primitive_type_id(PrimitiveType::Edge), tag_value));
    tags.emplace_back(
        std::make_tuple("vertex_tag", get_primitive_type_id(PrimitiveType::Vertex), tag_value));

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
            Accessor<int64_t> acc_vertex_tag = m.create_accessor<int64_t>(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[6]) = tag_value;
        }

        components::internal::RegularSpace rs(m);
        rs.regularize_tags(tags);

        CHECK(m.get_all(PrimitiveType::Vertex).size() == 15);

        if (false) {
            wmtk::io::ParaviewWriter writer(
                data_dir / "regular_space_result_0d_case",
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
            Accessor<int64_t> acc_vertex_tag = m.create_accessor<int64_t>(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[6]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[7]) = tag_value;
            Accessor<int64_t> acc_edge_tag = m.create_accessor<int64_t>(edge_tag_handle);
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(4, 5, 2)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(5, 1, 2)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 4, 2)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(7, 4, 5)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(7, 3, 5)) = tag_value;
        }

        components::internal::RegularSpace rs(m);
        rs.regularize_tags(tags);

        CHECK(m.get_all(PrimitiveType::Face).size() == 17);
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 15);

        if (false) {
            ParaviewWriter writer(
                data_dir / "regular_space_result_1d_case",
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

TEST_CASE("regular_space_component_tet", "[components][regular_space][tetmesh][3D][scheduler][.]")
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
    const int64_t tag_value = 1;
    Eigen::MatrixXd V(8, 3);
    V.row(0) << 0.5, 0.86, 0;
    V.row(1) << 0, 0, 0;
    V.row(2) << 1.0, 0, 1.0;
    V.row(3) << 1.0, 0, -1.0;
    V.row(4) << 1.5, 0.86, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 0.5, -0.86, 0;
    V.row(7) << 1.5, -0.86, 0;
    wmtk::attribute::MeshAttributeHandle vertex_tag_handle =
        m.register_attribute<int64_t>("vertex_tag", wmtk::PrimitiveType::Vertex, 1);
    wmtk::attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", wmtk::PrimitiveType::Edge, 1);

    std::vector<std::tuple<std::string, int64_t, int64_t>> tags;
    tags.emplace_back(
        std::make_tuple("vertex_tag", get_primitive_type_id(PrimitiveType::Vertex), tag_value));
    tags.emplace_back(
        std::make_tuple("edge_tag", get_primitive_type_id(PrimitiveType::Edge), tag_value));

    SECTION("points_in_3d_case")
    {
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            Accessor<int64_t> acc_vertex_tag = m.create_accessor<int64_t>(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = tag_value;
        }
        components::internal::RegularSpace rs(m);
        rs.regularize_tags(tags);

        CHECK(false); // TODO add real checks

        if (false) {
            ParaviewWriter writer(
                data_dir / "regular_space_result_points_3d_case",
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
            Accessor<int64_t> acc_vertex_tag = m.create_accessor<int64_t>(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[0]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[1]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[3]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[5]) = tag_value;
            acc_vertex_tag.scalar_attribute(vertex_tuples[7]) = tag_value;
            Accessor<int64_t> acc_edge_tag = m.create_accessor<int64_t>(edge_tag_handle);
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 2, 0)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 3, 0)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 3, 0)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(3, 1, 0)) = tag_value;
            acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 5, 2)) = tag_value;
        }
        components::internal::RegularSpace rs(m);
        rs.regularize_tags(tags);

        CHECK(false); // TODO add real checks

        if (false) {
            ParaviewWriter writer(
                data_dir / "regular_space_result_edges_3d_case",
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
