#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/../../tests/tools/DEBUG_TetMesh.hpp>
#include <wmtk/../../tests/tools/DEBUG_TriMesh.hpp>
#include <wmtk/../../tests/tools/TetMesh_examples.hpp>
#include <wmtk/../../tests/tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/marching/internal/Marching.hpp>
#include <wmtk_components/marching/internal/MarchingOptions.hpp>
#include <wmtk_components/marching/marching.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("marching_file_reading", "[components][marching][.]")
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
    // upload regular_space result .hdf5 file and use marching_component API
    REQUIRE(false);
}

TEST_CASE("marching_component", "[2D][components][marching][scheduler]")
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
    MeshAttributeHandle<long> face_tag_handle = m.register_attribute<long>(
        "face_tag",
        wmtk::PrimitiveType::Face,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> face_filter_handle =
        m.register_attribute<long>("face_filter_tag", wmtk::PrimitiveType::Face, 1, false, 1);
    SECTION("2d_case -- should be manifold")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /  .
        //    7---8
        // set edge 4 as input
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[4]) = input_tag_value;
        }

        components::internal::Marching mc(
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            face_tag_handle,
            face_filter_handle,
            input_tag_value,
            embedding_tag_value,
            split_tag_value,
            false);
        mc.process(m);

        // offset edge number should be correct
        {
            long offset_num = 0;
            Accessor<long> acc_edge_tag = m.create_accessor<long>(edge_tag_handle);
            for (const Tuple& t : m.get_all(wmtk::PrimitiveType::Edge)) {
                if (acc_edge_tag.scalar_attribute(t) == split_tag_value) {
                    offset_num++;
                }
            }
            CHECK(offset_num == 6);
        }

        // should be manifold
        {
            Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
            for (const Tuple& edge : m.get_all(PrimitiveType::Edge)) {
                if (acc_edge_tag.scalar_attribute(edge) == split_tag_value) {
                    Tuple t = m.switch_face(m.switch_edge(edge));
                    int neighbor_num = 0;
                    while (t != edge) {
                        if (acc_edge_tag.scalar_attribute(t) == split_tag_value) {
                            ++neighbor_num;
                        }
                        t = m.switch_face(m.switch_edge(t));
                    }
                    CHECK(neighbor_num == 1);
                }
            }
        }

        if (false) {
            ParaviewWriter
                writer(data_dir / "marching_2d_result", "position", m, true, true, true, false);
            m.serialize(writer);
        }
    }
}

TEST_CASE("marching_component", "[3D][components][marching][scheduler]")
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2;
    tests_3d::DEBUG_TetMesh m = wmtk::tests_3d::six_cycle_tets();
    Eigen::MatrixXd V(8, 3);
    V.row(0) << 0.5, 0.86, 0;
    V.row(1) << 0, 0, 0;
    V.row(2) << 1.0, 0, 1.0;
    V.row(3) << 1.0, 0, -1.0;
    V.row(4) << 1.5, 0.86, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 0.5, -0.86, 0;
    V.row(7) << 1.5, -0.86, 0;
    mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, m);

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
    MeshAttributeHandle<long> face_tag_handle = m.register_attribute<long>(
        "face_tag",
        wmtk::PrimitiveType::Face,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> face_filter_handle =
        m.register_attribute<long>("face_filter_tag", wmtk::PrimitiveType::Face, 1, false, 1);
    SECTION("3d_case--run_and_check_the_file")
    {
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
        {
            const std::vector<Tuple>& vertex_tuples = m.get_all(wmtk::PrimitiveType::Vertex);
            Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
            acc_vertex_tag.scalar_attribute(vertex_tuples[2]) = input_tag_value;
        }

        components::internal::Marching mc(
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            face_tag_handle,
            face_filter_handle,
            input_tag_value,
            embedding_tag_value,
            split_tag_value,
            false);
        mc.process(m);

        // offset edge number should be correct
        {
            long offset_num = 0;
            Accessor<long> acc_edge_tag = m.create_accessor<long>(edge_tag_handle);
            for (const Tuple& t : m.get_all(wmtk::PrimitiveType::Edge)) {
                if (acc_edge_tag.scalar_attribute(t) == split_tag_value) {
                    offset_num++;
                }
            }
            CHECK(offset_num == 12);
        }

        // offset face number should be correct
        {
            long offset_num = 0;
            Accessor<long> acc_face_tag = m.create_accessor<long>(face_tag_handle);
            for (const Tuple& t : m.get_all(wmtk::PrimitiveType::Face)) {
                if (acc_face_tag.scalar_attribute(t) == split_tag_value) {
                    offset_num++;
                }
            }
            CHECK(offset_num == 6);
        }

        if (false) {
            ParaviewWriter
                writer(data_dir / "marching_2d_result", "position", m, true, true, true, false);
            m.serialize(writer);
        }
    }
}