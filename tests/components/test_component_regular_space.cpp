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

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TriMesh hex_plus_two()
{
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
    //    7---8
    TriMesh m;
    RowVectors3l tris;
    tris.resize(8, 3);
    tris.row(0) << 3, 4, 0;
    tris.row(1) << 4, 1, 0;
    tris.row(2) << 4, 5, 1;
    tris.row(3) << 5, 2, 1;
    tris.row(4) << 5, 6, 2;
    tris.row(5) << 3, 7, 4;
    tris.row(6) << 7, 8, 4;
    tris.row(7) << 4, 8, 5;
    m.initialize(tris);
    return m;
}

TriMesh hex_plus_two_with_position()
{
    TriMesh m = hex_plus_two();

    Eigen::MatrixXd V;
    V.resize(9, 3);
    V.row(0) << 0.5, 1, 0;
    V.row(1) << 1.5, 1, 0;
    V.row(2) << 2.5, 1, 0;
    V.row(3) << 0, 0, 0;
    V.row(4) << 1, 0, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 3, 0, 0;
    V.row(7) << 0.5, -1, 0;
    V.row(8) << 1.5, -1, 0;

    mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, m);
    return m;
}

TEST_CASE("file reading", "[components][regular_space][.]")
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
}

TEST_CASE("1d case", "[components][regular_space][scheduler]")
{
    long input_value = 1;
    long embedding_value = 0;
    long split_value = 2;

    TriMesh m = hex_plus_two_with_position();
    MeshAttributeHandle<double> pos_handle =
        m.get_attribute_handle<double>(std::string("position"), wmtk::PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
        std::string("vertex_tag"),
        wmtk::PrimitiveType::Vertex,
        1,
        false,
        embedding_value);
    MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>(
        std::string("edge_tag"),
        wmtk::PrimitiveType::Edge,
        1,
        false,
        embedding_value);

    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
    //    7---8
    // set 0 1 4 5 6
    const std::vector<Tuple>& vs = m.get_all(wmtk::PrimitiveType::Vertex);
    Accessor<long> acc_vertex_tag = m.create_accessor(vertex_tag_handle);
    acc_vertex_tag.scalar_attribute(vs[0]) = input_value;
    acc_vertex_tag.scalar_attribute(vs[1]) = input_value;
    acc_vertex_tag.scalar_attribute(vs[4]) = input_value;
    acc_vertex_tag.scalar_attribute(vs[5]) = input_value;
    acc_vertex_tag.scalar_attribute(vs[6]) = input_value;

    components::internal::RegularSpace rs(
        m,
        pos_handle,
        vertex_tag_handle,
        edge_tag_handle,
        input_value,
        embedding_value,
        split_value,
        0);
    rs.process();
    ParaviewWriter writer1(data_dir / "my_result", "position", m, true, true, true, false);
    m.serialize(writer1);

    // Accessor<long> acc_todo = m.create_accessor(
    //     m.get_attribute_handle<long>(std::string("todo_edgesplit_same_tag"),
    //     PrimitiveType::Edge));
    // int todo_num = 0;
    // for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
    //     spdlog::info("{}", acc_todo.scalar_attribute(t));
    //     if (acc_todo.scalar_attribute(t) == 1) {
    //         todo_num++;
    //     }
    // }
    // spdlog::info("{}", todo_num);
}