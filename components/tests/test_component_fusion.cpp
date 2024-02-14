#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/fusion/fusion.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>

using namespace wmtk::components::base;
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;


TEST_CASE("fusion_2d", "[components][fusion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    RowVectors3d V;
    V.resize(15, 3);
    V.row(0) << 1.0, 0.0, 0.0;
    V.row(1) << 1.25, 0.0, 0.0;
    V.row(2) << 1.5, 0.0, 0.0;
    V.row(3) << 1.75, 0.0, 0.0;
    V.row(4) << 2.0, 0.0, 0.0;
    V.row(5) << 1.0, 1.0, 0.0;
    V.row(6) << 1.25, 1.0, 0.0;
    V.row(7) << 1.5, 1.0, 0.0;
    V.row(8) << 1.75, 1.0, 0.0;
    V.row(9) << 2.0, 1.0, 0.0;
    V.row(10) << 1.0, 2.0, 0.0;
    V.row(11) << 1.25, 2.0, 0.0;
    V.row(12) << 1.5, 2.0, 0.0;
    V.row(13) << 1.75, 2.0, 0.0;
    V.row(14) << 2.0, 2.0, 0.0;

    RowVectors3l F;
    F.resize(16, 3);
    F.row(0) << 0, 1, 6;
    F.row(1) << 0, 6, 5;
    F.row(2) << 1, 2, 7;
    F.row(3) << 1, 7, 6;
    F.row(4) << 2, 3, 8;
    F.row(5) << 2, 8, 7;
    F.row(6) << 3, 4, 9;
    F.row(7) << 3, 9, 8;
    F.row(8) << 5, 6, 11;
    F.row(9) << 5, 11, 10;
    F.row(10) << 6, 7, 12;
    F.row(11) << 6, 12, 11;
    F.row(12) << 7, 8, 13;
    F.row(13) << 7, 13, 12;
    F.row(14) << 8, 9, 14;
    F.row(15) << 8, 14, 13;

    DEBUG_TriMesh m;
    m.initialize(F);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);

    cache.write_mesh(m, "test_mesh");

    SECTION("x")
    {
        json input =
            R"({
        "input": "test_mesh",
        "name": "on_x",
        "fusion_X": true,
        "fusion_Y": false,
        "fusion_Z": false
        })"_json;

        wmtk::components::fusion(Paths(), input, cache);

        auto p_mesh = cache.read_mesh("on_x");
        DEBUG_TriMesh& periodic_mesh = static_cast<DEBUG_TriMesh&>(*p_mesh);

        wmtk::utils::EigenMatrixWriter writer;
        periodic_mesh.serialize(writer);

        MatrixX<int64_t> F_p;
        writer.get_FV_matrix(F_p);

        std::cout << F_p << std::endl << std::endl;

        auto childs = periodic_mesh.get_child_meshes();
        CHECK(childs.size() == 1);
        CHECK(
            childs[0]->get_all(PrimitiveType::Triangle).size() ==
            periodic_mesh.get_all(PrimitiveType::Triangle).size());
        CHECK(periodic_mesh.get_all(PrimitiveType::Vertex).size() == 12);
        CHECK(childs[0]->get_all(PrimitiveType::Vertex).size() == 15);

        wmtk::utils::EigenMatrixWriter writer_child;
        childs[0]->serialize(writer_child);
        MatrixX<int64_t> FV_child;
        MatrixX<double> V_child;

        writer_child.get_position_matrix(V_child);
        writer_child.get_FV_matrix(FV_child);

        std::cout << V_child << std::endl << std::endl;
        std::cout << FV_child << std::endl << std::endl;
    }
    SECTION("y")
    {
        json input =
            R"({
        "input": "test_mesh",
        "name": "on_y",
        "fusion_X": false,
        "fusion_Y": true,
        "fusion_Z": false
        })"_json;

        wmtk::components::fusion(Paths(), input, cache);

        auto p_mesh = cache.read_mesh("on_y");
        DEBUG_TriMesh& periodic_mesh = static_cast<DEBUG_TriMesh&>(*p_mesh);

        wmtk::utils::EigenMatrixWriter writer;
        periodic_mesh.serialize(writer);

        // MatrixX<double> V_p;
        MatrixX<int64_t> F_p;
        // writer.get_position_matrix(V_p);
        writer.get_FV_matrix(F_p);

        // std::cout << V_p << std::endl << std::endl;
        // std::cout << F_p << std::endl;

        auto childs = periodic_mesh.get_child_meshes();
        CHECK(childs.size() == 1);
        CHECK(
            childs[0]->get_all(PrimitiveType::Triangle).size() ==
            periodic_mesh.get_all(PrimitiveType::Triangle).size());
    }
    SECTION("all")
    {
        json input =
            R"({
        "input": "test_mesh",
        "name": "on_all",
        "fusion_X": true,
        "fusion_Y": true,
        "fusion_Z": false
        })"_json;

        wmtk::components::fusion(Paths(), input, cache);

        auto p_mesh = cache.read_mesh("on_all");
        DEBUG_TriMesh& periodic_mesh = static_cast<DEBUG_TriMesh&>(*p_mesh);

        wmtk::utils::EigenMatrixWriter writer;
        periodic_mesh.serialize(writer);

        // MatrixX<double> V_p;
        MatrixX<int64_t> F_p;
        // writer.get_position_matrix(V_p);
        writer.get_FV_matrix(F_p);

        // std::cout << V_p << std::endl << std::endl;
        // std::cout << F_p << std::endl;
        auto childs = periodic_mesh.get_child_meshes();
        CHECK(childs.size() == 1);
        CHECK(
            childs[0]->get_all(PrimitiveType::Triangle).size() ==
            periodic_mesh.get_all(PrimitiveType::Triangle).size());
    }
}

TEST_CASE("fusion_3d", "[components][fusion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    RowVectors3d V;
    V.resize(8, 3);
    V.row(0) << 1.0, 1.0, 1.0;
    V.row(1) << 3.0, 1.0, 1.0;
    V.row(2) << 3.0, 1.5, 1.0;
    V.row(3) << 1.0, 1.5, 1.0;
    V.row(4) << 1.0, 1.0, 4.0;
    V.row(5) << 3.0, 1.0, 4.0;
    V.row(6) << 3.0, 1.5, 4.0;
    V.row(7) << 1.0, 1.5, 4.0;

    RowVectors4l T;
    T.resize(6, 4);
    T.row(0) << 0, 1, 2, 3;
    T.row(1) << 5, 2, 6, 7;
    T.row(2) << 4, 1, 5, 3;
    T.row(3) << 4, 3, 7, 5;
    T.row(4) << 3, 1, 5, 2;
    T.row(5) << 2, 3, 7, 5;

    DEBUG_TetMesh m;
    m.initialize(T);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);

    cache.write_mesh(m, "test_mesh");

    SECTION("x")
    {
        json input =
            R"({
        "input": "test_mesh",
        "name": "on_x",
        "fusion_X": true,
        "fusion_Y": false,
        "fusion_Z": false
        })"_json;

        wmtk::components::fusion(Paths(), input, cache);

        auto p_mesh = cache.read_mesh("on_x");
        DEBUG_TetMesh& periodic_mesh = static_cast<DEBUG_TetMesh&>(*p_mesh);

        wmtk::utils::EigenMatrixWriter writer;
        periodic_mesh.serialize(writer);

        // MatrixX<double> V_p;
        MatrixX<int64_t> T_p;
        // writer.get_position_matrix(V_p);
        writer.get_TV_matrix(T_p);

        // std::cout << V_p << std::endl << std::endl;
        // std::cout << T_p << std::endl;
        auto childs = periodic_mesh.get_child_meshes();
        CHECK(childs.size() == 1);
        CHECK(
            childs[0]->get_all(PrimitiveType::Tetrahedron).size() ==
            periodic_mesh.get_all(PrimitiveType::Tetrahedron).size());
        CHECK(periodic_mesh.get_all(PrimitiveType::Vertex).size() == 4);

        wmtk::utils::EigenMatrixWriter writer_child;
        childs[0]->serialize(writer_child);
        MatrixX<int64_t> TV_child;
        MatrixX<double> V_child;

        writer_child.get_position_matrix(V_child);
        writer_child.get_TV_matrix(TV_child);

        std::cout << V_child << std::endl << std::endl;
        std::cout << TV_child << std::endl << std::endl;
    }

    SECTION("xy")
    {
        json input =
            R"({
        "input": "test_mesh",
        "name": "on_xy",
        "fusion_X": true,
        "fusion_Y": true,
        "fusion_Z": false
        })"_json;

        wmtk::components::fusion(Paths(), input, cache);

        auto p_mesh = cache.read_mesh("on_xy");
        DEBUG_TetMesh& periodic_mesh = static_cast<DEBUG_TetMesh&>(*p_mesh);

        wmtk::utils::EigenMatrixWriter writer;
        periodic_mesh.serialize(writer);

        // MatrixX<double> V_p;
        MatrixX<int64_t> T_p;
        // writer.get_position_matrix(V_p);
        writer.get_TV_matrix(T_p);

        // std::cout << V_p << std::endl << std::endl;
        // std::cout << T_p << std::endl;
        auto childs = periodic_mesh.get_child_meshes();
        CHECK(childs.size() == 1);
        CHECK(
            childs[0]->get_all(PrimitiveType::Tetrahedron).size() ==
            periodic_mesh.get_all(PrimitiveType::Tetrahedron).size());
        CHECK(periodic_mesh.get_all(PrimitiveType::Vertex).size() == 2);
    }

    SECTION("xyz")
    {
        json input =
            R"({
        "input": "test_mesh",
        "name": "on_xyz",
        "fusion_X": true,
        "fusion_Y": true,
        "fusion_Z": true
        })"_json;

        wmtk::components::fusion(Paths(), input, cache);

        auto p_mesh = cache.read_mesh("on_xyz");
        DEBUG_TetMesh& periodic_mesh = static_cast<DEBUG_TetMesh&>(*p_mesh);

        wmtk::utils::EigenMatrixWriter writer;
        periodic_mesh.serialize(writer);

        // MatrixX<double> V_p;
        MatrixX<int64_t> T_p;
        // writer.get_position_matrix(V_p);
        writer.get_TV_matrix(T_p);

        // std::cout << V_p << std::endl << std::endl;
        // std::cout << T_p << std::endl;
        auto childs = periodic_mesh.get_child_meshes();
        CHECK(childs.size() == 1);
        CHECK(
            childs[0]->get_all(PrimitiveType::Tetrahedron).size() ==
            periodic_mesh.get_all(PrimitiveType::Tetrahedron).size());
        CHECK(periodic_mesh.get_all(PrimitiveType::Vertex).size() == 1);
    }
}
