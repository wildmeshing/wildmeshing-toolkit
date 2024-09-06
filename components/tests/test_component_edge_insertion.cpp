#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/edge_insertion/edge_insertion.hpp>
#include <wmtk/components/edge_insertion/internal/edge_insertion.hpp>
#include <wmtk/components/get_all_meshes/get_all_meshes.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>

using namespace wmtk::components::base;
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;


TEST_CASE("edge_insertion", "[components][edge_insertion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    MatrixX<Rational> V;
    V.resize(4, 2);
    V.row(0) = Vector2r(0, 0);
    V.row(1) = Vector2r(1, 0);
    V.row(2) = Vector2r(1, 1);
    V.row(3) = Vector2r(0, 1);

    RowVectors3l F;
    F.resize(2, 3);
    F.row(0) << 0, 1, 2;
    F.row(1) << 0, 2, 3;

    std::shared_ptr<DEBUG_TriMesh> tm = std::make_shared<DEBUG_TriMesh>();
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    MatrixX<Rational> V_e;
    V_e.resize(2, 2);
    V_e.row(0) = Vector2r(0.75, 0.25);
    V_e.row(1) = Vector2r(0.25, 0.75);

    MatrixX<int64_t> E;
    E.resize(1, 2);
    E.row(0) << 0, 1;

    std::shared_ptr<DEBUG_EdgeMesh> em = std::make_shared<DEBUG_EdgeMesh>();
    em->initialize(E);
    mesh_utils::set_matrix_attribute(V_e, "vertices", PrimitiveType::Vertex, *em);

    cache.write_mesh(*tm, "trimesh");
    cache.write_mesh(*em, "edgemesh");

    json input =
        R"({
        "edges": "edgemesh",
        "triangles": "trimesh",
        "output": "embedded_mesh"
        })"_json;

    wmtk::components::edge_insertion(Paths(), input, cache);

    json getall =
        R"({
        "input": "embedded_mesh",
        "name": "meshes"
        })"_json;

    wmtk::components::get_all_meshes(Paths(), getall, cache);

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), output, cache);

    json output2 = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_edges_out",
                "input": "embedded_mesh.input_mesh"
            })"_json;

    wmtk::components::output(Paths(), output2, cache);
}

TEST_CASE("edge_insertion_2", "[components][edge_insertion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    MatrixX<Rational> V;
    V.resize(4, 2);
    V.row(0) = Vector2r(0, 0);
    V.row(1) = Vector2r(1, 0);
    V.row(2) = Vector2r(1, 1);
    V.row(3) = Vector2r(0, 1);

    RowVectors3l F;
    F.resize(2, 3);
    F.row(0) << 0, 1, 2;
    F.row(1) << 0, 2, 3;

    std::shared_ptr<DEBUG_TriMesh> tm = std::make_shared<DEBUG_TriMesh>();
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    MatrixX<Rational> V_e;
    V_e.resize(4, 2);
    V_e.row(0) = Vector2r(0.75, 0.25);
    V_e.row(1) = Vector2r(0.25, 0.75);
    V_e.row(2) = Vector2r(0.25, 0.25);
    V_e.row(3) = Vector2r(0.75, 0.75);

    MatrixX<int64_t> E;
    E.resize(2, 2);
    E.row(0) << 0, 1;
    E.row(1) << 2, 3;

    std::shared_ptr<DEBUG_EdgeMesh> em = std::make_shared<DEBUG_EdgeMesh>();
    em->initialize(E);
    mesh_utils::set_matrix_attribute(V_e, "vertices", PrimitiveType::Vertex, *em);

    cache.write_mesh(*tm, "trimesh");
    cache.write_mesh(*em, "edgemesh");

    json input =
        R"({
        "edges": "edgemesh",
        "triangles": "trimesh",
        "output": "embedded_mesh"
        })"_json;

    wmtk::components::edge_insertion(Paths(), input, cache);

    json getall =
        R"({
        "input": "embedded_mesh",
        "name": "meshes"
        })"_json;

    wmtk::components::get_all_meshes(Paths(), getall, cache);

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), output, cache);

    json output2 = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_edges_out",
                "input": "embedded_mesh.input_mesh"
            })"_json;

    wmtk::components::output(Paths(), output2, cache);
}

TEST_CASE("edge_insertion_3", "[components][edge_insertion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    MatrixX<Rational> V;
    V.resize(6, 2);
    V.row(0) = Vector2r(0, 0);
    V.row(1) = Vector2r(1, 0);
    V.row(2) = Vector2r(1, 1);
    V.row(3) = Vector2r(0, 1);
    V.row(4) = Vector2r(0, 0.5);
    V.row(5) = Vector2r(1, 0.5);

    RowVectors3l F;
    F.resize(4, 3);
    F.row(0) << 0, 1, 5;
    F.row(1) << 0, 5, 4;
    F.row(2) << 4, 5, 2;
    F.row(3) << 4, 2, 3;


    std::shared_ptr<DEBUG_TriMesh> tm = std::make_shared<DEBUG_TriMesh>();
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    MatrixX<Rational> V_e;
    V_e.resize(3, 2);
    V_e.row(0) = Vector2r(0, 1);
    V_e.row(1) = Vector2r(0.5, 0);
    V_e.row(2) = Vector2r(1, 1);

    RowVectors2l E;
    E.resize(3, 2);
    E.row(0) << 0, 1;
    E.row(1) << 1, 2;
    E.row(2) << 0, 2;

    std::shared_ptr<DEBUG_EdgeMesh> em = std::make_shared<DEBUG_EdgeMesh>();
    em->initialize(E);
    mesh_utils::set_matrix_attribute(V_e, "vertices", PrimitiveType::Vertex, *em);

    cache.write_mesh(*tm, "trimesh");
    cache.write_mesh(*em, "edgemesh");

    json input =
        R"({
        "edges": "edgemesh",
        "triangles": "trimesh",
        "output": "embedded_mesh"
        })"_json;

    wmtk::components::edge_insertion(Paths(), input, cache);

    json getall =
        R"({
        "input": "embedded_mesh",
        "name": "meshes"
        })"_json;

    wmtk::components::get_all_meshes(Paths(), getall, cache);

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), output, cache);

    json output2 = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_edges_out",
                "input": "embedded_mesh.input_mesh"
            })"_json;

    wmtk::components::output(Paths(), output2, cache);
}

TEST_CASE("edge_insertion_4", "[components][edge_insertion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    MatrixX<Rational> V;
    V.resize(6, 2);
    V.row(0) = Vector2r(0, 0);
    V.row(1) = Vector2r(1, 0);
    V.row(2) = Vector2r(1, 1);
    V.row(3) = Vector2r(0, 1);
    V.row(4) = Vector2r(0, 0.5);
    V.row(5) = Vector2r(1, 0.5);

    RowVectors3l F;
    F.resize(4, 3);
    F.row(0) << 0, 1, 5;
    F.row(1) << 0, 5, 4;
    F.row(2) << 4, 5, 2;
    F.row(3) << 4, 2, 3;


    std::shared_ptr<DEBUG_TriMesh> tm = std::make_shared<DEBUG_TriMesh>();
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    MatrixX<Rational> V_e;
    V_e.resize(7, 2);
    V_e.row(0) = Vector2r(0.25, 0.25);
    V_e.row(1) = Vector2r(0.75, 0.75);
    V_e.row(2) = Vector2r(0.25, 0.375);
    V_e.row(3) = Vector2r(0.75, 0.625);
    V_e.row(4) = Vector2r(0.25, 0.75);
    V_e.row(5) = Vector2r(0.75, 0.25);
    V_e.row(6) = Vector2r(0.5, 0.825);


    RowVectors2l E;
    E.resize(4, 2);
    E.row(0) << 0, 1;
    E.row(1) << 2, 3;
    E.row(2) << 4, 5;
    E.row(3) << 5, 6;

    std::shared_ptr<DEBUG_EdgeMesh> em = std::make_shared<DEBUG_EdgeMesh>();
    em->initialize(E);
    mesh_utils::set_matrix_attribute(V_e, "vertices", PrimitiveType::Vertex, *em);

    cache.write_mesh(*tm, "trimesh");
    cache.write_mesh(*em, "edgemesh");

    json input =
        R"({
        "edges": "edgemesh",
        "triangles": "trimesh",
        "output": "embedded_mesh"
        })"_json;

    wmtk::components::edge_insertion(Paths(), input, cache);

    json getall =
        R"({
        "input": "embedded_mesh",
        "name": "meshes"
        })"_json;

    wmtk::components::get_all_meshes(Paths(), getall, cache);

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), output, cache);

    json output2 = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_edges_out",
                "input": "embedded_mesh.input_mesh"
            })"_json;

    wmtk::components::output(Paths(), output2, cache);
}

TEST_CASE("edge_insertion_5", "[components][edge_insertion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    MatrixX<Rational> V;
    V.resize(6, 2);
    V.row(0) = Vector2r(0, 0);
    V.row(1) = Vector2r(1, 0);
    V.row(2) = Vector2r(1, 1);
    V.row(3) = Vector2r(0, 1);
    V.row(4) = Vector2r(0, 0.5);
    V.row(5) = Vector2r(1, 0.5);

    RowVectors3l F;
    F.resize(4, 3);
    F.row(0) << 0, 1, 5;
    F.row(1) << 0, 5, 4;
    F.row(2) << 4, 5, 2;
    F.row(3) << 4, 2, 3;


    std::shared_ptr<DEBUG_TriMesh> tm = std::make_shared<DEBUG_TriMesh>();
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    MatrixX<Rational> V_e;
    V_e.resize(10, 2);
    V_e.row(0) = Vector2r(0.25, 0.25);
    V_e.row(1) = Vector2r(0.75, 0.75);
    V_e.row(2) = Vector2r(0.25, 0.375);
    V_e.row(3) = Vector2r(0.75, 0.625);
    V_e.row(4) = Vector2r(0.25, 0.75);
    V_e.row(5) = Vector2r(0.75, 0.25);
    V_e.row(6) = Vector2r(0.5, 0.825);
    V_e.row(7) = Vector2r(0, 1);
    V_e.row(8) = Vector2r(0.5, 0);
    V_e.row(9) = Vector2r(1, 1);


    RowVectors2l E;
    E.resize(7, 2);
    E.row(0) << 0, 1;
    E.row(1) << 2, 3;
    E.row(2) << 4, 5;
    E.row(3) << 5, 6;
    E.row(4) << 7, 8;
    E.row(5) << 8, 9;
    E.row(6) << 9, 7;

    std::shared_ptr<DEBUG_EdgeMesh> em = std::make_shared<DEBUG_EdgeMesh>();
    em->initialize(E);
    mesh_utils::set_matrix_attribute(V_e, "vertices", PrimitiveType::Vertex, *em);

    cache.write_mesh(*tm, "trimesh");
    cache.write_mesh(*em, "edgemesh");

    json input =
        R"({
        "edges": "edgemesh",
        "triangles": "trimesh",
        "output": "embedded_mesh"
        })"_json;

    wmtk::components::edge_insertion(Paths(), input, cache);

    json getall =
        R"({
        "input": "embedded_mesh",
        "name": "meshes"
        })"_json;

    wmtk::components::get_all_meshes(Paths(), getall, cache);

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), output, cache);

    json output2 = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_edges_out",
                "input": "embedded_mesh.input_mesh"
            })"_json;

    wmtk::components::output(Paths(), output2, cache);
}

TEST_CASE("edge_insertion_siggraph", "[components][edge_insertion][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    MatrixX<Rational> V;
    V.resize(25, 2);

    V.row(0) = Vector2r(-100, -100);
    V.row(1) = Vector2r(50, -100);
    V.row(2) = Vector2r(200, -100);
    V.row(3) = Vector2r(350, -100);
    V.row(4) = Vector2r(500, -100);
    V.row(5) = Vector2r(-100, 50);
    V.row(6) = Vector2r(50, 50);
    V.row(7) = Vector2r(200, 50);
    V.row(8) = Vector2r(350, 50);
    V.row(9) = Vector2r(500, 50);
    V.row(10) = Vector2r(-100, 200);
    V.row(11) = Vector2r(50, 200);
    V.row(12) = Vector2r(200, 200);
    V.row(13) = Vector2r(350, 200);
    V.row(14) = Vector2r(500, 200);
    V.row(15) = Vector2r(-100, 350);
    V.row(16) = Vector2r(50, 350);
    V.row(17) = Vector2r(200, 350);
    V.row(18) = Vector2r(350, 350);
    V.row(19) = Vector2r(500, 350);
    V.row(20) = Vector2r(-100, 500);
    V.row(21) = Vector2r(50, 500);
    V.row(22) = Vector2r(200, 500);
    V.row(23) = Vector2r(350, 500);
    V.row(24) = Vector2r(500, 500);


    RowVectors3l F;
    F.resize(32, 3);
    F.row(0) << 1, 2, 7;
    F.row(1) << 1, 7, 6;
    F.row(2) << 2, 3, 8;
    F.row(3) << 2, 8, 7;
    F.row(4) << 3, 4, 9;
    F.row(5) << 3, 9, 8;
    F.row(6) << 4, 5, 10;
    F.row(7) << 4, 10, 9;
    F.row(8) << 6, 7, 12;
    F.row(9) << 6, 12, 11;
    F.row(10) << 7, 8, 13;
    F.row(11) << 7, 13, 12;
    F.row(12) << 8, 9, 14;
    F.row(13) << 8, 14, 13;
    F.row(14) << 9, 10, 15;
    F.row(15) << 9, 15, 14;
    F.row(16) << 11, 12, 17;
    F.row(17) << 11, 17, 16;
    F.row(18) << 12, 13, 18;
    F.row(19) << 12, 18, 17;
    F.row(20) << 13, 14, 19;
    F.row(21) << 13, 19, 18;
    F.row(22) << 14, 15, 20;
    F.row(23) << 14, 20, 19;
    F.row(24) << 16, 17, 22;
    F.row(25) << 16, 22, 21;
    F.row(26) << 17, 18, 23;
    F.row(27) << 17, 23, 22;
    F.row(28) << 18, 19, 24;
    F.row(29) << 18, 24, 23;
    F.row(30) << 19, 20, 25;
    F.row(31) << 19, 25, 24;

    for (int64_t i = 0; i < F.rows(); ++i) {
        for (int64_t j = 0; j < F.cols(); ++j) {
            F(i, j) -= 1;
        }
    }


    std::shared_ptr<DEBUG_TriMesh> tm = std::make_shared<DEBUG_TriMesh>();
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    cache.write_mesh(*tm, "trimesh");

    wmtk::logger().info("read edges");

    json edge_input =
        R"({
            "file": "/home/jiacheng/jiacheng/wmtk_hack/wildmeshing-toolkit/build_ei/siggraph_icon.msh",
            "name": "edgemesh",
            "ignore_z": true,
            "tetrahedron_attributes": []
        })"_json;

    wmtk::components::input(Paths(), edge_input, cache);

    // wmtk::logger().info("read tris");

    // json tri_input =
    //     R"({
    //         "file": "/home/jiacheng/jiacheng/wmtk_hack/wildmeshing-toolkit/build_ei/background.msh",
    //         "name": "trimesh",
    //         "ignore_z": true,
    //         "tetrahedron_attributes": []
    //     })"_json;

    // wmtk::components::input(Paths(), tri_input, cache);


    json input =
        R"({
        "edges": "edgemesh",
        "triangles": "trimesh",
        "output": "embedded_mesh"
        })"_json;

    wmtk::logger().info("start edge insertion");

    wmtk::components::edge_insertion(Paths(), input, cache);

    wmtk::logger().info("end edge insertion");

    json getall =
        R"({
        "input": "embedded_mesh",
        "name": "meshes"
        })"_json;

    wmtk::components::get_all_meshes(Paths(), getall, cache);

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "siggraph_icon_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), output, cache);

    json output2 = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "siggraph_icon_edges_out",
                "input": "embedded_mesh.input_mesh"
            })"_json;

    wmtk::components::output(Paths(), output2, cache);
}