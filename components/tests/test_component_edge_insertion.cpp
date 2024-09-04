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