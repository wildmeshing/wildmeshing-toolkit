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

    std::shared_ptr<TriMesh> tm;
    tm->initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    MatrixX<Rational> V_e;
    V_e.resize(2, 2);
    V_e.row(0) = Vector2r(0.75, 0.25);
    V_e.row(1) = Vector2r(0.25, 0.75);

    MatrixX<int64_t> E;
    E.resize(1, 2);
    E.row(0) << 0, 1;

    std::shared_ptr<EdgeMesh> em;
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

    json output = R"({
                "attributes": {
                    "position": "vertices"
                },
                "file": "test_ei_out",
                "input": "embedded_mesh"
            })"_json;

    wmtk::components::output(Paths(), input, cache);
}