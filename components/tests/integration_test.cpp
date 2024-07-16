////////////////////////////////////////////////////////////////////////////////
#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/run_components.hpp>

#include <wmtk/utils/random_seed.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////

using json = nlohmann::json;
using namespace wmtk;
namespace {

bool load_json(const std::string& json_file, json& out)
{
    std::ifstream file(json_file);

    if (!file.is_open()) return false;

    file >> out;

    return true;
}

bool contains_results(const json& in_args)
{
    if (!in_args.contains("tests")) {
        return false;
    }

    const auto& tests = in_args["tests"];
    for (const auto& type : {"meshes", "vertices", "edges", "faces", "tetrahedra"}) {
        if (!tests.contains(type) || !tests[type].is_array()) {
            return false;
        }
    }
    return true;
}

bool missing_tests_data(const json& j)
{
    return !j.contains("tests") || !j.at("tests").contains("meshes");
}

int authenticate_json(const std::string& json_file, const bool compute_validation)
{
    json in_args;
    if (!load_json(json_file, in_args)) {
        spdlog::error("unable to open {} file", json_file);
        return 1;
    }

    in_args["root_path"] = json_file;


    // in_args["settings"] = R"({
    //     "log_level": 5,
    //     "opt_log_level": 5
    //     })"_json;

    utils::set_random_seed(0);
    auto cache = wmtk::components::run_components(in_args, true);

    if (!compute_validation) {
        if (missing_tests_data(in_args)) {
            spdlog::error("JSON file missing \"tests\" or meshes key.");
            return 1;
        }

        REQUIRE(contains_results(in_args));

        spdlog::info("Authenticating...");

        const std::vector<std::string> meshes = in_args["tests"]["meshes"];

        if (in_args["tests"].contains("skip_check") && in_args["tests"]["skip_check"]) {
            spdlog::warn("Skpping checks for {}", json_file);
            return 0;
        }

        const auto vertices = in_args["tests"]["vertices"];
        const auto edges = in_args["tests"]["edges"];
        const auto faces = in_args["tests"]["faces"];
        const auto tetrahedra = in_args["tests"]["tetrahedra"];
        if (meshes.size() != vertices.size() || meshes.size() != edges.size() ||
            meshes.size() != faces.size() || meshes.size() != tetrahedra.size()) {
            spdlog::error(
                "JSON size missmatch between meshes and vertices, edges, faces, or tetrahedra.");
            return 2;
        }

        for (int64_t i = 0; i < meshes.size(); ++i) {
            const std::shared_ptr<Mesh> mesh = cache.read_mesh(meshes[i]);

            const int64_t expected_vertices = vertices[i];
            const int64_t expected_edges = edges[i];
            const int64_t expected_faces = faces[i];
            const int64_t expected_tetrahedra = tetrahedra[i];

            const int64_t n_vertices = mesh->get_all(PrimitiveType::Vertex).size();
            const int64_t n_edges = mesh->get_all(PrimitiveType::Edge).size();
            const int64_t n_faces = mesh->get_all(PrimitiveType::Triangle).size();
            const int64_t n_tetrahedra = mesh->get_all(PrimitiveType::Tetrahedron).size();

            if (n_vertices != expected_vertices) {
                spdlog::error(
                    "Violating Authenticate in mesh `{}` for vertices {} != {}",
                    meshes[i],
                    n_vertices,
                    expected_vertices);
                return 2;
            }
            if (n_edges != expected_edges) {
                spdlog::error(
                    "Violating Authenticate in mesh `{}` for edges {} != {}",
                    meshes[i],
                    n_edges,
                    expected_edges);
                return 2;
            }
            if (n_faces != expected_faces) {
                spdlog::error(
                    "Violating Authenticate in mesh `{}` for faces {} != {}",
                    meshes[i],
                    n_faces,
                    expected_faces);
                return 2;
            }
            if (n_tetrahedra != expected_tetrahedra) {
                spdlog::error(
                    "Violating Authenticate in mesh `{}` for tetrahedra {} != {}",
                    meshes[i],
                    n_tetrahedra,
                    expected_tetrahedra);
                return 2;
            }
        }

        spdlog::info("Authenticated ✅");
    } else {
        if (contains_results(in_args)) {
            spdlog::error(
                "JSON file contains results even though the test was run in `computation "
                "mode`. Set DO_VALIDATION to false to compare with saved results or remove "
                "results from JSON to re-compute them.");
            return 2;
        }

        spdlog::warn("Appending JSON...");

        const std::vector<std::string> meshes = cache.mesh_names();
        in_args["tests"]["meshes"] = meshes;

        std::vector<int64_t> expected_vertices, expected_edges, expected_faces, expected_tetrahedra;

        for (int64_t i = 0; i < meshes.size(); ++i) {
            const std::shared_ptr<Mesh> mesh = cache.read_mesh(meshes[i]);


            const int64_t n_vertices = mesh->get_all(PrimitiveType::Vertex).size();
            const int64_t n_edges = mesh->get_all(PrimitiveType::Edge).size();
            const int64_t n_faces = mesh->get_all(PrimitiveType::Triangle).size();
            const int64_t n_tetrahedra = mesh->get_all(PrimitiveType::Tetrahedron).size();

            expected_vertices.push_back(n_vertices);
            expected_edges.push_back(n_edges);
            expected_faces.push_back(n_faces);
            expected_tetrahedra.push_back(n_tetrahedra);
        }

        in_args["tests"]["vertices"] = expected_vertices;
        in_args["tests"]["edges"] = expected_edges;
        in_args["tests"]["faces"] = expected_faces;
        in_args["tests"]["tetrahedra"] = expected_tetrahedra;
        in_args.erase("root_path");
        in_args.erase("settings");

        std::ofstream file(json_file);
        file << in_args;
    }

    return 0;
}
} // namespace
namespace {
#if defined(NDEBUG)
std::string tagsrun = "[integration]";
#else
std::string tagsrun = "[.][integration]";
#endif
} // namespace

#define WMTK_INTEGRATION(NAME, DO_VALIDATION)                                        \
    TEST_CASE(std::string("integration_") + NAME, tagsrun)                           \
    {                                                                                \
        std::string path = std::string("unit_test/") + NAME + ".json";               \
        bool compute_validation = DO_VALIDATION;                                     \
        spdlog::info("Processing {}", NAME);                                         \
        auto flag = authenticate_json(WMTK_DATA_DIR "/" + path, compute_validation); \
        REQUIRE(flag == 0);                                                          \
    }


WMTK_INTEGRATION("input", false);
WMTK_INTEGRATION("to_points", false);
WMTK_INTEGRATION("delaunay", false);
WMTK_INTEGRATION("insertion", false);
WMTK_INTEGRATION("insertion_open", false);
WMTK_INTEGRATION("multimesh", false);
WMTK_INTEGRATION("multimesh_boundary_2d", false);
WMTK_INTEGRATION("multimesh_boundary_3d", false);
// WMTK_INTEGRATION("isotropic_remeshing", false);
// WMTK_INTEGRATION("isotropic_remeshing_mm", false);
WMTK_INTEGRATION("disk_fan_mm", false);
// WMTK_INTEGRATION("grid",false);
// WMTK_INTEGRATION("wildmeshing_2d", false);
// WMTK_INTEGRATION("wildmeshing_3d", false);
WMTK_INTEGRATION("marching", false);
