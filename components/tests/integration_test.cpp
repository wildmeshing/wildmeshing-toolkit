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

enum class ResultType { Success = 0, JSONParseFailure = 1, ResultDiverged = 2 };

bool contains_results(const json& in_args)
{
    const auto& tests = in_args["tests"];
    for (const auto& type : {"vertices", "edges", "faces", "tetrahedra"}) {
        if (!(tests.contains(type) && (tests[type].is_number() || tests[type].is_array()))) {
            spdlog::info("{} {}", tests.contains(type), tests[type].is_number());
            return false;
        }
    }
    return true;
}

bool missing_tests_data(const json& j)
{
    return !j.contains("tests") || !j.at("tests").contains("meshes");
}

ResultType authenticate_json(const std::string& json_file, const bool compute_validation)
{
    json in_args;
    if (!load_json(json_file, in_args)) {
        spdlog::error("unable to open {} file", json_file);
        return ResultType::JSONParseFailure;
    }

    if (missing_tests_data(in_args)) {
        spdlog::error("JSON file missing \"tests\" or meshes key.");
        return ResultType::JSONParseFailure;
    }
    in_args["root_path"] = json_file;


    if (compute_validation && !contains_results(in_args)) {
        spdlog::error("JSON file missing vertices edges faces or tetrahedra or meshes key. Add a * "
                      "to the beginning of filename to allow appends.");
        return ResultType::JSONParseFailure;
    }

    // in_args["settings"] = R"({
    //     "log_level": 5,
    //     "opt_log_level": 5
    //     })"_json;

    utils::set_random_seed(0);
    auto cache = wmtk::components::run_components(in_args, true);

    auto meshes = in_args["tests"]["meshes"];

    if (!compute_validation) {
        spdlog::info("Authenticating...");

        if (in_args["tests"].contains("skip_check") && in_args["tests"]["skip_check"]) {
            spdlog::warn("Skpping checks for {}", json_file);
            return ResultType::Success;
        }

        auto vertices = in_args["tests"]["vertices"];
        auto edges = in_args["tests"]["edges"];
        auto faces = in_args["tests"]["faces"];
        auto tetrahedra = in_args["tests"]["tetrahedra"];
        if (meshes.size() != vertices.size() || meshes.size() != edges.size() ||
            meshes.size() != faces.size() || meshes.size() != tetrahedra.size()) {
            spdlog::error("JSON size missmatch between meshes and vertices edges faces or "
                          "tetrahedra or meshes key. Add a * "
                          "to the beginning of filename to allow appends.");
            return ResultType::ResultDiverged;
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
                    "Violating Authenticate for vertices {}, expected {}",
                    n_vertices,
                    expected_vertices);
                return ResultType::ResultDiverged;
            }
            if (n_edges != expected_edges) {
                spdlog::error(
                    "Violating Authenticate for edges got {}, expected {}",
                    n_edges,
                    expected_edges);
                return ResultType::ResultDiverged;
            }
            if (n_faces != expected_faces) {
                spdlog::error(
                    "Violating Authenticate for faces got {}, expected {}",
                    n_faces,
                    expected_faces);
                return ResultType::ResultDiverged;
            }
            if (n_tetrahedra != expected_tetrahedra) {
                spdlog::error(
                    "Violating Authenticate for tetrahedra got {}, expected {}",
                    n_tetrahedra,
                    expected_tetrahedra);
                return ResultType::ResultDiverged;
            }
        }

        spdlog::info("Authenticated ✅");
    } else {
        spdlog::warn("Appending JSON...");

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

    return ResultType::Success;
}
} // namespace
namespace {
#if defined(NDEBUG)
std::string tagsrun = "[integration]";
#else
std::string tagsrun = "[.][integration]";
#endif
} // namespace

#define WMTK_INTEGRATION_BODY(NAME, DO_VALIDATION)                                   \
    {                                                                                \
        std::string path = std::string("unit_test/") + NAME + ".json";               \
        bool compute_validation = DO_VALIDATION;                                     \
        spdlog::info("Processing {}", NAME);                                         \
        auto flag = authenticate_json(WMTK_DATA_DIR "/" + path, compute_validation); \
        REQUIRE(flag == ResultType::Success);                                        \
    }

#define WMTK_INTEGRATION(NAME, DO_VALIDATION)              \
    TEST_CASE(std::string("integration_") + NAME, tagsrun) \
    WMTK_INTEGRATION_BODY(NAME, DO_VALIDATION)


WMTK_INTEGRATION("input", false);
WMTK_INTEGRATION("to_points", false);
WMTK_INTEGRATION("delaunay", false);
WMTK_INTEGRATION("insertion", false);
WMTK_INTEGRATION("insertion_open", false);
WMTK_INTEGRATION("multimesh", false);
WMTK_INTEGRATION("multimesh_boundary_2d", false);
WMTK_INTEGRATION("multimesh_boundary_3d", false);
WMTK_INTEGRATION("isotropic_remeshing", false);
WMTK_INTEGRATION("isotropic_remeshing_mm", false);
WMTK_INTEGRATION("disk_fan_mm", false);
WMTK_INTEGRATION("grid", false);
WMTK_INTEGRATION("wildmeshing_2d", true);
// WMTK_INTEGRATION("wildmeshing_3d", false);
WMTK_INTEGRATION("marching", false);


TEST_CASE("integration_benchmark", "[.][benchmark][integration]")
{
    // WMTK_INTEGRATION_BODY("wildmeshing_2d_timing",false)
    // WMTK_INTEGRATION_BODY("wildmeshing_3d_timing",false)
    WMTK_INTEGRATION_BODY("isotropic_remeshing_mm_timing", true)
}
