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

bool load_json(const std::string& json_file, json& out)
{
    std::ifstream file(json_file);

    if (!file.is_open()) return false;

    file >> out;

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

    if (missing_tests_data(in_args)) {
        spdlog::error("JSON file missing \"tests\" or meshes key.");
        return 1;
    }
    in_args["root_path"] = json_file;


    if (!compute_validation &&
        (!in_args["tests"].contains("vertices") || !in_args["tests"].contains("edges") ||
         !in_args["tests"].contains("faces") || !in_args["tests"].contains("tetrahedra"))) {
        spdlog::error("JSON file missing vertices edges faces or tetrahedra or meshes key. Add a * "
                      "to the beginning of filename to allow appends.");
        return 2;
    }

    in_args["settings"] = R"({
        "log_level": 5,
        "opt_log_level": 5
        })"_json;

    utils::set_random_seed(0);
    auto cache = wmtk::components::run_components(in_args, true);

    auto meshes = in_args["tests"]["meshes"];

    if (!compute_validation) {
        spdlog::info("Authenticating...");

        if (in_args["tests"].contains("skip_check") && in_args["tests"]["skip_check"]) {
            spdlog::warn("Skpping checks for {}", json_file);
            return 0;
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
                    "Violating Authenticate for vertices {}!={}",
                    n_vertices,
                    expected_vertices);
                return 2;
            }
            if (n_edges != expected_edges) {
                spdlog::error("Violating Authenticate for edges {}!={}", n_edges, expected_edges);
                return 2;
            }
            if (n_faces != expected_faces) {
                spdlog::error("Violating Authenticate for faces {}!={}", n_faces, expected_faces);
                return 2;
            }
            if (n_tetrahedra != expected_tetrahedra) {
                spdlog::error(
                    "Violating Authenticate for tetrahedra {}!={}",
                    n_tetrahedra,
                    expected_tetrahedra);
                return 2;
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

    return 0;
}

#if defined(NDEBUG)
std::string tagsrun = "[integration]";
#else
std::string tagsrun = "[.][integration]";
#endif
TEST_CASE("integration", tagsrun)
{
    // Disabled on Windows CI, due to the requirement for Pardiso.
    std::ifstream file(WMTK_TEST_DIR "/integration_test_list.txt");
    std::vector<std::string> failing_tests;
    std::string line;
    while (std::getline(file, line)) {
        bool compute_validation = false;
        if (line[0] == '#')
            continue;
        else if (line[0] == '*') {
            compute_validation = true;
            line = line.substr(1);
        }
        spdlog::info("Processing {}", line);
        auto flag = authenticate_json(WMTK_DATA_DIR "/" + line, compute_validation);
        CAPTURE(line);
        CHECK(flag == 0);
        if (flag != 0) failing_tests.push_back(line);
    }
    if (failing_tests.size() > 0) {
        std::cout << "Failing tests:" << std::endl;
        for (auto& t : failing_tests) std::cout << t << std::endl;
    }
}
