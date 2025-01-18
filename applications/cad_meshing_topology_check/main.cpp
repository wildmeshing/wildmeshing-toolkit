
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/element_count_report.hpp>
#include <wmtk/applications/utils/get_integration_test_data_root.hpp>


#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/input.hpp>

#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/output/parse_output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include <wmtk/io/HDF5Reader.hpp>

#include <wmtk/utils/TupleInspector.hpp>
#include "CLI/CLI.hpp"
#include "wmtk/components/multimesh/MeshCollection.hpp"
#include "wmtk/components/multimesh/utils/AttributeDescription.hpp"
#include "wmtk/components/multimesh/utils/get_attribute.hpp"
#include "wmtk/components/utils/PathResolver.hpp"
using namespace wmtk::components;
using namespace wmtk::applications;
using namespace wmtk;
namespace fs = std::filesystem;

void should_have(
    const nlohmann::json& ref_data,
    std::set<int64_t>& ref_global_d_set,
    const std::set<int64_t>& my_global_d_set)
{
    // Example: Accessing the data

    for (const auto& [key, value] : ref_data.items()) {
        auto global_d = std::stoi(key);
        ref_global_d_set.insert(global_d);
    }

    std::set<int64_t> should_be_set, should_not_be_set;

    std::set_difference(
        ref_global_d_set.begin(),
        ref_global_d_set.end(),
        my_global_d_set.begin(),
        my_global_d_set.end(),
        std::inserter(should_be_set, should_be_set.begin()));

    std::set_difference(
        my_global_d_set.begin(),
        my_global_d_set.end(),
        ref_global_d_set.begin(),
        ref_global_d_set.end(),
        std::inserter(should_not_be_set, should_not_be_set.begin()));
    if (!should_be_set.empty()) {
        logger().error(" {} SHOULD BE in the output", should_be_set);
    }
    if (!should_not_be_set.empty()) {
        logger().error("{} should NOT be in the output", should_not_be_set);
    }
}

int check_v_to_e(
    const wmtk::Mesh& point_mesh,
    const wmtk::Mesh& edge_mesh,
    const fs::path& v_to_e_path)
{
    nlohmann::json v_to_e_ref_data;

    try {
        std::ifstream ifs(v_to_e_path);
        if (!ifs.is_open()) {
            logger().error("v_to_e {}", v_to_e_path.string());
            throw std::ios_base::failure("Error: Unable to open file");
        }

        // Parse the JSON file
        v_to_e_ref_data = nlohmann::json::parse(ifs);

        // Check if the JSON object is empty
        if (v_to_e_ref_data.empty()) {
            throw std::runtime_error("Error: JSON file is empty or invalid.");
        }

        std::cout << "JSON loaded successfully!" << std::endl;
        // Further processing of v_to_e_ref_data
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    logger().info("Checking v_to_e");

    // TODO check if all the vertices are in the v_to_e_ref_data
    std::set<int64_t> my_global_d_set, ref_global_d_set;
    for (const auto& t : point_mesh.get_all(PrimitiveType::Vertex)) {
        int64_t global_d = wmtk::utils::TupleInspector::global_cid(t);
        my_global_d_set.insert(global_d);
    }
    should_have(v_to_e_ref_data, ref_global_d_set, my_global_d_set);
    auto edge_label_on_edge_mesh =
        edge_mesh.get_attribute_handle<int64_t>("edge_labels", PrimitiveType::Edge);
    auto edge_acc = edge_mesh.create_const_accessor<int64_t>(edge_label_on_edge_mesh);
    for (const auto& v : point_mesh.get_all(PrimitiveType::Vertex)) {
        int64_t global_d = wmtk::utils::TupleInspector::global_cid(v);
        std::set<int64_t> ref_es;
        if (ref_global_d_set.find(global_d) != ref_global_d_set.end()) {
            ref_es = v_to_e_ref_data[std::to_string(global_d)].get<std::set<int64_t>>();
        }
        if (!point_mesh.is_valid(v)) {
            logger().critical("Vertex {} is NOT valid", global_d);
        }
        auto edges = point_mesh.map(edge_mesh, simplex::Simplex(PrimitiveType::Vertex, v));
        std::set<int64_t> my_es;
        for (const auto& e : edges) {
            int64_t edge_d = edge_acc.const_scalar_attribute(e.tuple());
            my_es.insert(edge_d);
            // edge_d is next to critical vertex critical_point_tuple.global_id
        }
        if (my_es != ref_es) {
            logger().error("Vertex {} has edges {}", global_d, my_es);
            logger().error("Vertex {} should have edges {}", global_d, ref_es);
        }
    }
    return 0;
}

int check_e_to_f(
    const wmtk::Mesh& edge_mesh,
    const wmtk::Mesh& triangle_mesh,
    const fs::path& e_to_f_path)
{
    nlohmann::json e_to_f_ref_data;

    try {
        logger().info("Loading JSON file: {}", e_to_f_path.string());
        std::ifstream ifs(e_to_f_path);
        if (!ifs.is_open()) {
            logger().error("e_to_f {}", e_to_f_path.string());
            throw std::ios_base::failure("Error: Unable to open file");
        }

        // Parse the JSON file
        logger().info("Parsing JSON file");
        e_to_f_ref_data = nlohmann::json::parse(ifs);

        // Check if the JSON object is empty
        if (e_to_f_ref_data.empty()) {
            throw std::runtime_error("Error: JSON file is empty or invalid.");
        }

        std::cout << "JSON loaded successfully!" << std::endl;
        // Further processing of e_to_f_ref_data
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    logger().info("Checking e_to_f");

    std::set<int64_t> my_global_d_set, ref_global_d_set;

    auto edge_label_on_edge_mesh =
        edge_mesh.get_attribute_handle<int64_t>("edge_labels", PrimitiveType::Edge);
    auto edge_acc = edge_mesh.create_const_accessor<int64_t>(edge_label_on_edge_mesh);
    for (const auto& t : edge_mesh.get_all(PrimitiveType::Edge)) {
        int64_t global_d = edge_acc.const_scalar_attribute(t);
        my_global_d_set.insert(global_d);
    }
    should_have(e_to_f_ref_data, ref_global_d_set, my_global_d_set);
    auto triangle_label_on_triangle_mesh =
        triangle_mesh.get_attribute_handle<int64_t>("patch_labels", PrimitiveType::Triangle);
    auto triangle_acc =
        triangle_mesh.create_const_accessor<int64_t>(triangle_label_on_triangle_mesh);
    for (const auto& e : edge_mesh.get_all(PrimitiveType::Edge)) {
        int64_t global_d = edge_acc.const_scalar_attribute(e);
        std::set<int64_t> ref_fs;
        if (ref_global_d_set.find(global_d) != ref_global_d_set.end()) {
            ref_fs = e_to_f_ref_data[std::to_string(global_d)].get<std::set<int64_t>>();
        }

        auto triangles = edge_mesh.map(triangle_mesh, simplex::Simplex(PrimitiveType::Edge, e));
        std::set<int64_t> my_fs;
        for (const auto& t : triangles) {
            int64_t triangle_d = triangle_acc.const_scalar_attribute(t.tuple());
            my_fs.insert(triangle_d);
            // edge_d is next to critical vertex critical_point_tuple.global_id
        }
        if (my_fs != ref_fs) {
            logger().error("Edge {} has patches {}", global_d, my_fs);
            logger().error("Edge {} should have patches {}", global_d, ref_fs);
        }
    }
    return 0;
}

int check_topology_internal(
    const nlohmann::json& j,
    const std::optional<fs::path>& name_spec_file,
    const std::optional<fs::path>& integration_test_config_file)
{
    // if (name_spec_file.has_value()) {
    //     j["name"] = nlohmann::json::parse(std::ifstream(name_spec_file.value()));
    // }
    wmtk::components::multimesh::MeshCollection meshes;


    std::shared_ptr<wmtk::Mesh> output_mesh;
    if (j["input"].is_array()) return 2;

    wmtk::components::input::InputOptions opts = j["input"];
    const auto file_path = opts.path;
    if (!std::filesystem::exists(file_path)) {
        logger().error("File does not exist: {}", file_path.string());
        return 1;
    }
    wmtk::HDF5Reader reader;
    auto mesh = reader.read(file_path);
    const auto& point_mesh = mesh->get_multi_mesh_mesh({2});
    const auto& edge_mesh = mesh->get_multi_mesh_mesh({1});
    const auto& face_mesh = mesh->get_multi_mesh_mesh({0});

    const auto v_to_e_path = j["v_to_e"]["path"];
    const auto e_to_f_path = j["e_to_f"]["path"];

    check_v_to_e(point_mesh, edge_mesh, v_to_e_path);
    check_e_to_f(edge_mesh, face_mesh, e_to_f_path);


    return 0;
}


int check_topology(
    const fs::path& config_path,
    const std::optional<fs::path>& name_spec_file,
    const std::optional<fs::path>& integration_test_config_file)
{
    nlohmann::json j;
    std::ifstream ifs(config_path);
    j = nlohmann::json::parse(ifs);
    return check_topology_internal(j, name_spec_file, integration_test_config_file);
}

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    std::optional<fs::path> json_integration_config_file;
    std::optional<std::string> json_integration_app_name;
    std::optional<fs::path> name_spec_file;

    auto add_it_path = [&](CLI::App& a) {
        a.add_option(
             "-c, --integration-test-config",
             json_integration_config_file,
             "Test config file for integration test")
            ->check(CLI::ExistingFile);
        a.add_option(
             "-a, --integration-test-app",
             json_integration_config_file,
             "Test config file for integration test")
            ->check(CLI::ExistingFile);
    };

    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);

    add_it_path(app);


    fs::path input;
    fs::path output;
    std::string type;


    CLI11_PARSE(app, argc, argv);


    int exit_mode = -1;

    exit_mode = check_topology(json_input_file, name_spec_file, json_integration_config_file);


    assert(exit_mode != -1); // "Some subcommand should have updated the exit mode"
    return exit_mode;
}
