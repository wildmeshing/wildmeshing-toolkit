
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


int check_v_to_e(const wmtk::Mesh& point_mesh, const wmtk::Mesh& edge_mesh)
{
    for (const auto& t : point_mesh.get_all(PrimitiveType::Vertex)) {
        int64_t global_d = wmtk::utils::TupleInspector::global_cid(t);
    }
    auto edge_label_on_edge_mesh =
        edge_mesh.get_attribute_handle<int64_t>("edge_labels", PrimitiveType::Edge);
    auto edge_acc = edge_mesh.create_const_accessor<int64_t>(edge_label_on_edge_mesh);
    for (const auto& v : point_mesh.get_all(PrimitiveType::Vertex)) {
        auto edges = point_mesh.map(edge_mesh, simplex::Simplex(PrimitiveType::Vertex, v));
        for (const auto& e : edges) {
            int64_t edge_d = edge_acc.const_scalar_attribute(e.tuple());
            // edge_d is next to critical vertex critical_point_tuple.global_id
        }
    }
    return 0;
}

int check_e_to_f(const wmtk::Mesh& edge_mesh, const wmtk::Mesh& triangle_mesh)
{
    for (const auto& t : edge_mesh.get_all(PrimitiveType::Edge)) {
        int64_t global_d = wmtk::utils::TupleInspector::global_cid(t);
    }
    auto triangle_label_on_triangle_mesh =
        triangle_mesh.get_attribute_handle<int64_t>("patch_labels", PrimitiveType::Triangle);
    auto triangle_acc =
        triangle_mesh.create_const_accessor<int64_t>(triangle_label_on_triangle_mesh);
    for (const auto& e : edge_mesh.get_all(PrimitiveType::Edge)) {
        auto triangles = edge_mesh.map(triangle_mesh, simplex::Simplex(PrimitiveType::Edge, e));
        for (const auto& t : triangles) {
            int64_t triangle_d = triangle_acc.const_scalar_attribute(t.tuple());
            // edge_d is next to critical vertex critical_point_tuple.global_id
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
    wmtk::HDF5Reader reader;
    auto mesh = reader.read(file_path);
    const auto& point_mesh = mesh->get_multi_mesh_mesh({2});
    const auto& edge_mesh = mesh->get_multi_mesh_mesh({1});
    const auto& face_mesh = mesh->get_multi_mesh_mesh({0});

    check_v_to_e(point_mesh, edge_mesh);
    check_e_to_f(edge_mesh, face_mesh);

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