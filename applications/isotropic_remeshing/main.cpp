
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/element_count_report.hpp>
#include <wmtk/applications/utils/get_integration_test_data_root.hpp>
#include <wmtk/applications/utils/parse_jse.hpp>
#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/multimesh/utils/AttributeDescription.hpp>
#include <wmtk/components/multimesh/utils/get_attribute.hpp>
#include "wmtk/components/utils/PathResolver.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshingOptions.hpp>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "make_multimesh.hpp"
#include "spec.hpp"

using namespace wmtk::components;
using namespace wmtk;
namespace fs = std::filesystem;

constexpr static std::string root_attribute_name = "root";

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    fs::path json_integration_config_file;
    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);
    app.add_option(
           "-i, --integration-test-config",
           json_integration_config_file,
           "Test config file for integration test")
        ->check(CLI::ExistingFile);
    CLI11_PARSE(app, argc, argv);

    // nlohmann::json j = wmtk::applications::utils::parse_jse(
    //     wmtk::applications::isotropic_remeshing::spec,
    //     json_input_file);

    spdlog::warn("File is {}", json_input_file.string());
    std::ifstream ifs(json_input_file);
    nlohmann::json j = nlohmann::json::parse(ifs);

    const auto input_js = j["input"];
    components::utils::PathResolver path_resolver;

    if (j.contains(root_attribute_name)) {
        path_resolver = j[root_attribute_name];
    }
    if (!json_integration_config_file.empty()) {
        auto path = wmtk::applications::utils::get_integration_test_data_root(
            json_integration_config_file,
            argv[0]);
        path_resolver.add_path(path);
    }


    auto input_opts = input_js.get<wmtk::components::input::InputOptions>();


    wmtk::components::multimesh::MeshCollection mc;

    auto& named_mesh = mc.add_mesh(wmtk::components::input::input(input_opts, path_resolver));

    auto mesh_ptr = named_mesh.root().shared_from_this();
    if (input_js.contains("multimesh")) {
        mesh_ptr = make_multimesh(*mesh_ptr, input_js["multimesh"]);

        spdlog::info("{} children", mesh_ptr->get_all_child_meshes().size());
    }


    if(!mc.is_valid()) {
        wmtk::logger().error("Input mesh did not match the name specification, going to throw an exception to help debugging");
        mc.is_valid(true);
    }

    wmtk::components::isotropic_remeshing::IsotropicRemeshingOptions options;

    options.load_json(j);

    options.position_attribute =
        wmtk::components::multimesh::utils::get_attribute(mc, j["position_attribute"]);

    assert(options.position_attribute.is_valid());

    if (j.contains("inversion_position_attribute")) {
        options.inversion_position_attribute = wmtk::components::multimesh::utils::get_attribute(
            mc,
            j["inversion_position_attribute"]);
    }
    if (j.contains("other_position_attributes")) {
        for (const auto& other : j["other_position_attributes"]) {
            options.other_position_attributes.emplace_back(
                wmtk::components::multimesh::utils::get_attribute(mc, other));
        }
    }
    if (j.contains("pass_through_attributes")) {
        for (const auto& other : j["pass_through_attributes"]) {
            options.pass_through_attributes.emplace_back(
                wmtk::components::multimesh::utils::get_attribute(mc, other));
            assert(options.pass_through_attributes.back().is_valid());
        }
    }
    for (const auto& attr : options.pass_through_attributes) {
        spdlog::info("Pass through: {}", attr.name());
    }
    if (j.contains("static_mesh_names")) {
        for (const auto& other : j["static_mesh_names"]) {
            options.static_mesh_names.emplace_back( other);
        }
    }

    options.mesh_collection = &mc;
    if(j.contains("intermediate_output_format")) {
        options.intermediate_output_format = j["intermediate_output_format"];
    }
    assert(options.position_attribute.is_valid());
    wmtk::components::isotropic_remeshing::isotropic_remeshing(options);

    // input uv mesh

    // multimesh the input meshes if not already multimeshed
    // OR - should this be a diff app?

    // call isotropic_remeshing


    auto out_opts = j["output"].get<wmtk::components::output::OutputOptions>();
    out_opts.position_attribute = options.position_attribute;
    wmtk::components::output::output(*mesh_ptr, out_opts);

    if (j.contains("report")) {
        const std::string report = j["report"];
        mc.make_canonical();
        if (!report.empty()) {
            nlohmann::json out_json;
            auto& stats = out_json["stats"];
            stats = wmtk::applications::utils::element_count_report_named(mc);
            j.erase("report");
            if (j.contains(root_attribute_name)) {
                j.erase(root_attribute_name);
            }
            out_json["input"] = j;


            std::ofstream ofs(report);
            ofs << std::setw(2) << out_json;
        }
    }
}
