
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/element_count_report.hpp>
#include <wmtk/applications/utils/get_integration_test_data_root.hpp>
#include "read_vid_pairs.hpp"
#include "run.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/input.hpp>

#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/output/parse_output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "CLI/CLI.hpp"
#include "wmtk/components/multimesh/MeshCollection.hpp"
#include "wmtk/components/multimesh/utils/AttributeDescription.hpp"
#include "wmtk/components/multimesh/utils/get_attribute.hpp"
#include "wmtk/components/utils/PathResolver.hpp"

using namespace wmtk::components;
using namespace wmtk::applications;
using namespace wmtk;
namespace fs = std::filesystem;


int run_js(
    const std::string_view& app_name,
    const nlohmann::json& j,
    const std::optional<fs::path>& name_spec_file,
    const std::optional<fs::path>& integration_test_config_file)
{
    // if (name_spec_file.has_value()) {
    //     j["name"] = nlohmann::json::parse(std::ifstream(name_spec_file.value()));
    // }
    wmtk::components::multimesh::MeshCollection meshes;
    components::utils::PathResolver path_resolver;

    if (j.contains("root")) {
        path_resolver = j["root"];
    }
    if (integration_test_config_file.has_value()) {
        auto path = wmtk::applications::utils::get_integration_test_data_root(
            integration_test_config_file.value(),
            app_name);
        path_resolver.add_path(path);
    }

    std::shared_ptr<wmtk::Mesh> output_mesh;
    if (j["input"].is_array()) {
        for (const auto& in_opts_js : j["input"]) {
            wmtk::components::input::InputOptions opts = in_opts_js;
            meshes.add_mesh(wmtk::components::input::input(opts, path_resolver));
        }
    } else {
        wmtk::components::input::InputOptions opts = j["input"];
        output_mesh = meshes.add_mesh(wmtk::components::input::input(opts, path_resolver))
                          .root()
                          .shared_from_this();
    }


    std::string position_attribute_name = "vertices";
    if (!j.contains("position_attribute_name")) {
        wmtk::logger().info("convert: No output mesh position_attribute_name provided");
    } else {
        position_attribute_name = j["position_attribute_name"];
    }
    std::string name = "combo";
    if (!j.contains("name")) {
        wmtk::logger().info("convert: No output mesh name provided");
    } else {
        name = j["name"];
    }
    std::string tag_format = "tag_{}";
    if (!j.contains("tag_format")) {
        wmtk::logger().info("convert: No output mesh name provided");
    } else {
        tag_format = j["tag_format"];
    }


    Params params{meshes, name, tag_format, position_attribute_name};


    for (const auto& align_file : j["aligns"]) {
        std::string str = align_file["path"];
        auto [path, worked] = path_resolver.resolve(str);
        assert(worked);

        params.alignments.emplace(read_vid_pairs(path));
    }

    wmtk::components::multimesh::NamedMultiMesh& nmm = run(params);


    if (!j.contains("output")) {
        wmtk::logger().info("convert: No output path provided");
    } else {
        auto output_opts = wmtk::components::output::parse_output(j["output"]);
        wmtk::components::output::output(meshes, output_opts);
    }


    if (j.contains("report")) {
        nlohmann::ordered_json jnew = j;
        const std::string report = j["report"];
        meshes.make_canonical();
        if (!report.empty()) {
            nlohmann::json out_json;
            auto& stats = out_json["stats"];
            stats = wmtk::applications::utils::element_count_report_named(meshes);
            jnew.erase("report");
            out_json["input"] = jnew;


            std::ofstream ofs(report);
            ofs << out_json;
        }
    }
    return 0;
}

int run(
    const std::string_view& app_name,
    const fs::path& config_path,
    const std::optional<fs::path>& name_spec_file,
    const std::optional<fs::path>& integration_test_config_file)
{
    nlohmann::json j;
    std::ifstream ifs(config_path);
    j = nlohmann::json::parse(ifs);

    return run_js(app_name, j, name_spec_file, integration_test_config_file);
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

    exit_mode = run(argv[0], json_input_file, name_spec_file, json_integration_config_file);


    assert(exit_mode != -1); // "Some subcommand should have updated the exit mode"
    return exit_mode;
}
