
#include <jse/jse.h>
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
#include <wmtk/components/utils/resolve_path.hpp>

#include "CLI/CLI.hpp"
#include "wmtk/components/multimesh/MeshCollection.hpp"
#include "wmtk/components/multimesh/axis_aligned_periodic.hpp"
#include "wmtk/components/multimesh/from_boundary.hpp"
#include "wmtk/components/multimesh/from_facet_bijection.hpp"
#include "wmtk/components/multimesh/utils/get_attribute.hpp"
#include "wmtk/components/utils/PathResolver.hpp"

using namespace wmtk::components;
using namespace wmtk::applications;
using namespace wmtk;
namespace fs = std::filesystem;

namespace {
std::shared_ptr<wmtk::Mesh> merge_meshes(
    wmtk::components::multimesh::MeshCollection& mc,
    const nlohmann::json& js)
{
    for (const auto& [parent, child_datas] : js.items()) {
        auto& parent_mesh = mc.get_mesh(parent);
        auto& parent_named_mesh = mc.get_named_multimesh(parent);
        if (child_datas.is_string()) {
            const auto child_name = child_datas.get<std::string>();
            auto& child_mesh = mc.get_mesh(child_name);
            auto& child_named_mesh = mc.get_named_multimesh(child_name);
            components::multimesh::from_facet_bijection(parent_mesh, child_mesh);
            parent_named_mesh.append_child_mesh_names(parent_mesh, child_named_mesh);
            return parent_mesh.shared_from_this();

        } else if (child_datas.is_object()) {
            const std::string type = child_datas["type"];

            if (type == "facet_bijection") {
                const std::string child_name = child_datas["name"];
                auto& child_mesh = mc.get_mesh(child_name);
                auto& child_named_mesh = mc.get_named_multimesh(child_name);
                components::multimesh::from_facet_bijection(parent_mesh, child_mesh);
                parent_named_mesh.append_child_mesh_names(parent_mesh, child_named_mesh);
                return parent_mesh.shared_from_this();

            } else if (type == "boundary") {
                const int64_t dimension = child_datas["dimension"];
                std::string boundary_attr_name = fmt::format("boundary_{}", dimension);
                if (child_datas.contains("boundary_attribute_name")) {
                    boundary_attr_name = child_datas["boundary_attribute_name"];
                }

                std::string boundary_mesh_name = fmt::format("boundary_{}", dimension);
                if (child_datas.contains("boundary_mesh_name")) {
                    boundary_mesh_name = child_datas["boundary_mesh_name"];
                }
                auto mptr = components::multimesh::from_boundary(
                    parent_mesh,
                    wmtk::get_primitive_type_from_id(dimension),
                    boundary_attr_name);

                auto& child_named_mesh = mc.emplace_mesh(*mptr, boundary_mesh_name);
                parent_named_mesh.append_child_mesh_names(parent_mesh, child_named_mesh);

                return parent_mesh.shared_from_this();
            }
        } else if (type == "axis_aligned_periodic") {
            std::string position_attr_name = child_datas["position_attribute"];
            auto mah = wmtk::components::multimesh::utils::get_attribute(position_attr_name);

            std::vector<bool> mask = child_datas["axes"];
            std::string output_mesh_name = child_datas["fused_mesh_name"];
            auto mptr = components::multimesh::axis_aligned_fusion(mah, boundary_attr_name);

            nlohmann::json js;
            js[output_mesh_name] = nmm.get_names_json();
            mc.add_mesh(*mptr, js;

            return mptr;
        }
    }
}
return nullptr;
}
} // namespace

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

    if (j.contains("tree")) {
        output_mesh = merge_meshes(meshes, j["tree"]);
    }

    if (!j.contains("output")) {
        wmtk::logger().info("convert: No output path provided");
    } else {
        std::map<std::string, wmtk::components::output::OutputOptions> output_opts = j["output"];
        wmtk::components::output::output(meshes, output_opts);
    }


    if (j.contains("report")) {
        nlohmann::json jnew = j;
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

    CLI::App* json_cmd = app.add_subcommand("json", "Run application using a json");
    json_cmd->add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);

    add_it_path(*json_cmd);


    fs::path input;
    fs::path output;
    std::string type;
    CLI::App* run_cmd = app.add_subcommand("run", "Convert mesh to another type");
    run_cmd->add_option("-i, --input", input, "input file")
        ->required(true)
        ->check(CLI::ExistingFile);

    run_cmd->add_option("-o, --output", output, "output file");
    run_cmd->add_option("-t, --type", type, "output file type, knows [vtu,hdf5]");
    add_it_path(*run_cmd);


    // json_cmd->add_option("-n, --name_spec", name_spec_file, "json specification file")
    //     ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    // someday may add other suboptions

    // if (!json_input_file.has_value() && !fill_config_path.has_value()) {
    //     wmtk::logger().error("An input json file with [-j] is required unless blank config "
    //                          "generation is being used with [--fill-config]");
    //     return 1;
    // }

    int exit_mode = -1;

    // json_cmd->callback([&]() {
    //     spdlog::warn("YOW!");
    //     assert(json_input_file.has_value());
    //     exit_mode = run(json_input_file.value());
    // });
    if (json_cmd->parsed()) {
        exit_mode = run(argv[0], json_input_file, name_spec_file, json_integration_config_file);
    } else {
        wmtk::components::input::InputOptions in;
        in.path = input;
        wmtk::components::output::OutputOptions out;
        out.path = output;
        out.position_attribute = "vertices";
        if (!type.empty() && type[0] != '.') {
            type = '.' + type;
        }
        out.type = type;

        nlohmann::json js;
        js["input"] = in;
        js["output"][""] = out;

        exit_mode = run_js(argv[0], js, name_spec_file, json_integration_config_file);
    }


    assert(exit_mode != -1); // "Some subcommand should have updated the exit mode"
    return exit_mode;
}
