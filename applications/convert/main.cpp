
#include <jse/jse.h>
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/element_count_report.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/input.hpp>

#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "CLI/CLI.hpp"
#include "wmtk/components/multimesh/MeshCollection.hpp"
#include "wmtk/components/multimesh/from_boundary.hpp"
#include "wmtk/components/multimesh/from_facet_bijection.hpp"

using namespace wmtk::components;
using namespace wmtk::applications;
using namespace wmtk;
namespace fs = std::filesystem;

namespace {
void merge_meshes(wmtk::components::multimesh::MeshCollection& mc, const nlohmann::json& js)
{
    for (const auto& [parent, child_datas] : js.items()) {
        auto& parent_mesh = mc.get_mesh(parent);
        if (child_datas.is_string()) {
            auto& child_mesh = mc.get_mesh(child_datas.get<std::string>());
            components::multimesh::from_facet_bijection(parent_mesh, child_mesh);

        } else if (child_datas.is_object()) {
            const std::string type = child_datas["type"];

            if (type == "facet_bijection") {
                const std::string child_name = child_datas["name"];
                auto& child_mesh = mc.get_mesh(child_name);
                components::multimesh::from_facet_bijection(parent_mesh, child_mesh);

            } else if (type == "boundary") {
                const int64_t dimension = child_datas["dimension"];
                std::string boundary_attr_name = fmt::format("boundary_{}", dimension);
                if (child_datas.contains("boundary_attribute_name")) {
                    boundary_attr_name = child_datas["boundary_attribute_name"];
                }
                components::multimesh::from_boundary(
                    parent_mesh,
                    wmtk::get_primitive_type_from_id(dimension),
                    boundary_attr_name);
            }
        }
    }
}
} // namespace

int run(const fs::path& config_path /*, const std::optional<fs::path>& name_spec_file*/)
{
    nlohmann::json j;
    {
        std::ifstream ifs(config_path);
        j = nlohmann::json::parse(ifs);
        // if (name_spec_file.has_value()) {
        //     j["name"] = nlohmann::json::parse(std::ifstream(name_spec_file.value()));
        // }
    }

    wmtk::components::multimesh::MeshCollection meshes;
    if (j["input"].is_array()) {
        for (const auto& in_opts_js : j["input"]) {
            wmtk::components::input::InputOptions opts = in_opts_js;
            meshes.add_mesh(wmtk::components::input::input(opts));
        }
        merge_meshes(meshes, j["tree"]);

    } else {
        wmtk::components::input::InputOptions opts = j["input"];
        meshes.add_mesh(wmtk::components::input::input(opts));
    }


    if (!j.contains("output")) {
        wmtk::logger().info("convert: No output path provided");
    } else {
    }


    // if (j.contains("report")) {
    //     const std::string report = j["report"];
    //     if (!report.empty()) {
    //         nlohmann::json out_json;
    //         out_json.update(wmtk::applications::utils::element_count_report_named(*mesh));
    //         j.erase("report");
    //         out_json["input"] = j;


    //        std::ofstream ofs(report);
    //        ofs << out_json;
    //    }
    //}
    return 0;
}


int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    std::optional<fs::path> name_spec_file;

    CLI::App* run_cmd; // = app.add_subcommand("run", "Run application");
    run_cmd = &app;
    run_cmd->add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);

    // run_cmd->add_option("-n, --name_spec", name_spec_file, "json specification file")
    //     ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    // someday may add other suboptions
    assert(run_cmd->parsed());

    // if (!json_input_file.has_value() && !fill_config_path.has_value()) {
    //     wmtk::logger().error("An input json file with [-j] is required unless blank config "
    //                          "generation is being used with [--fill-config]");
    //     return 1;
    // }

    int exit_mode = -1;

    // run_cmd->callback([&]() {
    //     spdlog::warn("YOW!");
    //     assert(json_input_file.has_value());
    //     exit_mode = run(json_input_file.value());
    // });
    exit_mode = run(json_input_file /*, name_spec_file*/);


    assert(exit_mode != -1); // "Some subcommand should have updated the exit mode"
    return exit_mode;
}
