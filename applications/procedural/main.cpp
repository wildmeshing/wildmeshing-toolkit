
#include <jse/jse.h>
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/procedural/ProceduralOptions.hpp>
#include <wmtk/components/procedural/make_mesh.hpp>
#include "CLI/CLI.hpp"
#include "spec.hpp"

using namespace wmtk::components;
using namespace wmtk::applications;
using namespace wmtk;
namespace fs = std::filesystem;


int run(const fs::path& config_path)
{
    nlohmann::json j;
    {
        jse::JSE spec_engine;
        std::ifstream ifs(config_path);
        j = nlohmann::json::parse(ifs);

        bool r = spec_engine.verify_json(j, applications::procedural::spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        }
        j = spec_engine.inject_defaults(j, applications::procedural::spec);
    }


    wmtk::components::procedural::ProceduralOptions options =
        j.get<wmtk::components::procedural::ProceduralOptions>();


    auto [mesh, coordinate_handle_opt] = std::visit(
        [](auto&& op)
            -> std::
                tuple<std::shared_ptr<Mesh>, std::optional<wmtk::attribute::MeshAttributeHandle>>

        {
            auto m = wmtk::components::procedural::make_mesh(op);
            const auto coordinate_name_opt = op.get_coordinate_name();
            std::optional<wmtk::attribute::MeshAttributeHandle> h;
            if (coordinate_name_opt.has_value()) {
                h = m->template get_attribute_handle<double>(
                    coordinate_name_opt.value(),
                    wmtk::PrimitiveType::Vertex);
            }
            return std::make_tuple(std::move(m), h);
        },
        options.settings);


    using namespace internal;

    if (!bool(mesh)) {
        throw std::runtime_error("Did not obtain a mesh when generating a procedural one");
    }


    auto out_opts = j["output"].get<wmtk::components::output::OutputOptions>();
    if (coordinate_handle_opt.has_value()) {
        out_opts.position_attribute = *coordinate_handle_opt;
    }
    wmtk::components::output::output(*mesh, out_opts);
    return 0;
}

void fill_config(const fs::path& output_path, const auto& specific_options)
{
    wmtk::logger().info("Filling config for {}", specific_options.name());
    wmtk::components::procedural::ProceduralOptions options;
    options.settings = specific_options;
    nlohmann::json j;
    j.update(options);

    std::ofstream ofs(output_path);
    ofs << j;
}

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();
    app.require_subcommand(1, 2);

    std::optional<fs::path> json_input_file;
    wmtk::components::procedural::DiskOptions disk_options;
    wmtk::components::procedural::GridOptions grid_options;
    wmtk::components::procedural::TriangleFanOptions triangle_fan_options;
    std::optional<fs::path> fill_config_path;

    CLI::App* run_cmd = app.add_subcommand("run", "Run application");
    run_cmd->add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);

    CLI::App* config_cmd = app.add_subcommand("config", "Config creation options");
    config_cmd->add_option("-o, --output", fill_config_path, "Output config")->required(true);
    //->check(CLI::ExistingFile);

    CLI::App* grid_cmd =
        config_cmd->add_subcommand("grid", "Grid config options")->fallthrough(true);


    CLI::App* triangle_fan_cmd =
        config_cmd->add_subcommand("triangle_fan", "Grid config options")->fallthrough(true);
    CLI::App* disk_cmd = config_cmd->add_subcommand("disk", "Disk mesh options")->fallthrough(true);
    CLI11_PARSE(app, argc, argv);

    // if (!json_input_file.has_value() && !fill_config_path.has_value()) {
    //     wmtk::logger().error("An input json file with [-j] is required unless blank config "
    //                          "generation is being used with [--fill-config]");
    //     return 1;
    // }

    int exit_mode = -1;

    run_cmd->callback([&]() {
        spdlog::warn("YOW!");
        assert(json_input_file.has_value());
        exit_mode = run(json_input_file.value());
    });

    disk_cmd->callback([&]() {
        spdlog::warn("YOW!");
        assert(fill_config_path.has_value());
        fill_config(fill_config_path.value(), disk_options);
        exit_mode = 0;
    });
    triangle_fan_cmd->callback([&]() {
        spdlog::warn("YOW!");
        assert(fill_config_path.has_value());
        fill_config(fill_config_path.value(), triangle_fan_options);
        exit_mode = 0;
    });
    grid_cmd->callback([&]() {
        spdlog::warn("YOW!");
        assert(fill_config_path.has_value());
        fill_config(fill_config_path.value(), grid_options);
        exit_mode = 0;
    });

    config_cmd->callback([&]() {
        spdlog::warn("YOW!");
        assert(fill_config_path.has_value());
        // fill_config(fill_config_path.value(), options);
        exit_mode = 0;
    });

    spdlog::info(
        "{} {} {} {}",
        config_cmd->parsed(),
        disk_cmd->parsed(),
        triangle_fan_cmd->parsed(),
        grid_cmd->parsed());
    ;

    assert(exit_mode != -1); // "Some subcommand should have updated the exit mode"
    return exit_mode;
}
