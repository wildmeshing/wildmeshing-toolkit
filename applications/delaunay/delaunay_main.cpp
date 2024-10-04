#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <wmtk/applications/utils/element_count_report.hpp>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>


#include <wmtk/components/delaunay/delaunay.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/to_points/to_points.hpp>

#include "delaunay_spec.hpp"

using namespace wmtk;
namespace fs = std::filesystem;

using wmtk::components::utils::resolve_paths;

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);
    CLI11_PARSE(app, argc, argv);

    nlohmann::json j;
    {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);

        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(j, delaunay_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, delaunay_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    auto mesh = wmtk::components::input::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    wmtk::components::ToPtsOptions options;
    options.add_box = j["add_box"];
    options.box_scale = j["box_scale"];
    options.add_grid = j["add_grid"];
    options.grid_spacing = j["grid_spacing"];
    options.min_dist = j["min_dist"];
    options.remove_duplicates = j["remove_duplicates"];

    auto in_pts_attr = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pts_mesh =
        wmtk::components::to_points(*mesh, in_pts_attr, options, j["output_pos_attr_name"]);

    auto pts_pts_attr =
        pts_mesh->get_attribute_handle<double>(j["output_pos_attr_name"], PrimitiveType::Vertex);
    auto out = wmtk::components::delaunay(*pts_mesh, pts_pts_attr, j["output_pos_attr_name"]);

    std::string output_file = j["output"];
    wmtk::components::output::output(*out, output_file, j["output_pos_attr_name"]);

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json.update(wmtk::applications::utils::element_count_report_named(*out));
        out_json["input"] = j;

        spdlog::warn("{}", out_json.dump(2));

        std::ofstream ofs(report);
        ofs << out_json;
    }


    return 0;
}
