#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "marching_spec.hpp"

using namespace wmtk;
namespace fs = std::filesystem;

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

    const fs::path input_path_in = j["input_path"];

    const fs::path root = input_path_in.empty() ? json_input_file : input_path_in;

    fs::path input_file = wmtk::components::utils::resolve_path(j["input"], root);

    auto mesh = wmtk::components::input(input_file);

    // TODO ...

    // std::string output_file = j["output"];
    // wmtk::components::output(*out, output_file, j["output_pos_attr_name"]);

    // const std::string report = j["report"];
    // if (!report.empty()) {
    //     nlohmann::json out_json;
    //     out_json["vertices"] = out->get_all(PrimitiveType::Vertex).size();
    //     out_json["edges"] = out->get_all(PrimitiveType::Edge).size();
    //     out_json["faces"] = out->get_all(PrimitiveType::Triangle).size();
    //     out_json["cells"] = out->get_all(PrimitiveType::Tetrahedron).size();

    //     out_json["input"] = j;

    //     std::ofstream ofs(report);
    //     ofs << out_json;
    // }


    return 0;
}
