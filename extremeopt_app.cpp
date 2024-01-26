
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/extreme_opt/extreme_opt.hpp>
#include <wmtk/components/extreme_opt/extreme_opt_single.hpp>
#include <wmtk/components/extreme_opt/internal/ExtremeOptOptions.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk::components::base;

using json = nlohmann::json;

const std::filesystem::path data_dir =
    std::filesystem::path(WMTK_DATA_DIR) / ".." / "extreme_opt_data_msh";

int main(int argc, char** argv)
{
    using path = std::filesystem::path;

    CLI::App app{argv[0]};
    path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")->required(true);


    CLI11_PARSE(app, argc, argv);
    if (!std::filesystem::exists(json_input_file)) {
        std::cout << "Json File does not exist." << std::endl;
        exit(-1);
    }

    wmtk::components::internal::ExtremeOptOptions options;
    json extremeopt_json;
    std::ifstream f(json_input_file);
    extremeopt_json = json::parse(f);
    options = extremeopt_json.get<wmtk::components::internal::ExtremeOptOptions>();

    std::string mesh_name = options.mesh_name;

    wmtk::io::Cache cache("wmtk_cache", ".");
    json seamed_json = {
        {"type", "input"},
        {"name", mesh_name + "_seamed"},
        {"file", data_dir.string() + "/" + mesh_name + "_pos.msh"},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), seamed_json, cache);
    json cut_json = {
        {"type", "input"},
        {"name", mesh_name + "_cut"},
        {"file", data_dir.string() + "/" + mesh_name + "_tex.msh"},
        {"ignore_z", true}};
    wmtk::components::input(Paths(), cut_json, cache);


    // wmtk::components::extreme_opt(Paths(), extremeopt_json, cache);
    wmtk::components::extreme_opt_single(Paths(), extremeopt_json, cache);
    return 0;
}
