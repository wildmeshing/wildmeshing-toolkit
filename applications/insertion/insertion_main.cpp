#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>


#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/triangle_insertion/triangle_insertion.hpp>

#include "insertion_spec.hpp"

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

    fs::path tri_file = resolve_paths(json_input_file, {j["root"], j["tri_mesh"]});
    fs::path tet_file = resolve_paths(json_input_file, {j["root"], j["bg_mesh"]});

    auto tri_mesh = wmtk::components::input::input(tri_file);
    auto tet_mesh = wmtk::components::input::input(tet_file);

    if (tri_mesh->top_simplex_type() != PrimitiveType::Triangle)
        log_and_throw_error("triangle_insertion supports only triangle meshes");
    if (tet_mesh->top_simplex_type() != PrimitiveType::Tetrahedron)
        log_and_throw_error("triangle_insertion supports only bg tet meshes");


    wmtk::logger().info(
        "tri mesh has {} vertices",
        tri_mesh->get_all(PrimitiveType::Vertex).size());
    wmtk::logger().info(
        "tet mesh has {} vertices",
        tet_mesh->get_all(PrimitiveType::Vertex).size());


    const std::string bg_post = j["bg_pos_attr_name"];
    const std::string tri_pos = j["tri_pos_attr_name"];
    auto [out, _] = wmtk::components::triangle_insertion::triangle_insertion(
        static_cast<const TetMesh&>(*tet_mesh),
        bg_post,
        static_cast<const TriMesh&>(*tri_mesh),
        tri_pos,
        j["round"],
        j["track_submeshes"],
        j["make_child_free"]);

    std::string output_file = j["output"];
    wmtk::components::output::output(*out, output_file, j["bg_pos_attr_name"]);

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = out->get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = out->get_all(PrimitiveType::Edge).size();
        out_json["faces"] = out->get_all(PrimitiveType::Triangle).size();
        out_json["cells"] = out->get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << out_json;
    }


    return 0;
}
