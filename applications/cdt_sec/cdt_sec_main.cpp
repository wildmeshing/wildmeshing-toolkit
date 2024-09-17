#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>


#include <wmtk/components/CDT/CDT.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/multimesh.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/shortestedge_collapse/shortestedge_collapse.hpp>


#include "cdt_sec_spec.hpp"

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
        bool r = spec_engine.verify_json(j, cdt_sec_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, cdt_sec_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    auto mesh = wmtk::components::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    auto mesh_after_cdt = wmtk::components::CDT(mesh, true, false);

    auto [parent_mesh, child_mesh] =
        wmtk::components::multimesh("boundary", mesh_after_cdt, nullptr, "vertices", "", -1, -1);
    wmtk::logger().info("registered boundary child mesh");

    std::vector<attribute::MeshAttributeHandle> pass_through;
    auto boundary_handle =
        parent_mesh->get_attribute_handle<int64_t>("is_boundary", PrimitiveType::Triangle);
    pass_through.push_back(boundary_handle);

    auto mesh_after_sec = wmtk::components::shortestedge_collapse(
        child_mesh,
        "vertices",
        parent_mesh,
        "vertices",
        false,
        j["length_rel"],
        false,
        j["envelope_size"],
        pass_through);

    std::string output_file = j["output"];
    wmtk::components::output(*parent_mesh, output_file, "vertices");
    wmtk::components::output(*mesh_after_sec, output_file + "_surface", "vertices");

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = parent_mesh->get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = parent_mesh->get_all(PrimitiveType::Edge).size();
        out_json["faces"] = parent_mesh->get_all(PrimitiveType::Triangle).size();
        out_json["cells"] = parent_mesh->get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << out_json;
    }


    return 0;
}