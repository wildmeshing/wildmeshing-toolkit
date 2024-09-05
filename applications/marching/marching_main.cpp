#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/marching/internal/Marching.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
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

    std::shared_ptr<Mesh> mesh_in = wmtk::components::input(input_file);
    Mesh& mesh = *mesh_in;


    auto pos_handle = mesh.get_attribute_handle<double>(j["pos_attr_name"], PrimitiveType::Vertex);

    // marching
    {
        std::map<PrimitiveType, attribute::MeshAttributeHandle> label_handles;
        label_handles[PrimitiveType::Vertex] =
            mesh.get_attribute_handle<int64_t>(j["vertex_label"], PrimitiveType::Vertex);
        label_handles[PrimitiveType::Edge] =
            mesh.get_attribute_handle<int64_t>(j["edge_label"], PrimitiveType::Edge);
        label_handles[PrimitiveType::Triangle] =
            mesh.get_attribute_handle<int64_t>(j["face_label"], PrimitiveType::Triangle);

        const auto pass_through = wmtk::components::utils::get_attributes(mesh, j["pass_through"]);

        const std::vector<int64_t> input_values = j["input_values"];
        const int64_t output_value = j["output_value"];

        wmtk::components::Marching mc(pos_handle, label_handles, input_values, output_value);

        mc.add_pass_through(pass_through);

        mc.process();
    }

    wmtk::components::output(mesh, j["output"], pos_handle);

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = mesh.get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = mesh.get_all(PrimitiveType::Edge).size();
        out_json["triangles"] = mesh.get_all(PrimitiveType::Triangle).size();
        out_json["tets"] = mesh.get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << std::setw(4) << out_json;
    }


    return 0;
}
