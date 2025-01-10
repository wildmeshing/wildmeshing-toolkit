#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/utils/json_utils.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "tetwild_msh_converter_spec.hpp"

using namespace wmtk;
using namespace wmtk::components;
namespace fs = std::filesystem;

int main(int argc, char* argv[])
{
    opt_logger().set_level(spdlog::level::off);
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    fs::path input_file = "";
    fs::path output_file;

    auto cli_group = app.add_option_group("subgroup");

    CLI::Option* cli_json_spec_file_option =
        cli_group->add_option("-j, --json", json_input_file, "json specification file")
            ->check(CLI::ExistingFile);
    CLI::Option* cli_input_file_option =
        cli_group->add_option("-i, --input", input_file, "input MSH file")
            ->check(CLI::ExistingFile);

    cli_group->require_option(1); // --j xor --i

    CLI::Option* cli_output_file_option =
        app.add_option("-o, --output", output_file, "output VTU file")
            ->needs(cli_input_file_option);

    CLI11_PARSE(app, argc, argv);

    // json spec file
    nlohmann::json j;
    if (!json_input_file.empty()) {
        using wmtk::components::utils::resolve_paths;

        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);

        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(j, tetwild_msh_converter_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, tetwild_msh_converter_spec);
        }

        input_file = resolve_paths(json_input_file, {j["input_path"], j["input"]});
        output_file = std::string(j["output"]);
    }

    if (output_file.empty()) {
        output_file = input_file;
    }
    output_file.replace_extension();


    std::shared_ptr<Mesh> mesh;
    try {
        mesh = wmtk::components::input::input(input_file, false, {"in_out"});
    } catch (const std::exception&) {
        // try again but without in_out attribute
        mesh = wmtk::components::input::input(input_file);
    }
    Mesh& m = *mesh;
    wmtk::logger().info("mesh has {} vertices", m.get_all(PrimitiveType::Vertex).size());

    // add winding_number attribute
    if (m.has_attribute<double>("in_out", m.top_simplex_type())) {
        auto wn_handle = m.register_attribute<double>("winding_number", m.top_simplex_type(), 1);
        auto wn_acc = m.create_accessor<double>(wn_handle);

        auto in_out_handle = m.get_attribute_handle<double>("in_out", PrimitiveType::Tetrahedron);
        auto in_out_acc = m.create_accessor<double>(in_out_handle);

        for (const Tuple& t : m.get_all(m.top_simplex_type())) {
            wn_acc.scalar_attribute(t) = in_out_acc.scalar_attribute(t);
        }

        auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        m.clear_attributes({pos_handle, wn_handle});
    }

    // output
    {
        logger().info("Write mesh '{}'", output_file.string());

        const bool edge = m.top_simplex_type() == PrimitiveType::Edge;
        const bool tri = m.top_simplex_type() == PrimitiveType::Triangle;
        const bool tet = m.top_simplex_type() == PrimitiveType::Tetrahedron;

        wmtk::io::ParaviewWriter writer(output_file, "vertices", m, false, edge, tri, tet);
        m.serialize(writer);
    }

    if (!json_input_file.empty()) {
        const std::string report = j["report"];
        if (!report.empty()) {
            nlohmann::json out_json;
            out_json["stats"]["vertices"] = m.get_all(PrimitiveType::Vertex).size();
            out_json["stats"]["edges"] = m.get_all(PrimitiveType::Edge).size();
            out_json["stats"]["triangles"] = m.get_all(PrimitiveType::Triangle).size();
            out_json["stats"]["tets"] = m.get_all(PrimitiveType::Tetrahedron).size();

            out_json["input"] = j;

            std::ofstream ofs(report);
            ofs << std::setw(4) << out_json;
        }
    }


    return 0;
}