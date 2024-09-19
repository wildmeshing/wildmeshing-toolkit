
#include <jse/jse.h>
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/procedural/ProceduralOptions.hpp>
#include <wmtk/components/procedural/make_mesh.hpp>
#include "spec.hpp"

using namespace wmtk::components;
using namespace wmtk::applications;
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
        bool r = spec_engine.verify_json(j, applications::procedural::spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, applications::procedural::spec);
        }
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


    const std::string output_path = j["output"];
    if (coordinate_handle_opt.has_value()) {
        wmtk::components::output(*mesh, output_path, *coordinate_handle_opt);
    } else {
        assert(output_path.size() > 4);
        assert(output_path.substr(output_path.size() - 4) == ".hdf5");
        wmtk::components::output_hdf5(*mesh, j["output"]);
    }
}
