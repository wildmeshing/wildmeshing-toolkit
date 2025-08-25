#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/multimesh.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/shortest_edge_collapse/shortest_edge_collapse.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "shortest_edge_collapse_spec.hpp"
#ifdef WMTK_RECORD_OPERATIONS
#include <wmtk/Record_Operations.hpp>
#endif
using namespace wmtk;
namespace fs = std::filesystem;


using wmtk::components::utils::resolve_paths;

namespace {

enum class MultiMeshOptions { None, OptBoundary, OptInterior };

NLOHMANN_JSON_SERIALIZE_ENUM(
    MultiMeshOptions,
    {{MultiMeshOptions::None, "none"},
     {MultiMeshOptions::OptInterior, "interior"},
     {MultiMeshOptions::OptBoundary, "boundary"}});

} // namespace

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
        bool r = spec_engine.verify_json(j, shortest_edge_collapse_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, shortest_edge_collapse_spec);
        }
    }
#ifdef WMTK_RECORD_OPERATIONS
    OperationLogPath = generatePathNameWithCurrentTime();
    initializeBatchLogging();
#endif
    const fs::path input_file = resolve_paths(json_input_file, {j["input_path"], j["input"]});

    std::shared_ptr<Mesh> mesh_in = wmtk::components::input::input(input_file, true);

    attribute::MeshAttributeHandle pos_handle =
        mesh_in->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    attribute::MeshAttributeHandle other_pos_handle;

    // create multi-mesh
    std::shared_ptr<Mesh> current_mesh = mesh_in;
    std::shared_ptr<Mesh> other_mesh;
    MultiMeshOptions mm_opt = j["use_multimesh"];

    if (mm_opt != MultiMeshOptions::None) {
        auto [parent_mesh, child_mesh] = wmtk::components::multimesh::multimesh(
            wmtk::components::multimesh::MultiMeshType::Boundary,
            *mesh_in,
            nullptr,
            pos_handle,
            "",
            -1,
            -1);
        parent_mesh->clear_attributes({pos_handle});

        if (mm_opt == MultiMeshOptions::OptBoundary) {
            current_mesh = child_mesh;
            other_mesh = parent_mesh;
        } else {
            current_mesh = parent_mesh;
            other_mesh = child_mesh;
        }
        pos_handle = current_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        other_pos_handle =
            other_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    }

    Mesh& mesh = *mesh_in;

    // shortest-edge collapse
    {
        using namespace components::shortest_edge_collapse;
        ShortestEdgeCollapseOptions options;
        options.position_handle = pos_handle;
        if (other_mesh) {
            options.other_position_handles.emplace_back(other_pos_handle);
        }

        options.length_rel = j["length_rel"];
        const double env_size = j["envelope_size"];
        if (env_size >= 0) {
            options.envelope_size = j["envelope_size"];
        }
        options.lock_boundary = j["lock_boundary"];
        options.check_inversions = j["check_inversion"];

        shortest_edge_collapse(mesh, options);
    }

    wmtk::components::output::output(mesh, j["output"], pos_handle);

    // output child meshes
    {
        const std::string output_name = j["output"];
        const auto children = mesh.get_all_child_meshes();
        for (size_t i = 0; i < children.size(); ++i) {
            Mesh& child = *children[i];
            if (!child.has_attribute<double>("vertices", PrimitiveType::Vertex)) {
                logger().warn("Child has no vertices attribute");
                continue;
            }
            auto ph = child.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
            wmtk::components::output::output(child, fmt::format("{}_child_{}", output_name, i), ph);
        }
    }


    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["stats"]["vertices"] = mesh.get_all(PrimitiveType::Vertex).size();
        out_json["stats"]["edges"] = mesh.get_all(PrimitiveType::Edge).size();
        out_json["stats"]["triangles"] = mesh.get_all(PrimitiveType::Triangle).size();
        out_json["stats"]["tets"] = mesh.get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << std::setw(4) << out_json;
    }

#ifdef WMTK_RECORD_OPERATIONS
    finalizeBatchLogging();
#endif

    return 0;
}
