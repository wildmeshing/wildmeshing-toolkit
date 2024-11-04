#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>


#include <wmtk/components/CDT/CDT.hpp>
#include <wmtk/components/cdt_optimization/cdt_optimization.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/multimesh.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/shortest_edge_collapse/shortest_edge_collapse.hpp>
#include <wmtk/components/wildmeshing/wildmeshing.hpp>


#include "cdt_opt_spec.hpp"

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
        bool r = spec_engine.verify_json(j, cdt_opt_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, cdt_opt_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    auto mesh = wmtk::components::input::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    auto mesh_after_cdt = wmtk::components::CDT(static_cast<const TriMesh&>(*mesh), true, true);

    attribute::MeshAttributeHandle mesh_after_cdt_position_handle;

    if (mesh_after_cdt->has_attribute<Rational>("vertices", PrimitiveType::Vertex)) {
        mesh_after_cdt_position_handle =
            mesh_after_cdt->get_attribute_handle<Rational>("vertices", PrimitiveType::Vertex);
    } else {
        mesh_after_cdt_position_handle =
            mesh_after_cdt->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    }

    auto [parent_mesh, child_mesh] = wmtk::components::multimesh::multimesh(
        wmtk::components::multimesh::MultiMeshType::Boundary,
        *mesh_after_cdt,
        nullptr,
        mesh_after_cdt_position_handle,
        "",
        -1,
        -1);
    wmtk::logger().info("registered boundary child mesh");

    // check open
    for (const auto& e : child_mesh->get_all(PrimitiveType::Edge)) {
        if (child_mesh->is_boundary(PrimitiveType::Edge, e)) {
            wmtk::logger().info("open edge on trimesh!");
        }
    }

    std::string output_file = j["output"];
    wmtk::components::output::output(*parent_mesh, output_file + "_before_sec", "vertices");
    wmtk::components::output::output(*child_mesh, output_file + "_surface_before_sec", "vertices");

    std::vector<attribute::MeshAttributeHandle> pass_through;
    auto boundary_handle =
        parent_mesh->get_attribute_handle<int64_t>("is_boundary", PrimitiveType::Triangle);
    pass_through.push_back(boundary_handle);

    // auto child_mesh_position_handle =
    //     child_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // attribute::MeshAttributeHandle parent_mesh_position_handle =
    //     parent_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // {
    //     using namespace components::shortest_edge_collapse;

    //     ShortestEdgeCollapseOptions options;
    //     options.position_handle = child_mesh_position_handle;
    //     options.other_position_handles.emplace_back(parent_mesh_position_handle);
    //     options.length_rel = j["length_rel"];
    //     options.envelope_size = j["envelope_size"];
    //     options.check_inversions = true;
    //     options.pass_through_attributes = pass_through;

    //     shortest_edge_collapse(*parent_mesh, options);
    // }

    std::vector<wmtk::components::EnvelopeOptions> enves;

    wmtk::components::EnvelopeOptions e_surface;
    e_surface.envelope_name = "surface";
    e_surface.envelope_constrained_mesh = child_mesh;
    e_surface.envelope_geometry_mesh = child_mesh;
    e_surface.constrained_position_name = "vertices";
    e_surface.geometry_position_name = "vertices";
    e_surface.thickness = j["envelope_size"];

    enves.push_back(e_surface);

    wmtk::components::WildMeshingOptions wmo;
    wmo.input_mesh = parent_mesh;
    wmo.input_mesh_position = "vertices";
    wmo.target_edge_length = j["length_rel"];
    wmo.target_max_amips = 50;
    wmo.max_passes = 10;
    wmo.intermediate_output = false;
    wmo.replace_double_coordinate = false;
    wmo.scheduler_update_frequency = 0;
    wmo.intermediate_output_path = "";
    wmo.intermediate_output_name = j["output"];
    wmo.envelopes = enves;
    wmo.pass_through = pass_through;
    wmo.skip_split = false;
    wmo.skip_collapse = false;
    wmo.skip_swap = false;
    wmo.skip_smooth = true;

    auto meshes_after_wildmeshing = wildmeshing(wmo);

    wmtk::components::output::output(*meshes_after_wildmeshing[0].first, output_file, "vertices");
    wmtk::components::output::output(
        *meshes_after_wildmeshing[1].first,
        output_file + "_surface",
        "vertices");

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
