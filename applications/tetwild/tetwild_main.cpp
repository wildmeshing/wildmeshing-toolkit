#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>


#include <wmtk/components/delaunay/delaunay.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/tetwild_simplification/tetwild_simplification.hpp>
#include <wmtk/components/to_points/to_points.hpp>
#include <wmtk/components/triangle_insertion/triangle_insertion.hpp>
#include <wmtk/components/wildmeshing/wildmeshing.hpp>
#include <wmtk/components/winding_number/winding_number.hpp>


#include "tetwild_spec.hpp"

using namespace wmtk;
using namespace wmtk::components;
namespace fs = std::filesystem;

using wmtk::components::utils::resolve_paths;

int main(int argc, char* argv[])
{
    opt_logger().set_level(spdlog::level::off);
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
        bool r = spec_engine.verify_json(j, tetwild_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, tetwild_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    auto mesh = wmtk::components::input::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    auto mesh_after_simp = mesh;
    if (!j["skip_simplification"]) {
        auto [out, stats] = tetwild_simplification(
            static_cast<const TriMesh&>(*mesh),
            "vertices",
            j["envelope_size"]);
        mesh_after_simp = out;
    }

    wmtk::components::ToPtsOptions tpo;
    tpo.add_box = false;
    tpo.box_scale = 0.05;
    tpo.add_grid = true;
    tpo.grid_spacing = j["target_edge_length"];
    tpo.min_dist = 0;
    tpo.remove_duplicates = true;

    auto pts_attr_simp =
        mesh_after_simp->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto mesh_after_to_points = to_points(*mesh_after_simp, pts_attr_simp, tpo);

    auto pts_attr_tp =
        mesh_after_to_points->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto mesh_after_delaunay = delaunay(*mesh_after_to_points, pts_attr_tp);

    std::vector<attribute::MeshAttributeHandle> pass_through;

    auto [main_mesh, child_meshes] = wmtk::components::triangle_insertion::triangle_insertion(
        static_cast<const TetMesh&>(*mesh_after_delaunay),
        "vertices",
        static_cast<const TriMesh&>(*mesh_after_simp),
        "vertices",
        pass_through,
        true,
        true,
        false);

    std::string output_file = j["output"];

    wmtk::components::output::output(*main_mesh, output_file + "_after_insertion", "vertices");
    wmtk::components::output::output(
        *child_meshes.surface_mesh,
        output_file + "_surface_after_insertion",
        "vertices");

    std::vector<wmtk::components::EnvelopeOptions> enves;

    wmtk::components::EnvelopeOptions e_surface;
    e_surface.envelope_name = "surface";
    e_surface.envelope_constrained_mesh = child_meshes.surface_mesh;
    e_surface.envelope_geometry_mesh = mesh_after_simp;
    e_surface.constrained_position_name = "vertices";
    e_surface.geometry_position_name = "vertices";
    e_surface.thickness = j["envelope_size"];

    enves.push_back(e_surface);

    wmtk::components::EnvelopeOptions e_bbox;
    e_bbox.envelope_name = "bbox";
    e_bbox.envelope_constrained_mesh = child_meshes.bbox_mesh;
    e_bbox.envelope_geometry_mesh = child_meshes.bbox_mesh;
    e_bbox.constrained_position_name = "vertices";
    e_bbox.geometry_position_name = "vertices";
    e_bbox.thickness = j["envelope_size"];

    enves.push_back(e_bbox);

    if (child_meshes.open_boundary_mesh != nullptr) {
        wmtk::components::EnvelopeOptions e_open_boundary;
        e_open_boundary.envelope_name = "open_boundary";
        e_open_boundary.envelope_constrained_mesh = child_meshes.open_boundary_mesh;
        e_open_boundary.envelope_geometry_mesh = child_meshes.open_boundary_mesh;
        e_open_boundary.constrained_position_name = "vertices";
        e_open_boundary.geometry_position_name = "vertices";
        e_open_boundary.thickness = j["envelope_size"];

        enves.push_back(e_open_boundary);
    }

    if (child_meshes.nonmanifold_edge_mesh != nullptr) {
        wmtk::components::EnvelopeOptions e_nonmanifold_edge;
        e_nonmanifold_edge.envelope_name = "nonmanifold_edge";
        e_nonmanifold_edge.envelope_constrained_mesh = child_meshes.nonmanifold_edge_mesh;
        e_nonmanifold_edge.envelope_geometry_mesh = child_meshes.nonmanifold_edge_mesh;
        e_nonmanifold_edge.constrained_position_name = "vertices";
        e_nonmanifold_edge.geometry_position_name = "vertices";
        e_nonmanifold_edge.thickness = j["envelope_size"];

        enves.push_back(e_nonmanifold_edge);
    }


    wmtk::components::WildMeshingOptions wmo;
    wmo.input_mesh = main_mesh;
    wmo.input_mesh_position = "vertices";
    wmo.target_edge_length = j["target_edge_length"];
    wmo.target_max_amips = j["target_max_amips"];
    wmo.max_passes = j["max_passes"];
    wmo.intermediate_output = j["intermediate_output"];
    wmo.replace_double_coordinate = false;
    wmo.scheduler_update_frequency = 0;
    wmo.intermediate_output_path = "";
    wmo.intermediate_output_name = j["output"];
    wmo.envelopes = enves;
    wmo.pass_through = pass_through;

    auto meshes_after_tetwild = wildmeshing(wmo);
    auto main_mesh_after_tetwild = meshes_after_tetwild[0].first;

    std::shared_ptr<Mesh> surface_mesh;
    for (int64_t i = 1; i < meshes_after_tetwild.size(); ++i) {
        // output child meshes
        wmtk::components::output::output(
            *(meshes_after_tetwild[i].first),
            output_file + "_" + meshes_after_tetwild[i].second,
            "vertices");

        if (meshes_after_tetwild[i].second == "surface") {
            surface_mesh = meshes_after_tetwild[i].first;
        }
    }


    auto mesh_after_winding_number = winding_number(main_mesh_after_tetwild, surface_mesh);

    wmtk::components::output::output(*mesh_after_winding_number, output_file, "vertices");

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = mesh_after_winding_number->get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = mesh_after_winding_number->get_all(PrimitiveType::Edge).size();
        out_json["faces"] = mesh_after_winding_number->get_all(PrimitiveType::Triangle).size();
        out_json["cells"] = mesh_after_winding_number->get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << out_json;
    }


    return 0;
}