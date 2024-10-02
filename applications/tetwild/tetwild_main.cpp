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

    auto mesh = wmtk::components::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    auto mesh_after_simp = mesh;
    if (!j["skip_simplification"]) {
        auto [out, stats] = tetwild_simplification(
            static_cast<const TriMesh&>(*mesh),
            "vertices",
            j["target_edge_length"]);
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
    auto meshes_after_insertion = triangle_insertion(
        mesh_after_simp,
        "vertices",
        mesh_after_delaunay,
        "vertices",
        pass_through);

    assert(meshes_after_insertion[0].second == "main");

    auto main_mesh = meshes_after_insertion[0].first;
    // std::vector<std::pair<std::shared_ptr<wmtk::Mesh>, std::string>> envelope_meshes;

    // for (int64_t i = 1; i < meshes_after_insertion.size(); ++i) {
    //     envelope_meshes.push_back(meshes_after_insertion[i]);
    // }

    // assert(envelope_meshes.size() >= 2);

    std::vector<wmtk::components::EnvelopeOptions> enves;

    for (int64_t i = 1; i < meshes_after_insertion.size(); ++i) {
        wmtk::components::EnvelopeOptions e;
        e.envelope_name = meshes_after_insertion[i].second;
        e.envelope_constrained_mesh = meshes_after_insertion[i].first;
        e.envelope_geometry_mesh = meshes_after_insertion[i].first;
        e.constrained_position_name = "vertices";
        e.geometry_position_name = "vertices";
        e.thickness = j["envelope_size"];

        if (e.envelope_name == "surface") {
            e.envelope_geometry_mesh = mesh; // set as input
        }

        enves.push_back(e);
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
    main_mesh = meshes_after_tetwild[0].first;

    std::string output_file = j["output"];

    std::shared_ptr<Mesh> surface_mesh;
    for (int64_t i = 1; i < meshes_after_tetwild.size(); ++i) {
        // output child meshes
        wmtk::components::output(
            *(meshes_after_tetwild[i].first),
            output_file + "_" + meshes_after_tetwild[i].second,
            "vertices");

        if (meshes_after_tetwild[i].second == "surface") {
            surface_mesh = meshes_after_tetwild[i].first;
        }
    }


    auto mesh_after_winding_number = winding_number(main_mesh, surface_mesh);

    wmtk::components::output(*mesh_after_winding_number, output_file, "vertices");

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