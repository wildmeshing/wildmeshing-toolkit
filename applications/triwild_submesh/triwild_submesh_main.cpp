#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>

#include <wmtk/components/edge_insertion/edge_insertion.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/procedural/make_mesh.hpp>
#include <wmtk/components/wildmeshing/wildmeshing.hpp>
#include <wmtk/components/winding_number/winding_number.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "triwild_grid.hpp"
#include "triwild_submesh_spec.hpp"

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
        bool r = spec_engine.verify_json(j, triwild_submesh_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, triwild_submesh_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    if (!fs::exists(input_file)) {
        log_and_throw_error("File {} does not exist.", input_file.string());
    }

    auto mesh = wmtk::components::input::input(input_file, true);
    wmtk::logger().info(
        "mesh has {} vertices and {} edges",
        mesh->get_all(PrimitiveType::Vertex).size(),
        mesh->get_all(PrimitiveType::Edge).size());

    // get bbox;
    double x_min, y_min, x_max, y_max;
    x_min = std::numeric_limits<double>::max();
    y_min = std::numeric_limits<double>::max();
    x_max = std::numeric_limits<double>::lowest();
    y_max = std::numeric_limits<double>::lowest();

    auto mesh_pt_handle = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto mesh_pt_accessor = mesh->create_const_accessor<double>(mesh_pt_handle);

    for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
        x_min = std::min(x_min, mesh_pt_accessor.const_vector_attribute(v)[0]);
        x_max = std::max(x_max, mesh_pt_accessor.const_vector_attribute(v)[0]);
        y_min = std::min(y_min, mesh_pt_accessor.const_vector_attribute(v)[1]);
        y_max = std::max(y_max, mesh_pt_accessor.const_vector_attribute(v)[1]);
    }

    auto bg_mesh =
        wmtk::triwild::generate_bg_grid(x_min, y_min, x_max, y_max, j["target_edge_length"]);

    if (j["intermediate_output"]) {
        wmtk::components::output::output(bg_mesh, "bg_mesh", "vertices");
    }

    wmtk::logger().info("generated bg mesh");

    wmtk::components::EdgeInsertionMeshes eim =
        wmtk::components::edge_insertion(static_cast<EdgeMesh&>(*mesh), bg_mesh);

    wmtk::logger().info("finised edge insertion");

    auto trimesh = eim.tri_mesh;
    auto edgemesh = eim.inserted_edge_mesh;
    auto bboxmesh = eim.bbox_mesh;

    std::string output_file = j["output"];

    if (j["intermediate_output"]) {
        wmtk::components::output::output(*trimesh, output_file + "_after_insertion", "vertices");
        wmtk::components::output::output(
            *edgemesh,
            output_file + "_after_insertion_edge_mesh",
            "vertices");
    }

    // clean up
    {
        auto pos_handle =
            trimesh->get_attribute_handle<Rational>("vertices", PrimitiveType::Vertex);
        trimesh->clear_attributes({pos_handle});
    }

    std::vector<wmtk::components::EnvelopeOptions> enves;
    {
        wmtk::components::EnvelopeOptions e;
        e.envelope_name = "input";
        e.envelope_constrained_mesh = edgemesh;
        e.envelope_geometry_mesh = edgemesh;
        e.constrained_position_name = "vertices";
        e.geometry_position_name = "vertices";
        e.thickness = j["envelope_size"];

        // if (e.envelope_name == "input") {
        //    e.envelope_geometry_mesh = mesh; // set as input
        //}

        enves.push_back(e);

        wmtk::components::EnvelopeOptions e2;
        e2.envelope_name = "bbox";
        e2.envelope_constrained_mesh = bboxmesh;
        e2.envelope_geometry_mesh = bboxmesh;
        e2.constrained_position_name = "vertices";
        e2.geometry_position_name = "vertices";
        e2.thickness = 0.0001;

        enves.push_back(e2);
    }


    wmtk::components::WildMeshingOptions wmo;
    wmo.input_mesh = trimesh;
    wmo.input_mesh_position = "vertices";
    wmo.target_edge_length = j["target_edge_length"];
    wmo.target_max_amips = j["target_max_amips"];
    wmo.max_passes = j["max_passes"];
    wmo.replace_double_coordinate = false;
    wmo.scheduler_update_frequency = 0;
    wmo.intermediate_output = j["intermediate_output"];
    wmo.intermediate_output_path = "";
    wmo.intermediate_output_name = j["output"];
    wmo.envelopes = enves;
    // wmo.pass_through = pass_through;
    wmo.skip_split = j["skip_split"];
    wmo.skip_collapse = j["skip_collapse"];
    wmo.skip_swap = j["skip_swap"];
    wmo.skip_smooth = j["skip_smooth"];
    wmo.use_embedding = true;

    auto meshes_after_tetwild = wildmeshing(wmo);
    assert(meshes_after_tetwild.size() == 1);
    auto main_mesh = meshes_after_tetwild[0].first;

    wmtk::components::output::output(*main_mesh, output_file, "vertices");

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = main_mesh->get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = main_mesh->get_all(PrimitiveType::Edge).size();
        out_json["cells"] = main_mesh->get_all(PrimitiveType::Triangle).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << std::setw(4) << out_json;
    }

    return 0;
}