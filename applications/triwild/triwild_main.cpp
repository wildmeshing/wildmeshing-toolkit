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

#include "triwild_spec.hpp"

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
        bool r = spec_engine.verify_json(j, triwild_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, triwild_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    auto mesh = wmtk::components::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    // TODO: use procedural

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

    const double diag = (Eigen::Vector2d(x_min, y_min) - Eigen::Vector2d(x_max, y_max)).norm();
    const double eps = 0.2; // TODO: change with length_rel
    x_min -= diag * eps;
    y_min -= diag * eps;
    x_max += diag * eps;
    y_max += diag * eps;

    MatrixX<Rational> V;
    V.resize(4, 2);
    V.row(0) = Vector2r(x_min, y_min);
    V.row(1) = Vector2r(x_max, y_min);
    V.row(2) = Vector2r(x_max, y_max);
    V.row(3) = Vector2r(x_min, y_max);

    RowVectors3l F;
    F.resize(2, 3);
    F.row(0) << 0, 1, 2;
    F.row(1) << 0, 2, 3;

    wmtk::TriMesh bg_mesh;
    bg_mesh.initialize(F, false);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, bg_mesh);

    // auto em = static_cast<const wmtk::EdgeMesh&>(*mesh);

    wmtk::components::EdgeInsertionMeshes eim =
        wmtk::components::edge_insertion(static_cast<EdgeMesh&>(*mesh), bg_mesh);

    std::vector<attribute::MeshAttributeHandle> pass_through;
    auto trimesh = eim.tri_mesh;
    auto edgemesh = eim.inserted_edge_mesh;

    auto input_handle = trimesh->get_attribute_handle<int64_t>("input", PrimitiveType::Edge);
    pass_through.push_back(input_handle);

    // TODO: add open vertex boundary
    std::vector<wmtk::components::EnvelopeOptions> enves;

    wmtk::components::EnvelopeOptions e;
    e.envelope_name = "input";
    e.envelope_constrained_mesh = edgemesh;
    e.envelope_geometry_mesh = edgemesh;
    e.constrained_position_name = "vertices";
    e.geometry_position_name = "vertices";
    e.thickness = j["envelope_size"];

    if (e.envelope_name == "input") {
        e.envelope_geometry_mesh = mesh; // set as input
    }

    enves.push_back(e);


    wmtk::components::WildMeshingOptions wmo;
    wmo.input_mesh = trimesh;
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
    auto main_mesh = meshes_after_tetwild[0].first;

    std::string output_file = j["output"];

    std::shared_ptr<Mesh> input_mesh;
    for (int64_t i = 1; i < meshes_after_tetwild.size(); ++i) {
        // output child meshes
        wmtk::components::output(
            *(meshes_after_tetwild[i].first),
            output_file + "_" + meshes_after_tetwild[i].second,
            "vertices");

        if (meshes_after_tetwild[i].second == "input") {
            input_mesh = meshes_after_tetwild[i].first;
        }
    }


    auto mesh_after_winding_number = winding_number(main_mesh, input_mesh);

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