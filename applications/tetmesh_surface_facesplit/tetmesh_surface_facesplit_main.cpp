#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/utils/resolve_path.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/multimesh.hpp>
#include <wmtk/components/output/output.hpp>

#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/attribute_update/make_cast_attribute_transfer_strategy.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>

#include "tetmesh_surface_facesplit_spec.hpp"

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
        bool r = spec_engine.verify_json(j, tetmesh_surface_facesplit_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, tetmesh_surface_facesplit_spec);
        }
    }

    fs::path input_file = resolve_paths(json_input_file, {j["root"], j["input"]});

    auto mesh = wmtk::components::input::input(input_file);
    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    auto input_pos_handle = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto [tetmesh, surface_mesh] = wmtk::components::multimesh::multimesh(
        wmtk::components::multimesh::MultiMeshType::Boundary,
        *mesh,
        nullptr,
        input_pos_handle,
        "",
        -1,
        -1);
    wmtk::logger().info("registered boundary child mesh");

    const std::string output_file = j["output"];

    wmtk::components::output::output(
        *surface_mesh,
        output_file + "_surface_mesh_before_split",
        "vertices");

    // handles and updates
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    auto boundary_handle =
        tetmesh->get_attribute_handle<int64_t>("is_boundary", PrimitiveType::Triangle);

    auto surface_mesh_pos_handle =
        surface_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto tetmesh_pos_handle =
        tetmesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto visited_face_flag_handle =
        surface_mesh
            ->register_attribute<int64_t>("visited_face", PrimitiveType::Triangle, 1, false, 0);

    pass_through_attributes.push_back(boundary_handle);
    pass_through_attributes.push_back(visited_face_flag_handle);
    pass_through_attributes.push_back(surface_mesh_pos_handle);
    pass_through_attributes.push_back(tetmesh_pos_handle);

    // Facesplit
    auto visited_face_accessor = surface_mesh->create_accessor<int64_t>(visited_face_flag_handle);
    auto surface_pos_accessor = surface_mesh->create_accessor<double>(surface_mesh_pos_handle);

    wmtk::operations::composite::TriFaceSplit facesplit(*surface_mesh);

    for (const auto& attr : pass_through_attributes) {
        facesplit.split().set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        facesplit.collapse().set_new_attribute_strategy(
            attr,
            wmtk::operations::CollapseBasicStrategy::None);
    }

    const auto& original_surface_cnt = surface_mesh->get_all(PrimitiveType::Triangle).size();
    wmtk::logger().info("input mesh has {} surface faces", original_surface_cnt);

    int64_t success;
    int64_t total_success = 0;
    do {
        success = 0;
        auto all_face_tuples = surface_mesh->get_all(PrimitiveType::Triangle);
        auto all_face_simplices = wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
            *surface_mesh,
            all_face_tuples,
            PrimitiveType::Triangle);
        for (const auto& s : all_face_simplices) {
            if (!surface_mesh->is_valid(s)) {
                continue;
            }
            const auto& t = s.tuple();
            if (visited_face_accessor.scalar_attribute(s) != 0) {
                // new face, dont split
                continue;
            }

            const auto v0 = t;
            const auto v1 = surface_mesh->switch_tuple(t, PrimitiveType::Vertex);
            const auto v2 =
                surface_mesh->switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex});

            const Vector3d& p0 = surface_pos_accessor.const_vector_attribute(v0);
            const Vector3d& p1 = surface_pos_accessor.const_vector_attribute(v1);
            const Vector3d& p2 = surface_pos_accessor.const_vector_attribute(v2);

            // std::cout << "p0: " << p0 << std::endl;
            // std::cout << "p1: " << p1 << std::endl;
            // std::cout << "p2: " << p2 << std::endl;

            const auto old_neighbor_f = surface_mesh->switch_tuple(t, PrimitiveType::Triangle);
            const auto old_neigh_f_visited_flag =
                visited_face_accessor.scalar_attribute(old_neighbor_f);

            const auto new_v = facesplit(s).front().tuple();

            const Tuple new_f0 = new_v;
            const Tuple new_f1 = surface_mesh->switch_tuple(new_f0, PrimitiveType::Triangle);
            const Tuple new_f2 =
                surface_mesh->switch_tuples(new_f0, {PrimitiveType::Edge, PrimitiveType::Triangle});

            // assign new position
            surface_pos_accessor.vector_attribute(new_v) = (p0 + p1 + p2) / 3.;

            // std::cout << "new_v: " << surface_pos_accessor.vector_attribute(new_v) << std::endl;

            // assign visited flag
            visited_face_accessor.scalar_attribute(new_f0) = 1;
            visited_face_accessor.scalar_attribute(new_f1) = 1;
            visited_face_accessor.scalar_attribute(new_f2) = 1;

            // resurrect neighbor face visited tag
            const auto new_neighbor_f = surface_mesh->switch_tuples(
                new_v,
                {PrimitiveType::Vertex, PrimitiveType::Edge, PrimitiveType::Triangle});
            visited_face_accessor.scalar_attribute(new_neighbor_f) = old_neigh_f_visited_flag;
            // visited_face_accessor.scalar_attribute(new_neighbor_f) = 1;


            success++;

            int64_t remaining_cnt = 0;
            for (const auto& tt : surface_mesh->get_all(PrimitiveType::Triangle)) {
                if (visited_face_accessor.scalar_attribute(tt) == 0) {
                    remaining_cnt++;
                }
            }

            std::cout << "success: " << success << "  remaining: " << remaining_cnt
                      << "  current total cnt:  "
                      << surface_mesh->get_all(PrimitiveType::Triangle).size()
                      << "  old_flag: " << old_neigh_f_visited_flag << std::endl;
            // break;
        }

        // int64_t remaining_cnt = 0;
        // for (const auto& t : surface_mesh->get_all(PrimitiveType::Triangle)) {
        //     if (visited_face_accessor.scalar_attribute(t) == char(0)) {
        //         remaining_cnt++;
        //     }
        // }

        // std::cout << "success: " << success << "  remaining: " << remaining_cnt << std::endl;
        total_success += success;
        // break;
    } while (success > 0);

    wmtk::logger().info(
        "splitted {} faces, input mesh has {} surface faces",
        total_success,
        original_surface_cnt);

    multimesh::consolidate(*surface_mesh);

    // propagate positions
    auto propagate_to_parent_position =
        [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<double> {
        assert(P.cols() == 1);
        return P.col(0);
    };

    auto update_parent_position =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            tetmesh_pos_handle,
            surface_mesh_pos_handle,
            propagate_to_parent_position);

    update_parent_position->run_on_all();

    wmtk::components::output::output(*surface_mesh, output_file + "_surface_mesh", "vertices");
    wmtk::components::output::output(*tetmesh, output_file + "_tetmesh", "vertices");

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = tetmesh->get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = tetmesh->get_all(PrimitiveType::Edge).size();
        out_json["faces"] = tetmesh->get_all(PrimitiveType::Triangle).size();
        out_json["cells"] = tetmesh->get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << out_json;
    }

    return 0;
}
