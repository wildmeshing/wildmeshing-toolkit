#include "edge_insertion.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/EdgeInsOptions.hpp"
#include "internal/edge_insertion.hpp"

namespace wmtk::components {

using namespace internal;

void edge_insertion(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    EdgeInsOptions options = j.get<EdgeInsOptions>();

    std::shared_ptr<Mesh> tris = cache.read_mesh(options.triangles);
    std::shared_ptr<Mesh> edges = cache.read_mesh(options.edges);

    TriMesh& trimesh = static_cast<TriMesh&>(*tris);
    EdgeMesh& edgemesh = static_cast<EdgeMesh&>(*edges);

    std::vector<Vector2r> v_final;
    std::vector<std::array<int64_t, 3>> FV_new;
    std::vector<std::array<int, 3>> local_e_on_input;

    wmtk::logger().info("start edge insertion ...");

    edge_insertion(trimesh, edgemesh, v_final, FV_new, local_e_on_input);

    wmtk::logger().info("finished edge insertion");

    wmtk::logger().info("creating multimesh ...");

    MatrixX<Rational> V;
    MatrixX<int64_t> FV;

    V.resize(v_final.size(), 2);
    FV.resize(FV_new.size(), 3);

    for (int64_t i = 0; i < v_final.size(); ++i) {
        V.row(i) = v_final[i];
    }

    for (int64_t i = 0; i < FV_new.size(); ++i) {
        FV(i, 0) = FV_new[i][0];
        FV(i, 1) = FV_new[i][1];
        FV(i, 2) = FV_new[i][2];
    }

    std::shared_ptr<wmtk::TriMesh> m = std::make_shared<wmtk::TriMesh>();
    m->initialize(FV);
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *m);

    auto input_handle = m->register_attribute<int64_t>("input", PrimitiveType::Edge, 1);
    auto input_accessor = m->create_accessor<int64_t>(input_handle);

    const auto& triangles = m->get_all(PrimitiveType::Triangle);

    for (int64_t i = 0; i < triangles.size(); ++i) {
        const auto& e01 = triangles[i];
        const auto& e02 = m->switch_tuple(e01, PrimitiveType::Edge);
        const auto& e12 = m->switch_tuples(e01, {PrimitiveType::Vertex, PrimitiveType::Edge});

        input_accessor.scalar_attribute(e01) = local_e_on_input[i][2];
        input_accessor.scalar_attribute(e02) = local_e_on_input[i][1];
        input_accessor.scalar_attribute(e12) = local_e_on_input[i][0];
    }

    std::shared_ptr<Mesh> input_mesh;

    internal::MultiMeshFromTag mmft(*m, input_handle, true);
    mmft.compute_substructure_mesh();

    input_mesh = m->get_child_meshes().back();

    mmft.remove_soup();

    auto pt_attribute = m->get_attribute_handle<Rational>("vertices", PrimitiveType::Vertex);

    for (auto child : m->get_child_meshes()) {
        auto child_position_handle = child->register_attribute<Rational>(
            "vertices",
            PrimitiveType::Vertex,
            m->get_attribute_dimension(pt_attribute.as<Rational>()));

        auto propagate_to_child_position =
            [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<Rational> { return P; };
        auto update_child_positon =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<Rational, Rational>>(
                child_position_handle,
                pt_attribute,
                propagate_to_child_position);
        update_child_positon->run_on_all();
    }


    std::map<std::string, std::vector<int64_t>> names;

    names["trimesh"] = m->absolute_multi_mesh_id();
    names["input_mesh"] = input_mesh->absolute_multi_mesh_id();

    cache.write_mesh(*m, options.output, names);
}

} // namespace wmtk::components