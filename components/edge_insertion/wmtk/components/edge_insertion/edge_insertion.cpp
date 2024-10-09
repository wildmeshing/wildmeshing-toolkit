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

#include <fstream>

namespace wmtk::components {

using namespace internal;

EdgeInsertionMeshes edge_insertion(EdgeMesh& input_mesh, TriMesh& bg_mesh)
{
    std::vector<Vector2r> v_final;
    std::vector<std::array<int64_t, 3>> FV_new;
    std::vector<std::array<int, 3>> local_e_on_input;

    wmtk::logger().info("start edge insertion ...");

    edge_insertion(bg_mesh, input_mesh, v_final, FV_new, local_e_on_input);

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

    // debug code
    std::ofstream f("ei_debug.txt");
    f << FV << std::endl;

    // remove unused vertices
    std::vector<bool> v_is_used(V.rows(), false);
    for (int64_t i = 0; i < FV.rows(); ++i) {
        for (int64_t j = 0; j < 3; ++j) {
            v_is_used[FV(i, j)] = true;
        }
    }

    std::map<int64_t, int64_t> v_map;
    int64_t used_cnt = 0;
    for (auto b : v_is_used) {
        if (b) {
            used_cnt++;
        }
    }

    MatrixX<Rational> V_valid;
    MatrixX<int64_t> FV_valid;

    V_valid.resize(used_cnt, 2);
    int64_t valid_row = 0;
    for (int64_t i = 0; i < V.rows(); ++i) {
        if (v_is_used[i]) {
            V_valid.row(valid_row) = V.row(i);
            v_map[i] = valid_row;
            valid_row++;
        }
    }

    FV_valid.resize(FV.rows(), 3);
    for (int64_t i = 0; i < FV.rows(); ++i) {
        for (int64_t j = 0; j < 3; ++j) {
            FV_valid(i, j) = v_map[FV(i, j)];
        }
    }

    std::shared_ptr<wmtk::TriMesh> m = std::make_shared<wmtk::TriMesh>();
    m->initialize(FV_valid);
    mesh_utils::set_matrix_attribute(V_valid, "vertices", PrimitiveType::Vertex, *m);

    // input child mesh
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

    std::shared_ptr<Mesh> inserted_input_mesh;

    internal::MultiMeshFromTag mmft(*m, input_handle, 1);
    mmft.compute_substructure_mesh();

    inserted_input_mesh = m->get_child_meshes().back();

    mmft.remove_soup();

    // bbox child mesh
    auto bbox_handle = m->register_attribute<int64_t>("bbox", PrimitiveType::Edge, 1);
    auto bbox_accessor = m->create_accessor<int64_t>(bbox_handle);

    for (const auto& e : m->get_all(PrimitiveType::Edge)) {
        if (m->is_boundary(PrimitiveType::Edge, e)) {
            bbox_accessor.scalar_attribute(e) = 1;
        }
    }

    std::shared_ptr<Mesh> bbox_mesh;

    internal::MultiMeshFromTag mmft2(*m, bbox_handle, 1);
    mmft2.compute_substructure_mesh();

    bbox_mesh = m->get_child_meshes().back();

    mmft2.remove_soup();

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

    EdgeInsertionMeshes eim;
    eim.inserted_edge_mesh = inserted_input_mesh;
    eim.tri_mesh = m;
    eim.bbox_mesh = bbox_mesh;

    return eim;
}

} // namespace wmtk::components