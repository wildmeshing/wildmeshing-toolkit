#include <catch2/catch_test_macros.hpp>

#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/AMIPSOptimizationSmoothing.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/submesh/utils/write.hpp>

using namespace wmtk;
using namespace operations;
using namespace composite;

void write(const Mesh& mesh, const std::string& name)
{
    if (mesh.top_simplex_type() == PrimitiveType::Triangle) {
        wmtk::io::ParaviewWriter writer(name, "vertices", mesh, true, true, true, false);
        mesh.serialize(writer);
    } else if (mesh.top_simplex_type() == PrimitiveType::Edge) {
        wmtk::io::ParaviewWriter writer(name, "vertices", mesh, true, true, false, false);
        mesh.serialize(writer);
    }
}

attribute::MeshAttributeHandle register_and_transfer_positions(
    attribute::MeshAttributeHandle parent_pos_handle,
    Mesh& child)
{
    auto child_pos_handle = child.register_attribute<double>(
        "vertices",
        PrimitiveType::Vertex,
        parent_pos_handle.dimension());

    throw std::runtime_error("deleted");
    // attribute_update::make_cast_attribute_transfer_strategy(
    //     /*source=*/parent_pos_handle,
    //     /*target=*/child_pos_handle)
    //     ->run_on_all();

    return child_pos_handle;
}

TEST_CASE("test_projection_operation", "[operations][.]")
{
    throw std::runtime_error("rewrite test without multimesh");
    // logger().set_level(spdlog::level::off);
    //// logger().set_level(spdlog::level::trace);
    // opt_logger().set_level(spdlog::level::off);
    //
    // std::shared_ptr<tests::DEBUG_TriMesh> mesh_ptr =
    //     std::make_shared<tests::DEBUG_TriMesh>(tests::edge_region_with_position_2D());
    // tests::DEBUG_TriMesh& m = *mesh_ptr;
    //
    //// add input (edge 4-5)
    // std::shared_ptr<tests::DEBUG_EdgeMesh> child_input_ptr;
    //{
    //     child_input_ptr = std::make_shared<tests::DEBUG_EdgeMesh>(tests::single_line());
    //
    //     const Tuple child_tuple = child_input_ptr->edge_tuple_from_vids(0, 1);
    //     const Tuple parent_tuple = m.edge_tuple_from_vids(4, 5);
    //     std::vector<std::array<Tuple, 2>> map_tuples;
    //     map_tuples.emplace_back(std::array<Tuple, 2>{child_tuple, parent_tuple});
    //     m.register_child_mesh(child_input_ptr, map_tuples);
    // }
    //// add boundary
    // std::shared_ptr<tests::DEBUG_EdgeMesh> child_bnd_ptr;
    //{
    //     child_bnd_ptr = std::make_shared<tests::DEBUG_EdgeMesh>();
    //     tests::DEBUG_EdgeMesh& c = *child_bnd_ptr;
    //
    //     RowVectors2l edges;
    //     edges.resize(8, 2);
    //     edges.row(0) << 0, 1;
    //     edges.row(1) << 1, 2;
    //     edges.row(2) << 2, 3;
    //     edges.row(3) << 3, 4;
    //     edges.row(4) << 4, 5;
    //     edges.row(5) << 5, 6;
    //     edges.row(6) << 6, 7;
    //     edges.row(7) << 7, 0;
    //     c.initialize(edges);
    //
    //     std::vector<std::array<Tuple, 2>> map_tuples;
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(0, 1), m.edge_tuple_from_vids(0, 1)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(1, 2), m.edge_tuple_from_vids(1, 2)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(2, 3), m.edge_tuple_from_vids(2, 6)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(3, 4), m.edge_tuple_from_vids(6, 9)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(4, 5), m.edge_tuple_from_vids(9, 8)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(5, 6), m.edge_tuple_from_vids(8, 7)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(6, 7), m.edge_tuple_from_vids(7, 3)});
    //     map_tuples.emplace_back(
    //         std::array<Tuple, 2>{c.edge_tuple_from_vids(7, 0), m.edge_tuple_from_vids(3, 0)});
    //     m.register_child_mesh(child_bnd_ptr, map_tuples);
    // }
    //
    // attribute::MeshAttributeHandle pos_handle =
    //     m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    //
    //// move vertices around
    //{
    //    auto acc = m.create_accessor<double>(pos_handle);
    //    const Tuple t4 = m.vertex_tuple_from_id(4);
    //    acc.vector_attribute(t4)[0] = 0.75;
    //    acc.vector_attribute(t4)[1] = 0.9;
    //}
    //
    // SECTION("multimesh")
    //{
    //    auto write_all = [&]() {
    //        static int64_t write_counter = 0;
    //        write(m, fmt::format("test_proj_mm_parent_{}", write_counter));
    //        write(*child_input_ptr, fmt::format("test_proj_mm_input_{}", write_counter));
    //        write(*child_bnd_ptr, fmt::format("test_proj_mm_bnd_{}", write_counter));
    //        ++write_counter;
    //    };
    //
    //    auto child_input_pos_handle = register_and_transfer_positions(pos_handle,
    //    *child_input_ptr); auto child_bnd_pos_handle = register_and_transfer_positions(pos_handle,
    //    *child_bnd_ptr);
    //
    //    std::vector<std::shared_ptr<AttributeTransferStrategyBase>> update_child_position;
    //    update_child_position.emplace_back(attribute_update::make_cast_attribute_transfer_strategy(
    //        /*source=*/pos_handle,
    //        /*target=*/child_input_pos_handle));
    //    update_child_position.emplace_back(attribute_update::make_cast_attribute_transfer_strategy(
    //        /*source=*/pos_handle,
    //        /*target=*/child_bnd_pos_handle));
    //    std::vector<std::shared_ptr<AttributeTransferStrategyBase>> update_parent_position;
    //    update_parent_position.emplace_back(attribute_update::make_cast_attribute_transfer_strategy(
    //        /*source=*/child_input_pos_handle,
    //        /*target=*/pos_handle));
    //    update_parent_position.emplace_back(attribute_update::make_cast_attribute_transfer_strategy(
    //        /*source=*/child_bnd_pos_handle,
    //        /*target=*/pos_handle));
    //
    //    std::vector<ProjectOperation::MeshConstrainPair> mesh_constraint_pairs;
    //    mesh_constraint_pairs.emplace_back(child_input_pos_handle, child_input_pos_handle);
    //    mesh_constraint_pairs.emplace_back(child_bnd_pos_handle, child_bnd_pos_handle);
    //
    //    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(m, pos_handle);
    //    for (auto& s : update_child_position) {
    //        smoothing->add_transfer_strategy(s);
    //    }
    //    auto proj_smoothing = std::make_shared<ProjectOperation>(m, mesh_constraint_pairs);
    //    for (auto& s : update_parent_position) {
    //        proj_smoothing->add_transfer_strategy(s);
    //    }
    //    for (auto& s : update_child_position) {
    //        proj_smoothing->add_transfer_strategy(s);
    //    }
    //
    //    const simplex::Simplex v1(PrimitiveType::Vertex, m.vertex_tuple_from_id(1));
    //
    //    write_all();
    //    (*smoothing)(v1);
    //    write_all();
    //    CHECK_NOTHROW((*proj_smoothing)(v1));
    //
    //    write_all();
    //}
    // SECTION("submesh")
    //{
    //    std::map<std::shared_ptr<Mesh>, std::shared_ptr<submesh::SubMesh>> submesh_map;
    //    auto emb_ptr = submesh::utils::submesh_from_multimesh(mesh_ptr, submesh_map);
    //    submesh::Embedding& emb = *emb_ptr;
    //    auto sub_input_ptr = submesh_map[child_input_ptr];
    //    auto sub_bnd_ptr = submesh_map[child_bnd_ptr];
    //    submesh::SubMesh& sub_input = *sub_input_ptr;
    //    submesh::SubMesh& sub_bnd = *sub_bnd_ptr;
    //
    //    auto write_all = [&]() {
    //        static int64_t write_counter = 0;
    //        submesh::utils::write(
    //            fmt::format("test_proj_sub_parent_{}", write_counter),
    //            "vertices",
    //            emb,
    //            false,
    //            true,
    //            true,
    //            false);
    //        submesh::utils::write(
    //            fmt::format("test_proj_sub_input_{}", write_counter),
    //            "vertices",
    //            sub_input,
    //            false,
    //            true,
    //            false,
    //            false);
    //        submesh::utils::write(
    //            fmt::format("test_proj_sub_bnd_{}", write_counter),
    //            "vertices",
    //            sub_bnd,
    //            false,
    //            true,
    //            false,
    //            false);
    //        ++write_counter;
    //    };
    //
    //    // deregister all child meshes
    //    for (const auto [child, sub] : submesh_map) {
    //        m.deregister_child_mesh(child);
    //    }
    //    emb.update_tag_attribute_handles();
    //
    //    write_all();
    //
    //    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(m, pos_handle);
    //    auto proj_smoothing = std::make_shared<ProjectOperation>(emb, pos_handle);
    //
    //    const simplex::Simplex v1(PrimitiveType::Vertex, m.vertex_tuple_from_id(1));
    //
    //    (*smoothing)(v1);
    //    write_all();
    //    CHECK_NOTHROW((*proj_smoothing)(v1));
    //
    //    write_all();
    //}
}