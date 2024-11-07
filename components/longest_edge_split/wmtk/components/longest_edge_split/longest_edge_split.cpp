#include "longest_edge_split.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/MeshConsolidate.hpp>
#include <wmtk/operations/attribute_new/NewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::longest_edge_split {

void longest_edge_split(Mesh& mesh_in, const LongestEdgeSplitOptions& options)
{
    if (mesh_in.top_simplex_type() != PrimitiveType::Edge &&
        mesh_in.top_simplex_type() != PrimitiveType::Triangle &&
        mesh_in.top_simplex_type() != PrimitiveType::Tetrahedron) {
        log_and_throw_error(
            "logest edge split works only for edge, triangle, or tet meshes: {}",
            primitive_type_name(mesh_in.top_simplex_type()));
    }

    if (!mesh_in.is_multi_mesh_root()) {
        log_and_throw_error("The mesh passed in longest_edge_split must be the root mesh");
    }

    attribute::MeshAttributeHandle position_handle = options.position_handle;
    std::vector<attribute::MeshAttributeHandle> other_position_handles =
        options.other_position_handles;

    Mesh& mesh = position_handle.mesh();

    std::vector<attribute::MeshAttributeHandle> pass_through_attributes =
        options.pass_through_attributes;

    for (auto& h : other_position_handles) {
        pass_through_attributes.emplace_back(h);
    }

    /////////////////////////////////////////////

    auto visited_edge_flag =
        mesh.register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

    auto update_flag_func = [](Eigen::Ref<const Eigen::MatrixXd> P) -> Eigen::VectorX<char> {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto tag_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, double>>(
            visited_edge_flag,
            position_handle,
            update_flag_func);

    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](Eigen::Ref<const Eigen::MatrixXd> P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_attribute,
            position_handle,
            compute_edge_length);
    edge_length_update->run_on_all();

    //////////////////////////////////
    // computing bbox diagonal
    Eigen::VectorXd bmin(position_handle.dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(position_handle.dimension());
    bmax.setConstant(std::numeric_limits<double>::lowest());

    auto pt_accessor = mesh.create_const_accessor<double>(position_handle);

    const auto vertices = mesh.get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();

    const double length_abs = bbdiag * options.length_rel;

    wmtk::logger().info(
        "bbox max {}, bbox min {}, diag {}, target edge length {}",
        bmax,
        bmin,
        bbdiag,
        length_abs);

    auto long_edges_first_priority = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return -edge_length_accessor.const_scalar_attribute(s.tuple());
    };
    pass_through_attributes.push_back(edge_length_attribute);
    auto todo =
        std::make_shared<TodoLargerInvariant>(mesh, edge_length_attribute.as<double>(), length_abs);

    //////////////////////////invariants
    auto invariant_interior_edge = std::make_shared<invariants::InvariantCollection>(mesh);
    auto invariant_interior_vertex = std::make_shared<invariants::InvariantCollection>(mesh);

    auto set_all_invariants = [&](auto&& m) {
        invariant_interior_edge->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Edge));
        invariant_interior_vertex->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Vertex));
    };
    wmtk::multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(mesh);

    ////////////// positions
    std::vector<attribute::MeshAttributeHandle> position_handles;
    position_handles.emplace_back(position_handle);

    //////////////////////////////////////////
    // split
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(mesh);
    split->add_invariant(todo);

    split->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    split->add_transfer_strategy(tag_update);

    split->set_priority(long_edges_first_priority);
    split->add_transfer_strategy(edge_length_update);


    for (const auto& pos_handle : position_handles) {
        split->set_new_attribute_strategy(pos_handle);
    }

    for (const auto& attr : pass_through_attributes) {
        split->set_new_attribute_strategy(attr);
    }

    auto propagate_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd { return P; };
    for (auto& h : other_position_handles) {
        auto transfer_position =
            std::make_shared<operations::SingleAttributeTransferStrategy<double, double>>(
                h,
                position_handle,
                propagate_position);
        split->add_transfer_strategy(transfer_position);
    }


    //////////////////////////////////////////
    Scheduler scheduler;
    SchedulerStats pass_stats =
        scheduler.run_operation_on_all(*split, visited_edge_flag.as<char>());

    logger().info(
        "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
        pass_stats.number_of_performed_operations(),
        pass_stats.number_of_successful_operations(),
        pass_stats.number_of_failed_operations(),
        pass_stats.collecting_time,
        pass_stats.sorting_time,
        pass_stats.executing_time);

    // wmtk::multimesh::consolidate(mesh);
    auto op_consolidate = wmtk::operations::MeshConsolidate(mesh);
    op_consolidate(simplex::Simplex(mesh, PrimitiveType::Vertex, Tuple()));
}

void longest_edge_split(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& position_handle,
    const double length_rel,
    const std::vector<attribute::MeshAttributeHandle>& pass_through)
{
    LongestEdgeSplitOptions options;
    options.position_handle = position_handle;
    options.length_rel = length_rel;

    options.pass_through_attributes = pass_through;
    longest_edge_split(mesh, options);
}
} // namespace wmtk::components::longest_edge_split
