#include "IsotropicRemeshing.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/FusionEdgeInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/MeshConsolidate.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/utils/VertexTangentialLaplacianSmooth.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

void isotropic_remeshing(
    attribute::MeshAttributeHandle& position,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    const double length,
    const bool lock_boundary,
    const bool use_for_periodic,
    const bool dont_disable_split,
    const bool fix_uv_seam,
    const int64_t iterations,
    const std::vector<attribute::MeshAttributeHandle>& other_positions,
    bool update_other_positions,
    const std::optional<attribute::MeshAttributeHandle>& position_for_inversion)
{
    assert(dynamic_cast<TriMesh*>(&position.mesh()) != nullptr);

    TriMesh& mesh = static_cast<TriMesh&>(position.mesh());

    const double length_min = (4. / 5.) * length;
    const double length_max = (4. / 3.) * length;

    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(position);

    auto invariant_link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);

    auto invariant_min_edge_length = std::make_shared<MinEdgeLengthInvariant>(
        mesh,
        position.as<double>(),
        length_max * length_max);

    auto invariant_max_edge_length = std::make_shared<MaxEdgeLengthInvariant>(
        mesh,
        position.as<double>(),
        length_min * length_min);

    auto invariant_interior_edge = std::make_shared<invariants::InvariantCollection>(mesh);
    auto invariant_interior_vertex = std::make_shared<invariants::InvariantCollection>(mesh);

    auto set_all_invariants = [&](auto&& m) {
        invariant_interior_edge->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Edge));
        invariant_interior_vertex->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Vertex));
    };
    multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);

    auto invariant_valence_improve =
        std::make_shared<invariants::ValenceImprovementInvariant>(mesh);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(mesh);

    auto update_position_func = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.col(0);
    };
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        update_position;

    if (update_other_positions) {
        update_position =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                other_positions.front(),
                position,
                update_position_func);
    }

    using namespace operations;

    assert(mesh.is_connectivity_valid());

    std::vector<std::shared_ptr<Operation>> ops;

    // split
    auto op_split = std::make_shared<EdgeSplit>(mesh);
    op_split->add_invariant(invariant_min_edge_length);
    if (lock_boundary && !use_for_periodic && !dont_disable_split) {
        op_split->add_invariant(invariant_interior_edge);
    }
    for (auto& p : positions) {
        op_split->set_new_attribute_strategy(
            p,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
    }
    for (const auto& attr : pass_through_attributes) {
        op_split->set_new_attribute_strategy(attr);
    }
    // ops.push_back(op_split);


    //////////////////////////////////////////
    // collapse
    auto op_collapse = std::make_shared<EdgeCollapse>(mesh);
    op_collapse->add_invariant(invariant_link_condition);
    if (position_for_inversion) {
        op_collapse->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    op_collapse->add_invariant(invariant_max_edge_length);
    op_collapse->add_invariant(invariant_mm_map);

    // hack for uv
    if (fix_uv_seam) {
        op_collapse->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (lock_boundary && !use_for_periodic) {
        op_collapse->add_invariant(invariant_interior_edge);
        // set collapse towards boundary
        for (auto& p : positions) {
            auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(CollapseBasicStrategy::Mean);
            tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            op_collapse->set_new_attribute_strategy(p, tmp);
        }
    } else if (use_for_periodic) {
        op_collapse->add_invariant(
            std::make_shared<invariants::FusionEdgeInvariant>(mesh, mesh.get_multi_mesh_root()));
        for (auto& p : positions) {
            op_collapse->set_new_attribute_strategy(p, CollapseBasicStrategy::Mean);
        }
    } else {
        for (auto& p : positions) {
            op_collapse->set_new_attribute_strategy(p, CollapseBasicStrategy::Mean);
        }
    }


    for (const auto& attr : pass_through_attributes) {
        op_collapse->set_new_attribute_strategy(attr);
    }
    ops.push_back(op_collapse);


    //////////////////////////////////////////
    // swap
    auto op_swap = std::make_shared<composite::TriEdgeSwap>(mesh);
    op_swap->add_invariant(invariant_interior_edge);

    // hack for uv
    if (fix_uv_seam) {
        op_swap->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    op_swap->add_invariant(invariant_valence_improve);
    op_swap->collapse().add_invariant(invariant_link_condition);
    op_swap->collapse().add_invariant(invariant_mm_map);
    for (auto& p : positions) {
        op_swap->split().set_new_attribute_strategy(
            p,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
    }
    if (position_for_inversion) {
        op_swap->collapse().add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    for (auto& p : positions)
        op_swap->collapse().set_new_attribute_strategy(p, CollapseBasicStrategy::CopyOther);
    for (const auto& attr : pass_through_attributes) {
        op_swap->split().set_new_attribute_strategy(attr);
        op_swap->collapse().set_new_attribute_strategy(attr);
    }
    // ops.push_back(op_swap);


    //////////////////////////////////////////
    // smooth
    auto op_smooth = std::make_shared<AttributesUpdateWithFunction>(mesh);
    if (position.dimension() == 3) {
        op_smooth->set_function(VertexTangentialLaplacianSmooth(position));
    } else {
        op_smooth->set_function(VertexLaplacianSmooth(position));
    }

    if (lock_boundary) {
        op_smooth->add_invariant(invariant_interior_vertex);
    }

    // hack for uv
    if (fix_uv_seam) {
        op_smooth->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (position_for_inversion) {
        op_smooth->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    if (update_position) op_smooth->add_transfer_strategy(update_position);
    // ops.push_back(op_smooth);


    //////////////////////////////////////////
    Scheduler scheduler;
    for (long i = 0; i < iterations; ++i) {
        wmtk::logger().info("Iteration {}", i);

        SchedulerStats pass_stats;
        for (size_t j = 0; j < ops.size(); ++j) {
            const auto& op = ops[j];
            pass_stats += scheduler.run_operation_on_all(*op);
        }

        auto op_consolidate = MeshConsolidate(mesh);
        op_consolidate(simplex::Simplex(PrimitiveType::Vertex, Tuple()));
        // multimesh::consolidate(mesh);

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        // multimesh::consolidate(mesh);
    }
}

} // namespace wmtk::components::internal
