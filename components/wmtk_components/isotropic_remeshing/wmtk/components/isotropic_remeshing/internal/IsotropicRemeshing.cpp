#include "IsotropicRemeshing.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/utils/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

void isotropic_remeshing(
    TriMesh& mesh,
    attribute::MeshAttributeHandle& position,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    const double length,
    const bool lock_boundary,
    const int64_t iterations,
    const std::vector<attribute::MeshAttributeHandle>& other_positions,
    const std::optional<attribute::MeshAttributeHandle>& position_for_inversion)
{
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

    using namespace operations;

    assert(mesh.is_connectivity_valid());

    std::vector<std::shared_ptr<Operation>> ops;

    // split
    auto op_split = std::make_shared<EdgeSplit>(mesh);
    op_split->add_invariant(invariant_min_edge_length);
    if (lock_boundary) {
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
    ops.push_back(op_split);


    //////////////////////////////////////////
    // collapse
    auto op_collapse = std::make_shared<EdgeCollapse>(mesh);
    op_collapse->add_invariant(invariant_link_condition);
    if (position_for_inversion) {
        op_collapse->add_invariant(std::make_shared<SimplexInversionInvariant>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }
    op_collapse->add_invariant(invariant_max_edge_length);
    op_collapse->add_invariant(invariant_mm_map);
    if (lock_boundary) {
        op_collapse->add_invariant(invariant_interior_edge);
        // set collapse towards boundary
        for (auto& p : positions) {
            auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(CollapseBasicStrategy::Mean);
            tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            op_collapse->set_new_attribute_strategy(p, tmp);
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
    // op_swap->add_invariant(invariant_inversion);
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
        op_swap->collapse().add_invariant(std::make_shared<SimplexInversionInvariant>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }
    for (auto& p : positions)
        op_swap->collapse().set_new_attribute_strategy(p, CollapseBasicStrategy::CopyOther);
    for (const auto& attr : pass_through_attributes) {
        op_swap->split().set_new_attribute_strategy(attr);
        op_swap->collapse().set_new_attribute_strategy(attr);
    }
    ops.push_back(op_swap);


    //////////////////////////////////////////
    // smooth
    auto op_smooth = std::make_shared<AttributesUpdateWithFunction>(mesh);
    op_smooth->set_function(VertexTangentialLaplacianSmooth(position));
    // op_smooth->add_invariant(invariant_inversion);
    if (lock_boundary) {
        op_smooth->add_invariant(invariant_interior_vertex);
    }
    if (position_for_inversion) {
        op_smooth->add_invariant(std::make_shared<SimplexInversionInvariant>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }
    ops.push_back(op_smooth);


    //////////////////////////////////////////
    Scheduler scheduler;
    for (long i = 0; i < iterations; ++i) {
        wmtk::logger().info("Iteration {}", i);

        SchedulerStats pass_stats;
        for (auto& op : ops) pass_stats += scheduler.run_operation_on_all(*op);

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);
    }
}

} // namespace wmtk::components::internal
