#include "IsotropicRemeshing.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
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

IsotropicRemeshing::IsotropicRemeshing(
    TriMesh& mesh,
    attribute::MeshAttributeHandle& position,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    const double length,
    const bool lock_boundary,
    const bool preserve_childmesh_topology,
    const bool preserve_childmesh_geometry,
    const bool do_split,
    const bool do_collapse,
    const bool do_swap,
    const bool do_smooth,
    const bool debug_output)
    : m_mesh{mesh}
    , m_pos_attribute{position}
    , m_pass_through_attributes{pass_through_attributes}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_preserve_childmesh_topology{preserve_childmesh_topology}
    , m_preserve_childmesh_geometry{preserve_childmesh_geometry}
    , m_do_split{do_split}
    , m_do_collapse{do_collapse}
    , m_do_swap{do_swap}
    , m_do_smooth{do_smooth}
    , m_debug_output{debug_output}
{
    m_invariant_link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh);

    m_invariant_min_edge_length = std::make_shared<MinEdgeLengthInvariant>(
        m_mesh,
        m_pos_attribute.as<double>(),
        m_length_max * m_length_max);

    m_invariant_max_edge_length = std::make_shared<MaxEdgeLengthInvariant>(
        m_mesh,
        m_pos_attribute.as<double>(),
        m_length_min * m_length_min);

    m_invariant_interior_edge =
        std::make_shared<invariants::InteriorSimplexInvariant>(m_mesh, PrimitiveType::Edge);

    m_invariant_interior_vertex =
        std::make_shared<invariants::InteriorSimplexInvariant>(m_mesh, PrimitiveType::Vertex);

    m_invariant_valence_improve = std::make_shared<invariants::ValenceImprovementInvariant>(m_mesh);
}

void IsotropicRemeshing::remeshing(const long iterations)
{
    using namespace operations;

    assert(m_mesh.is_connectivity_valid());

    // split
    EdgeSplit op_split(m_mesh);
    op_split.add_invariant(m_invariant_min_edge_length);
    if (m_lock_boundary) {
        op_split.add_invariant(m_invariant_interior_edge);
    }
    op_split.set_new_attribute_strategy(
        m_pos_attribute,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);

    for (const auto& attr : m_pass_through_attributes) {
        op_split.set_new_attribute_strategy(attr);
    }

    // collapse
    EdgeCollapse op_collapse(m_mesh);
    op_collapse.add_invariant(m_invariant_link_condition);
    op_collapse.add_invariant(m_invariant_max_edge_length);
    op_collapse.add_invariant(std::make_shared<MultiMeshMapValidInvariant>(m_mesh));
    if (m_lock_boundary) {
        op_collapse.add_invariant(m_invariant_interior_edge);
        // set collapse towards boundary
        auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(m_pos_attribute);
        tmp->set_strategy(CollapseBasicStrategy::Mean);
        tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
        op_collapse.set_new_attribute_strategy(m_pos_attribute, tmp);
    } else {
        op_collapse.set_new_attribute_strategy(m_pos_attribute, CollapseBasicStrategy::Mean);
    }

    for (const auto& attr : m_pass_through_attributes) {
        op_collapse.set_new_attribute_strategy(attr);
    }

    // swap
    composite::TriEdgeSwap op_swap(m_mesh);
    op_swap.add_invariant(m_invariant_interior_edge);
    op_swap.add_invariant(m_invariant_valence_improve);
    op_swap.collapse().add_invariant(m_invariant_link_condition);
    op_swap.collapse().add_invariant(std::make_shared<MultiMeshMapValidInvariant>(m_mesh));
    op_swap.split().set_new_attribute_strategy(
        m_pos_attribute,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);
    op_swap.collapse().set_new_attribute_strategy(
        m_pos_attribute,
        CollapseBasicStrategy::CopyOther);

    for (const auto& attr : m_pass_through_attributes) {
        op_swap.split().set_new_attribute_strategy(attr);
        op_swap.collapse().set_new_attribute_strategy(attr);
    }

    // smooth
    AttributesUpdateWithFunction op_smooth(m_mesh);
    op_smooth.set_function(VertexTangentialLaplacianSmooth(m_pos_attribute));
    if (m_lock_boundary) {
        op_smooth.add_invariant(m_invariant_interior_vertex);
    }

    Scheduler scheduler;
    for (long i = 0; i < iterations; ++i) {
        wmtk::logger().debug("Iteration {}", i);

        if (m_do_split) {
            scheduler.run_operation_on_all(op_split);
        }

        // if (m_do_collapse) {
        //     scheduler.run_operation_on_all(op_collapse);
        // }

        // if (m_do_swap) {
        //     scheduler.run_operation_on_all(op_swap);
        // }

        // if (m_do_smooth) {
        //     scheduler.run_operation_on_all(op_smooth);
        // }
    }
}

} // namespace wmtk::components::internal
