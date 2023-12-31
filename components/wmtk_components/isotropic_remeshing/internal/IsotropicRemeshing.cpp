#include "IsotropicRemeshing.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/PredicateAwareCollapseNewAttributeStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

IsotropicRemeshing::IsotropicRemeshing(
    TriMesh& mesh,
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
    m_pos_attribute = std::make_unique<MeshAttributeHandle<double>>(
        m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));

    m_invariant_link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh);

    m_invariant_min_edge_length = std::make_shared<MinEdgeLengthInvariant>(
        m_mesh,
        *m_pos_attribute,
        m_length_max * m_length_max);

    m_invariant_max_edge_length = std::make_shared<MaxEdgeLengthInvariant>(
        m_mesh,
        *m_pos_attribute,
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

    // split
    EdgeSplit op_split(m_mesh);
    op_split.add_invariant(m_invariant_min_edge_length);
    if (m_lock_boundary) {
        op_split.add_invariant(m_invariant_interior_edge);
    }
    op_split.set_standard_strategy(
        *m_pos_attribute,
        NewAttributeStrategy::SplitBasicStrategy::None,
        NewAttributeStrategy::SplitRibBasicStrategy::Mean);

    // collapse
    EdgeCollapse op_collapse(m_mesh);
    op_collapse.add_invariant(m_invariant_link_condition);
    op_collapse.add_invariant(m_invariant_max_edge_length);
    if (m_lock_boundary) {
        op_collapse.add_invariant(m_invariant_interior_edge);
        // set collapse towards boundary
        auto tmp = std::make_shared<tri_mesh::PredicateAwareCollapseNewAttributeStrategy<double>>(
            *m_pos_attribute);
        tmp->set_standard_collapse_strategy(NewAttributeStrategy::CollapseBasicStrategy::Mean);
        tmp->set_standard_simplex_predicate(
            NewAttributeStrategy::BasicSimplexPredicate::IsInterior);
        op_collapse.set_strategy(*m_pos_attribute, tmp);
    } else {
        op_collapse.set_standard_strategy(
            *m_pos_attribute,
            NewAttributeStrategy::CollapseBasicStrategy::Mean);
    }

    // swap
    composite::TriEdgeSwap op_swap(m_mesh);
    op_swap.add_invariant(m_invariant_interior_edge);
    op_swap.add_invariant(m_invariant_valence_improve);
    op_swap.collapse().add_invariant(m_invariant_link_condition);

    // smooth
    VertexTangentialLaplacianSmooth op_smooth(m_mesh, *m_pos_attribute);
    if (m_lock_boundary) {
        op_smooth.add_invariant(m_invariant_interior_vertex);
    }

    Scheduler scheduler;
    for (long i = 0; i < iterations; ++i) {
        wmtk::logger().debug("Iteration {}", i);

        if (m_do_split) {
            scheduler.run_operation_on_all(op_split);
        }

        if (m_do_collapse) {
            scheduler.run_operation_on_all(op_collapse);
        }

        if (m_do_swap) {
            scheduler.run_operation_on_all(op_swap);
        }

        if (m_do_smooth) {
            scheduler.run_operation_on_all(op_smooth);
        }
    }
}

} // namespace wmtk::components::internal
