#pragma once

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>

namespace wmtk::components::internal {

class IsotropicRemeshing
{
public:
    IsotropicRemeshing(
        TriMesh& mesh,
        const double length,
        const bool lock_boundary,
        const bool preserve_childmesh_topology,
        const bool preserve_childmesh_geometry,
        const bool do_split,
        const bool do_collapse,
        const bool do_swap,
        const bool do_smooth,
        const bool debug_output);

    void remeshing(const long iterations);

private:
    TriMesh& m_mesh;
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();
    bool m_lock_boundary = true;
    bool m_preserve_childmesh_topology = false;
    bool m_preserve_childmesh_geometry = false;
    bool m_do_split = true;
    bool m_do_collapse = true;
    bool m_do_swap = true;
    bool m_do_smooth = true;
    bool m_debug_output = false;

    std::unique_ptr<MeshAttributeHandle<double>> m_pos_attribute;

    std::shared_ptr<MultiMeshLinkConditionInvariant> m_invariant_link_condition;
    std::shared_ptr<MinEdgeLengthInvariant> m_invariant_min_edge_length;
    std::shared_ptr<MaxEdgeLengthInvariant> m_invariant_max_edge_length;
    std::shared_ptr<invariants::InteriorSimplexInvariant> m_invariant_interior_edge;
    std::shared_ptr<invariants::InteriorSimplexInvariant> m_invariant_interior_vertex;
    std::shared_ptr<invariants::ValenceImprovementInvariant> m_invariant_valence_improve;

    std::shared_ptr<operations::CollapseNewAttributeStrategy> m_pos_collapse_strategy;
};

} // namespace wmtk::components::internal
