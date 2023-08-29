#include "TriMeshCollapseEdgeToMidpointOperation.hpp"
#include <wmtk/SimplicialComplex.hpp>

#include <wmtk/TriMesh.hpp>
#include "TriMeshCollapseEdgeOperation.hpp"

namespace wmtk {

void OperationSettings<TriMeshCollapseEdgeOperation>::initialize_invariants(const TriMesh& m)
{
    collapse_settings.initialize_invariants(m);
        invariants.add(std::make_shared<InteriorEdgeInvariant>(m));
}

bool OperationSettings<TriMeshCollapseEdgeOperation>::are_invariants_initialized() const
{
    if (!collapse_boundary_edges) {
        return find_invariants_in_collection_by_type<InteriorEdgeInvariant>(invariants);
    }

    if (!collapse_boundary_vertex_to_interior) {
        return find_invariants_in_collection_by_type<InteriorVertexInvariant>(invariants);
    }
    return find_invariants_in_collection_by_type<
        ValidTupleInvariant,
        TriMeshLinkConditionInvariant>(invariants);
}
wmtk::TriMeshCollapseEdgeToMidpointOperation::TriMeshCollapseEdgeToMidpointOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshCollapseEdgeToMidpointOperation>& settings)
    : TupleOperation(m, settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
}

std::string TriMeshCollapseEdgeToMidpointOperation::name() const
{
    return "tri_mesh_collapse_edge_to_mid";
}

Tuple TriMeshCollapseEdgeToMidpointOperation::return_tuple() const
{
    return m_output_tuple;
}

bool TriMeshCollapseEdgeToMidpointOperation::before() const
{
    if(!TupleOperation::before()) {
        return false;
    }

    // TODO: this si implemented in a maxedgelengthinvariant. settings need to be adapted to use invariants for this
    auto p0 = m_pos_accessor.vector_attribute(input_tuple());
    auto p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(input_tuple()));
    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_settings.max_squared_length;
}

bool TriMeshCollapseEdgeToMidpointOperation::execute()
{
    // cache endpoint data for computing the midpoint
    bool v0_is_boundary = false;
    bool v1_is_boundary = false;
    auto p0 = m_pos_accessor.vector_attribute(input_tuple()).eval();
    auto p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(input_tuple())).eval();
    if (m_settings.collapse_towards_boundary) {
        v0_is_boundary = m_mesh.is_boundary_vertex(input_tuple());
        v1_is_boundary = m_mesh.is_boundary_vertex(m_mesh.switch_vertex(input_tuple()));
    }

    // collapse
    {
        OperationSettings<TriMeshCollapseEdgeOperation> op_settings;
        op_settings.collapse_boundary_edges = m_settings.collapse_boundary_edges;
        op_settings.initialize_invariants(static_cast<TriMesh&>(m_mesh));

        TriMeshCollapseEdgeOperation split_op(m_mesh, input_tuple(), op_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    // execute according to endpoint data
    if (v0_is_boundary && !v1_is_boundary) {
        m_pos_accessor.vector_attribute(m_output_tuple) = p0;
    } else if (v1_is_boundary && !v0_is_boundary) {
        m_pos_accessor.vector_attribute(m_output_tuple) = p1;
    } else {
        m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);
    }

    return true;
}


std::vector<Tuple> TriMeshCollapseEdgeToMidpointOperation::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        // TODO: this is a copy paste from TriMeshCollapseEdgeOperation. Need to change operation structure to enable updated primitives
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    auto sc = SimplicialComplex::open_star(m_mesh, v);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
    } else {
        return {};
    }
}

} // namespace wmtk
