#include "EdgeCollapseToMidpoint.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>


namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeCollapseToMidpoint>::create_invariants()
{
    OperationSettings<tri_mesh::EdgeCollapse>::create_invariants();

    invariants->add(std::make_shared<MaxEdgeLengthInvariant>(m_mesh, position, max_squared_length));
}


namespace tri_mesh {
EdgeCollapseToMidpoint::EdgeCollapseToMidpoint(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<EdgeCollapseToMidpoint>& settings)
    : EdgeCollapse(m,t,settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Edge);
}

std::string EdgeCollapseToMidpoint::name() const
{
    return "tri_mesh_collapse_edge_to_mid";
}


bool EdgeCollapseToMidpoint::before() const
{
    return TupleOperation::before();
}

bool EdgeCollapseToMidpoint::execute()
{
    // cache endpoint data for computing the midpoint
    bool v0_is_boundary = false;
    bool v1_is_boundary = false;
    auto p0 = m_pos_accessor.vector_attribute(input_tuple()).eval();
    auto p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple())).eval();
    if (m_settings.collapse_towards_boundary) {
        v0_is_boundary = mesh().is_boundary_vertex(input_tuple());
        v1_is_boundary = mesh().is_boundary_vertex(mesh().switch_vertex(input_tuple()));
    }

    // collapse
    {
        if(!EdgeCollapse::execute()) {
            return false;
        }
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


} // namespace tri_mesh
} // namespace wmtk::operations
