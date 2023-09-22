#include "EdgeCollapseToMidpoint.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>

namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeCollapseToMidpoint>::initialize_invariants(const TriMesh& m)
{
    collapse_settings.initialize_invariants(m);
    collapse_settings.invariants.add(
        std::make_shared<MaxEdgeLengthInvariant>(m, position, max_squared_length));
}

bool OperationSettings<tri_mesh::EdgeCollapseToMidpoint>::are_invariants_initialized() const
{
    return collapse_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<MaxEdgeLengthInvariant>(
               collapse_settings.invariants);
}

namespace tri_mesh {
EdgeCollapseToMidpoint::EdgeCollapseToMidpoint(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeCollapseToMidpoint>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.collapse_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{}

std::string EdgeCollapseToMidpoint::name() const
{
    return "tri_mesh_collapse_edge_to_mid";
}

Tuple EdgeCollapseToMidpoint::return_tuple() const
{
    return m_output_tuple;
}

bool EdgeCollapseToMidpoint::before() const
{
    return TupleOperation::before();
    if (!TupleOperation::before()) {
        return false;
    }

    // TODO: this si implemented in a maxedgelengthinvariant. settings need to be adapted to use
    // invariants for this
    auto p0 = m_pos_accessor.vector_attribute(input_tuple());
    auto p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_settings.max_squared_length;
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
        EdgeCollapse split_op(mesh(), input_tuple(), m_settings.collapse_settings);
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


std::vector<Tuple> EdgeCollapseToMidpoint::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        // TODO: this is a copy paste from EdgeCollapse. Need to change operation structure to
        // enable updated primitives
        Simplex v(PrimitiveType::Vertex, m_output_tuple);
        auto sc = SimplicialComplex::open_star(mesh(), v);
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

} // namespace tri_mesh
} // namespace wmtk::operations
