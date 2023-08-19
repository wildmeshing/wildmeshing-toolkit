

#include "TriMeshCollapseEdgeOperation.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/TriMeshLinkConditionInvariant.hpp>

namespace wmtk {

OperationSettings<TriMeshCollapseEdgeOperation>::OperationSettings(const TriMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
    invariants.add(std::make_shared<TriMeshLinkConditionInvariant>(m));
}

TriMeshCollapseEdgeOperation::TriMeshCollapseEdgeOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshCollapseEdgeOperation>& settings)
    : TriMeshCollapseEdgeOperation(dynamic_cast<TriMesh&>(m), t, settings)
{}
TriMeshCollapseEdgeOperation::TriMeshCollapseEdgeOperation(
    TriMesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshCollapseEdgeOperation>& settings)
    : TupleOperation(m, settings.invariants, t)
    , m_settings{settings}
{}

bool TriMeshCollapseEdgeOperation::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.collapse_edge(input_tuple());
    return true;
}

std::vector<Tuple> TriMeshCollapseEdgeOperation::modified_primitives(
    PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        return modified_triangles();
    } else {
        return {};
    }
}
/*
bool TriMeshCollapseEdgeOperation::before() const
{
    if (m_mesh.is_outdated(input_tuple())) {
        return false;
    }
    if (!m_mesh.is_valid(input_tuple())) {
        return false;
    }

    if (!m_settings.collapse_boundary_edges && m_mesh.is_boundary(input_tuple())) {
        return false;
    }
    if (!m_settings.collapse_boundary_vertex_to_interior &&
        m_mesh.is_boundary_vertex(input_tuple())) {
        return false;
    }

    return SimplicialComplex::link_cond_bd_2d(m_mesh, input_tuple());
}
*/

std::string TriMeshCollapseEdgeOperation::name() const
{
    return "tri_mesh_collapse_edge";
}

Tuple TriMeshCollapseEdgeOperation::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> TriMeshCollapseEdgeOperation::modified_triangles() const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    auto sc = SimplicialComplex::open_star(m_mesh, v);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}
} // namespace wmtk
