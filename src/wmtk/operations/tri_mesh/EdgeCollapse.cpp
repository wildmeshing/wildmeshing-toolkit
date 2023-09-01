#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/TriMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

OperationSettings<EdgeCollapse>::OperationSettings() {}

void OperationSettings<EdgeCollapse>::initialize_invariants(const TriMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
    invariants.add(std::make_shared<TriMeshLinkConditionInvariant>(m));
    if (!collapse_boundary_edges) {
        invariants.add(std::make_shared<InteriorEdgeInvariant>(m));
    }
    if (!collapse_boundary_vertex_to_interior) {
        invariants.add(std::make_shared<InteriorVertexInvariant>(m));
    }
}

bool OperationSettings<EdgeCollapse>::are_invariants_initialized() const
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

namespace tri_mesh {

EdgeCollapse::EdgeCollapse(Mesh& m, const Tuple& t, const OperationSettings<EdgeCollapse>& settings)
    : EdgeCollapse(dynamic_cast<TriMesh&>(m), t, settings)
{}
EdgeCollapse::EdgeCollapse(
    TriMesh& m,
    const Tuple& t,
    const OperationSettings<EdgeCollapse>& settings)
    : TupleOperation(m, settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool EdgeCollapse::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.collapse_edge(input_tuple());
    return true;
}

std::vector<Tuple> EdgeCollapse::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        return modified_triangles();
    } else {
        return {};
    }
}
bool EdgeCollapse::before() const
{
    return TupleOperation::before();

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

std::string EdgeCollapse::name() const
{
    return "tri_mesh_collapse_edge";
}

Tuple EdgeCollapse::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> EdgeCollapse::modified_triangles() const
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
} // namespace tri_mesh
} // namespace wmtk::operations
