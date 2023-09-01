#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/TriMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

OperationSettings<tri_mesh::EdgeCollapse>::OperationSettings() {}

void OperationSettings<tri_mesh::EdgeCollapse>::initialize_invariants(const TriMesh& m)
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

bool OperationSettings<tri_mesh::EdgeCollapse>::are_invariants_initialized() const
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
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool EdgeCollapse::execute()
{
    m_output_tuple = mesh().collapse_edge(input_tuple(), hash_accessor());
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
    auto sc = SimplicialComplex::open_star(mesh(), v);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}
} // namespace tri_mesh
} // namespace wmtk::operations
