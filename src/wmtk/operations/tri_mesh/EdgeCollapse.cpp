#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TriMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>

namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeCollapse>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
    invariants->add(std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
    if (!collapse_boundary_edges) {
        invariants->add(std::make_shared<InteriorEdgeInvariant>(m_mesh));
    }
    if (!collapse_boundary_vertex_to_interior) {
        invariants->add(std::make_shared<InteriorVertexInvariant>(m_mesh));
    }
}

namespace tri_mesh {

EdgeCollapse::EdgeCollapse(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<EdgeCollapse>& settings)
    : EdgeCollapse(dynamic_cast<TriMesh&>(m), t, settings)
{}
EdgeCollapse::EdgeCollapse(
    TriMesh& m,
    const Simplex& t,
    const OperationSettings<EdgeCollapse>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
// , m_settings(settings)
{
    // assert(m_settings.are_invariants_initialized());
}

bool EdgeCollapse::execute()
{
    auto return_data = operations::utils::multi_mesh_edge_collapse(mesh(), input_tuple().tuple());

    const operations::tri_mesh::EdgeOperationData& my_data =
        return_data.get(mesh(), Simplex(PrimitiveType::Edge, input_tuple().tuple()));
    // move vertex to center of old vertices
    m_output_tuple = my_data.m_output_tuple;

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
