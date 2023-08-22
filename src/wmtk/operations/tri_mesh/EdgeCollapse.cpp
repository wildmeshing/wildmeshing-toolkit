

#include "EdgeCollapse.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk::operations::tri_mesh {
EdgeCollapse::EdgeCollapse(
    wmtk::Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeCollapse>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_settings{settings}
{}

bool EdgeCollapse::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.collapse_edge(m_input_tuple);
    return true;
}
bool EdgeCollapse::before() const
{
    if (m_mesh.is_outdated(m_input_tuple)) {
        return false;
    }
    if (!m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    if (!m_settings.collapse_boundary_edges && m_mesh.is_boundary(m_input_tuple)) {
        return false;
    }
    if (!m_settings.collapse_boundary_vertex_to_interior &&
        m_mesh.is_boundary_vertex(m_input_tuple)) {
        return false;
    }

    return SimplicialComplex::link_cond_bd_2d(m_mesh, m_input_tuple);
}

std::string EdgeCollapse::name() const
{
    return "tri_mesh_edge_collapse";
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
} // namespace wmtk::operations::tri_mesh
