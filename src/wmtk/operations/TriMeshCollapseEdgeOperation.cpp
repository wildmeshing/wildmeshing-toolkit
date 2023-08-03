

#include "TriMeshCollapseEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {
TriMeshCollapseEdgeOperation::TriMeshCollapseEdgeOperation(Mesh& m, const Tuple& t)
    : Operation(m)
    , m_input_tuple(t)
{
    if (m_mesh.is_valid(m_input_tuple)){
        m_is_output_tuple_from_left_ear = !m_mesh.is_boundary(m_mesh.switch_tuple(m_input_tuple, PrimitiveType::Edge));
    }
    
}

bool TriMeshCollapseEdgeOperation::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.collapse_edge(m_input_tuple);
    return true;
}
bool TriMeshCollapseEdgeOperation::before() const
{
    if(!m_mesh.is_valid(m_input_tuple)) {
        return false;
    }
    return SimplicialComplex::link_cond_bd_2d(m_input_tuple, m_mesh);
}

std::string TriMeshCollapseEdgeOperation::name() const
{
    return "tri_mesh_collapse_edge";
}

Tuple TriMeshCollapseEdgeOperation::return_tuple() const
{
    return m_output_tuple;
}

bool TriMeshCollapseEdgeOperation::is_return_tuple_from_left_ear() const
{
    return m_is_output_tuple_from_left_ear;
}
std::vector<Tuple> TriMeshCollapseEdgeOperation::modified_triangles() const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    auto sc = SimplicialComplex::open_star(v, m_mesh);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}
} // namespace wmtk
