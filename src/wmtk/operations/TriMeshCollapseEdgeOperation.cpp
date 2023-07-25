

#include "TriMeshCollapseEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {
TriMeshCollapseEdgeOperation::TriMeshCollapseEdgeOperation(Mesh& m, const Tuple& t)
    : Operation(m)
    , m_input_tuple(t)
{}

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
    return SimplicialComplex::link_cond(m_input_tuple, m_mesh);
}

std::string TriMeshCollapseEdgeOperation::name() const
{
    return "tri_mesh_collapse_edge";
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
