

#include "TriMeshCollapseEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {

bool TriMeshCollapseEdgeOperation::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.collapse_edge(m_input_tuple);
    return true;
}
bool TriMeshCollapseEdgeOperation::before() const
{
    return m_mesh.is_valid(m_input_tuple);
}
std::string TriMeshCollapseEdgeOperation::name() const
{
    return "tri_mesh_collapse_edge";
}


std::vector<Tuple> TriMeshCollapseEdgeOperation::modified_triangles() const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple );
    auto sc = SimplicialComplex::open_star(v, m_mesh);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}
} // namespace wmtk
