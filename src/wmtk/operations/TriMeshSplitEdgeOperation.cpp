
#include "TriMeshSplitEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {

bool TriMeshSplitEdgeOperation::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m.split_edge(m_input_tuple);
    return true;
}
bool TriMeshSplitEdgeOperation::before() const
{
    return m_mesh.is_valid(m_input_tuple);
}
std::string TriMeshSplitEdgeOperation::name() const
{
    return "tri_mesh_split_edge";
}

Tuple TriMeshSplitEdgeOperation::new_vertex() const
{
    return m_output_tuple;
}

std::vector<Tuple> TriMeshSplitEdgeOperation::modified_triangles() const
{
    Simplex v(PrimitiveType::Vertex, new_vertex() );
    auto sc = SimplicialComplex::open_star(v, m_mesh);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}
} // namespace wmtk
