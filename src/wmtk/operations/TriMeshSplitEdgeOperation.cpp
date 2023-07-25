
#include "TriMeshSplitEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {

TriMeshSplitEdgeOperation::TriMeshSplitEdgeOperation(Mesh& m, const Tuple& t)
    : Operation(m)
    , m_input_tuple(t)
{}
bool TriMeshSplitEdgeOperation::execute()
{
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);



    if(m.split_edge(m_input_tuple)) {

            for(const acc: tri_accessors) {
            ConstACcessor old_tri_acc(acc, checkpoint);
        for(tri: new_triangles) {

            value = old_tri_acc(m_input_tuple);
                acc.assign(old,tri);
            }
        }
        for(edge: new_edges) {
            for(const acc: edge_accessors) {
                acc.assign(old,edge);
            }
        }
            return true;;
    }

    return false;
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
    Simplex v(PrimitiveType::Vertex, new_vertex());
    auto sc = SimplicialComplex::open_star(v, m_mesh);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}

std::vector<Tuple> new_triangles() const  {
    return triangle_onering();
}
std::vector<Tuple> new_edges() const  {
    return spline_edges + rib_edges();
}


bool MySplitEdge::after()const {

    for(const auto& tri: modified_triangles()) {
        positive_area_invariant(m,tri);
    }


}
} // namespace wmtk
