
#include "TriMeshSplitEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk {

TriMeshSplitEdgeOperation::TriMeshSplitEdgeOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshSplitEdgeOperation>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_settings{settings}
{}
bool TriMeshSplitEdgeOperation::execute()
{
    // move vertex to center of old vertices
    TriMesh& m = dynamic_cast<TriMesh&>(m_mesh);
    m_output_tuple = m.split_edge(m_input_tuple);

    //    for(const acc: tri_accessors) {
    //    ConstACcessor old_tri_acc(acc, checkpoint);
    // for(tri: new_triangles) {

    //    value = old_tri_acc(m_input_tuple);
    //        acc.assign(old,tri);
    //    }
    //}
    // for(edge: new_edges) {
    //    for(const acc: edge_accessors) {
    //        acc.assign(old,edge);
    //    }
    //}
    return true;
}
bool TriMeshSplitEdgeOperation::before() const
{
    if (m_mesh.is_outdated(m_input_tuple) || !m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    if (!m_settings.split_boundary_edges && m_mesh.is_boundary(m_input_tuple)) {
        return false;
    }

    return true;
}

// potential after-like strucutre?
// bool TriMeshSplitEdgeOperation::after() const
//{
//    // energy decrease
//
//    //ConstAccessor<double> current_positions = {}; // current scope
//    //ConstAccessor<double> original_positions = {};// use checkpoint
//
//    //if(
//    //        get_area(current_positions, one_ring(m_output_tuple))
//    //        <
//    //        get_area(original_positions, one_ring(m_input_tuple))) {
//    //    return false;;
//    //}
//}
std::string TriMeshSplitEdgeOperation::name() const
{
    return "tri_mesh_split_edge";
}

Tuple TriMeshSplitEdgeOperation::new_vertex() const
{
    return m_output_tuple;
}

Tuple TriMeshSplitEdgeOperation::return_tuple() const
{
    return m_output_tuple;
}
///std::vector<Tuple> TriMeshSplitEdgeOperation::triangle_onering() const
///{
///    Simplex v(PrimitiveType::Vertex, new_vertex());
///    auto sc = SimplicialComplex::open_star(v, m_mesh);
///    auto faces = sc.get_simplices(PrimitiveType::Face);
///    std::vector<Tuple> ret;
///    for (const auto& face : faces) {
///        ret.emplace_back(face.tuple());
///    }
///    return ret;
///}
///std::vector<Tuple> TriMeshSplitEdgeOperation::triangle_tworing() const
///{
///    throw "not implemented";
///    return {};
///}

// std::vector<Tuple> new_triangles() const  {
//     return triangle_onering();
// }
// std::vector<Tuple> new_edges() const  {
//     return spline_edges + rib_edges();
// }


// bool MySplitEdge::after()const {
//
//    //for(const auto& tri: modified_triangles()) {
//    //    positive_area_invariant(m,tri);
//    //}
//
//
//}
} // namespace wmtk
