
#include "EdgeSplit.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/SimplicialComplex.hpp"

namespace wmtk::operations::tri_mesh {

EdgeSplit::EdgeSplit(wmtk::Mesh& m, const Tuple& t, const OperationSettings<EdgeSplit>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_settings{settings}
{}
bool EdgeSplit::execute()
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
bool EdgeSplit::before() const
{
    if (m_mesh.is_outdated(m_input_tuple) || !m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    if (!m_settings.split_boundary_edges && m_mesh.is_boundary(m_input_tuple)) {
        return false;
    }

    return true;
}

std::string EdgeSplit::name() const
{
    return "tri_mesh_edge_split";
}

Tuple EdgeSplit::new_vertex() const
{
    return m_output_tuple;
}

Tuple EdgeSplit::return_tuple() const
{
    return m_output_tuple;
}

} // namespace wmtk::operations::tri_mesh
