#include "TriMeshSwapEdgeOperation.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include "TriMeshCollapseEdgeOperation.hpp"
#include "TriMeshSplitEdgeOperation.hpp"
namespace wmtk {
TriMeshSwapEdgeOperation::TriMeshSwapEdgeOperation(TriMesh& m, const Tuple& t)
    : Operation(m)
    , m_input_tuple(t)
{}

std::string TriMeshSwapEdgeOperation::name() const
{
    return "TriMeshSwapEdgeOperation";
}

bool TriMeshSwapEdgeOperation::before() const
{
    if (m_mesh.is_outdated(m_input_tuple)) {
        return false;
    }
    if (m_mesh.is_boundary(m_input_tuple)) {
        return false;
    }

    // do not allow swaps if one incident vertex has valence 3 (2 at boundary)
    const Tuple v0 = m_input_tuple;
    const Tuple v1 = m_mesh.switch_vertex(m_input_tuple);
    size_t val0 = SimplicialComplex::vertex_one_ring(v0, m_mesh).size();
    size_t val1 = SimplicialComplex::vertex_one_ring(v1, m_mesh).size();
    if (m_mesh.is_vertex_boundary(v0)) {
        ++val0;
    }
    if (m_mesh.is_vertex_boundary(v1)) {
        ++val1;
    }
    if (val0 < 4 || val1 < 4) {
        return false;
    }
    if (!m_mesh.is_valid(m_input_tuple)) {
        return false;
    }
    // not allowing boundary edges to be swapped
    return (!m_mesh.is_boundary(m_input_tuple));
}

Tuple TriMeshSwapEdgeOperation::return_tuple() const
{
    return m_output_tuple;
}

bool TriMeshSwapEdgeOperation::execute()
{
    // input
    //    / \
    //   /   \
    //  /  f  \
    // X--->---
    //  \     /
    //   \   /
    //    \ /

    Tuple split_ret;
    {
        TriMeshSplitEdgeOperation split_op(m_mesh, m_input_tuple);
        if (!split_op()) {
            return false;
        }
        split_ret = split_op.return_tuple();
    }
    // after split
    //    /|\
    //   / | \
    //  /  | f\
    //  ---X-->
    //  \  |  /
    //   \ | /
    //    \|/

    // switch also face to keep edge orientation
    const Tuple coll_input_tuple = m_mesh.switch_face(m_mesh.switch_edge(split_ret));
    // switch edge - switch face
    //    /|\
    //   / ^ \
    //  /f |  \
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    TriMeshCollapseEdgeOperation coll_op(m_mesh, coll_input_tuple);
    if (!coll_op()) {
        return false;
    }
    const Tuple& coll_ret = coll_op.return_tuple();
    // collapse output
    //     X
    //    /|\
    //   < | \
    //  /  |  \
    // | f |   |
    //  \  |  /
    //   \ | /
    //    \|/
    // adjust return tuple to be the swapped edge in the same orientation as the input
    m_output_tuple = m_mesh.switch_vertex(m_mesh.switch_edge(coll_ret));

    return true;
}


} // namespace wmtk
