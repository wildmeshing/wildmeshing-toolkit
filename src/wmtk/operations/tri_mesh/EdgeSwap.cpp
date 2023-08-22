#include "EdgeSwap.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
namespace wmtk::operations::tri_mesh {
EdgeSwap::EdgeSwap(wmtk::Mesh& m, const Tuple& t, const OperationSettings<EdgeSwap>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_settings{settings}
{}

std::string EdgeSwap::name() const
{
    return "tri_mesh_edge_swap";
}

bool EdgeSwap::before() const
{
    if (m_mesh.is_outdated(m_input_tuple)) {
        return false;
    }
    if (!m_mesh.is_valid(m_input_tuple)) {
        return false;
    }
    if (m_mesh.is_boundary(m_input_tuple)) {
        return false;
    }

    // do not allow swaps if one incident vertex has valence 3 (2 at boundary)
    const Tuple v0 = m_input_tuple;
    const Tuple v1 = m_mesh.switch_vertex(m_input_tuple);
    long val0 = static_cast<long>(
        SimplicialComplex::vertex_one_ring(dynamic_cast<TriMesh&>(m_mesh), v0).size());
    long val1 = static_cast<long>(
        SimplicialComplex::vertex_one_ring(dynamic_cast<TriMesh&>(m_mesh), v1).size());
    if (m_mesh.is_boundary_vertex(v0)) {
        ++val0;
    }
    if (m_mesh.is_boundary_vertex(v1)) {
        ++val1;
    }
    if (val0 < 4 || val1 < 4) {
        return false;
    }

    //     v2
    //    / \
    //  v0---v1
    //    \ /
    //     v3
    if (m_settings.must_improve_valence) {
        const Tuple v2 = m_mesh.switch_vertex(m_mesh.switch_edge(m_input_tuple));
        const Tuple v3 =
            m_mesh.switch_vertex(m_mesh.switch_edge(m_mesh.switch_face(m_input_tuple)));
        long val2 = static_cast<long>(
            SimplicialComplex::vertex_one_ring(dynamic_cast<TriMesh&>(m_mesh), v2).size());
        long val3 = static_cast<long>(
            SimplicialComplex::vertex_one_ring(dynamic_cast<TriMesh&>(m_mesh), v3).size());
        if (m_mesh.is_boundary_vertex(v2)) {
            val2 += 2;
        }
        if (m_mesh.is_boundary_vertex(v3)) {
            val3 += 2;
        }

        // formula from: https://github.com/daniel-zint/hpmeshgen/blob/cdfb9163ed92523fcf41a127c8173097e935c0a3/src/HPMeshGen2/TriRemeshing.cpp#L315
        const long val_before = std::max(std::abs(val0 - 6), std::abs(val1 - 6)) +
                                std::max(std::abs(val2 - 6), std::abs(val3 - 6));
        const long val_after = std::max(std::abs(val0 - 7), std::abs(val1 - 7)) +
                               std::max(std::abs(val2 - 5), std::abs(val3 - 5));

        return val_after < val_before;
    }

    return true;
}

Tuple EdgeSwap::return_tuple() const
{
    return m_output_tuple;
}

bool EdgeSwap::execute()
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
        OperationSettings<tri_mesh::EdgeSplit> op_settings;
        tri_mesh::EdgeSplit split_op(m_mesh, m_input_tuple, op_settings);
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
    tri_mesh::EdgeCollapse coll_op(m_mesh, coll_input_tuple);
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


} // namespace wmtk::operations::tri_mesh
