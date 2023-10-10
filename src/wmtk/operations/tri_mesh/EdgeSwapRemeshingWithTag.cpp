#include "EdgeSwapRemeshingWithTag.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
namespace wmtk::operations::tri_mesh {
EdgeSwapRemeshingWithTag::EdgeSwapRemeshingWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSwapRemeshingWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
    , m_pos_accessor(m.create_accessor(m_settings.position))
    , m_vertex_tag_accessor(m.create_accessor(m_settings.vertex_tag))
    , m_edge_tag_accessor(m.create_accessor(m_settings.edge_tag))
{}

std::string EdgeSwapRemeshingWithTag::name() const
{
    return "edge_swap_remeshing_with_tag";
}

bool EdgeSwapRemeshingWithTag::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    if (mesh().is_boundary(input_tuple())) {
        return false;
    }

    long vt0 = m_vertex_tag_accessor.const_vector_attribute(input_tuple())(0);
    long vt1 = m_vertex_tag_accessor.const_vector_attribute(mesh().switch_vertex(input_tuple()))(0);
    if (vt0 == m_settings.offset_tag_value && vt1 == m_settings.offset_tag_value) {
        return false;
    }
    if (vt0 == m_settings.input_tag_value && vt1 == m_settings.input_tag_value) {
        return false;
    }

    if ((vt0 == m_settings.input_tag_value && vt1 == m_settings.offset_tag_value) ||
        (vt1 == m_settings.input_tag_value && vt0 == m_settings.offset_tag_value)) {
        auto p0 = m_pos_accessor.const_vector_attribute(input_tuple());
        auto p1 = m_pos_accessor.const_vector_attribute(mesh().switch_vertex(input_tuple()));
        double l0 = (p0 - p1).norm();
        auto p2 = m_pos_accessor.const_vector_attribute(
            mesh().switch_vertex(mesh().switch_edge(input_tuple())));
        auto p3 = m_pos_accessor.const_vector_attribute(
            mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(input_tuple()))));
        double l1 = (p2 - p3).norm();
        return l0 > l1;
    }

    // do not allow swaps if one incident vertex has valence 3 (2 at boundary)
    const Tuple v0 = input_tuple();
    const Tuple v1 = mesh().switch_vertex(input_tuple());
    long val0 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v0).size());
    long val1 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v1).size());
    if (mesh().is_boundary_vertex(v0)) {
        ++val0;
    }
    if (mesh().is_boundary_vertex(v1)) {
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
        const Tuple v2 = mesh().switch_vertex(mesh().switch_edge(input_tuple()));
        const Tuple v3 =
            mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(input_tuple())));
        long val2 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v2).size());
        long val3 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v3).size());
        if (mesh().is_boundary_vertex(v2)) {
            val2 += 2;
        }
        if (mesh().is_boundary_vertex(v3)) {
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

Tuple EdgeSwapRemeshingWithTag::return_tuple() const
{
    return m_output_tuple;
}

bool EdgeSwapRemeshingWithTag::execute()
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
        op_settings.initialize_invariants(mesh());
        tri_mesh::EdgeSplit split_op(mesh(), input_tuple(), op_settings);
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
    const Tuple coll_input_tuple = mesh().switch_face(mesh().switch_edge(split_ret));
    // switch edge - switch face
    //    /|\
    //   / ^ \
    //  /f |  \
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/

    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;

    collapse_settings.initialize_invariants(mesh());
    tri_mesh::EdgeCollapse coll_op(mesh(), coll_input_tuple, collapse_settings);
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
    m_output_tuple = mesh().switch_vertex(mesh().switch_edge(coll_ret));

    return true;
}


} // namespace wmtk::operations::tri_mesh
