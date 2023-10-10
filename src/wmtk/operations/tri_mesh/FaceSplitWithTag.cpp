#include "FaceSplitWithTag.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
namespace wmtk::operations::tri_mesh {
FaceSplitWithTag::FaceSplitWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<FaceSplitWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_vertex_tag_accessor{m.create_accessor(settings.vertex_tag)}
    , m_edge_tag_accessor{m.create_accessor(settings.edge_tag)}
    , m_settings{settings}
{}

std::string FaceSplitWithTag::name() const
{
    return "face_split_with_tag";
}

bool FaceSplitWithTag::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    if (mesh().is_boundary(input_tuple())) {
        return false;
    }
    long t0, t1, t2;
    t0 = m_edge_tag_accessor.vector_attribute(input_tuple())(0);
    t1 = m_edge_tag_accessor.vector_attribute(mesh().switch_edge(input_tuple()))(0);
    t2 = m_edge_tag_accessor.vector_attribute(
        mesh().switch_edge(mesh().switch_edge(input_tuple())))(0);
    if (t0 != m_settings.input_tag_value || t1 != m_settings.input_tag_value ||
        t2 != m_settings.input_tag_value) {
        return false;
    }
    return true;
}

Tuple FaceSplitWithTag::return_tuple() const
{
    return m_output_tuple;
}

bool FaceSplitWithTag::execute()
{
    Eigen::Vector3d p0 = m_pos_accessor.const_vector_attribute(input_tuple());
    Eigen::Vector3d p1 = m_pos_accessor.const_vector_attribute(mesh().switch_edge(input_tuple()));
    Eigen::Vector3d p2 = m_pos_accessor.const_vector_attribute(
        mesh().switch_edge(mesh().switch_edge(input_tuple())));
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

    // switch edge - switch face
    //    /|\
    //   / v \
    //  /f |  \
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    const Tuple second_split_input_tuple = mesh().switch_edge(split_ret);
    Tuple second_split_ret;
    {
        OperationSettings<tri_mesh::EdgeSplit> op_settings;
        op_settings.initialize_invariants(mesh());
        tri_mesh::EdgeSplit split_op(mesh(), second_split_input_tuple, op_settings);
        if (!split_op()) {
            return false;
        }
        second_split_ret = split_op.return_tuple();
    }
    // after split
    //      ^
    //     /|\
    //    / | \
    //   /  X  \
    //  /  /|\  \
    // /__/_|_\__\
    //  \   |   /
    //   \  |  /
    //    \ | /
    //     \|/


    // collapse the split ret
    //     /|\
    //    / | \
    //   / /|\ \
    //  / / | \ \
    //  |/__X__\>
    //  \   |   /
    //   \  |  /
    //    \ | /
    //     \|/
    const Tuple coll_input_tuple = mesh().switch_edge(mesh().switch_edge(second_split_ret));
    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;

    collapse_settings.initialize_invariants(mesh());
    tri_mesh::EdgeCollapse coll_op(mesh(), coll_input_tuple, collapse_settings);
    if (!coll_op()) {
        return false;
    }
    const Tuple& coll_ret = coll_op.return_tuple();
    // collapse output
    //     /|\
    //    / | \
    //   / / \ \
    //  / /   \ \
    //  |/ _ _ \|
    //  \       /
    //   \     /
    //    \   /
    //     \ /

    // set attributes
    m_vertex_tag_accessor.vector_attribute(coll_ret)(0) = m_settings.embedding_tag_value;
    m_edge_tag_accessor.vector_attribute(coll_ret)(0) = m_settings.input_tag_value;
    m_edge_tag_accessor.vector_attribute(mesh().switch_edge(coll_ret))(0) =
        m_settings.embedding_tag_value;
    m_edge_tag_accessor.vector_attribute(mesh().switch_edge(mesh().switch_face(coll_ret)))(0) =
        m_settings.input_tag_value;
    m_edge_tag_accessor.vector_attribute(mesh().switch_edge(
        mesh().switch_edge(mesh().switch_face(coll_ret))))(0) = m_settings.embedding_tag_value;

    // return new vertex's tuple
    m_output_tuple = coll_ret;
    m_pos_accessor.vector_attribute(m_output_tuple) = (p0 + p1 + p2) / 3.0;
    return true;
}


} // namespace wmtk::operations::tri_mesh
