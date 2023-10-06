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
    return true;
}

Tuple FaceSplitWithTag::return_tuple() const
{
    return m_output_tuple;
}

bool FaceSplitWithTag::execute()
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

    // switch edge - switch face
    //    /|\
    //   / ^ \
    //  /f |  \
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    const Tuple second_split_input_tuple = mesh().switch_face(mesh().switch_edge(split_ret));
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
    const Tuple coll_input_tuple = split_ret;
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
    m_vertex_tag_accessor.vector_attribute(second_split_ret)(0) = m_settings.embedding_tag_value;
    m_edge_tag_accessor.vector_attribute(input_tuple())(0) = m_settings.input_tag_value;
    m_edge_tag_accessor.vector_attribute(mesh().switch_edge(input_tuple()))(0) =
        m_settings.embedding_tag_value;
    m_edge_tag_accessor.vector_attribute(mesh().switch_edge(mesh().switch_edge(input_tuple())))(0) =
        m_settings.embedding_tag_value;
    m_edge_tag_accessor.vector_attribute(second_split_ret)(0) = m_settings.embedding_tag_value;

    // return new vertex's tuple
    m_output_tuple = second_split_ret;
    return true;
}


} // namespace wmtk::operations::tri_mesh
