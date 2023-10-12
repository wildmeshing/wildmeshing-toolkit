#include "FaceSplit.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::FaceSplit>::initialize_invariants(const TriMesh& m)
{
    invariants = basic_invariant_collection(m);
}

bool OperationSettings<tri_mesh::FaceSplit>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}
namespace tri_mesh {

FaceSplit::FaceSplit(Mesh& m, const Tuple& t, const OperationSettings<FaceSplit>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{}

std::string FaceSplit::name() const
{
    return "face_split_with_tag";
}

bool FaceSplit::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }

    return true;
}

Tuple FaceSplit::return_tuple() const
{
    return m_output_tuple;
}

bool FaceSplit::execute()
{
    Eigen::Vector3d p0 = m_pos_accessor.const_vector_attribute(input_tuple());
    Eigen::Vector3d p1 = m_pos_accessor.const_vector_attribute(
        mesh().switch_edge(mesh().switch_vertex(input_tuple())));
    Eigen::Vector3d p2 = m_pos_accessor.const_vector_attribute(mesh().switch_vertex(input_tuple()));
    // input
    //     p1
    //    / \
    //   / f \
    //  /     \
    // p0-->--p2
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
    const Tuple second_split_input_tuple = mesh().switch_vertex(mesh().switch_edge(split_ret));
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
    //
    //     /|\
    //    / | \
    //   /  X  \
    //  /  /|\  \
    // /__/_v_\__\
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
    const Tuple coll_input_tuple = mesh().switch_edge(mesh().switch_vertex(second_split_ret));
    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;

    collapse_settings.initialize_invariants(mesh());
    tri_mesh::EdgeCollapse coll_op(mesh(), coll_input_tuple, collapse_settings);
    if (!coll_op()) {
        return false;
    }
    const Tuple& coll_ret = coll_op.return_tuple();
    // collapse output
    //     /| \
    //    / |  \
    //   /  * f \
    //  / /   \  \
    // / /      > \
    // |/_ _ _ _ \|
    //  \       /
    //   \     /
    //    \   /
    //     \ /

    // return new vertex's tuple
    m_output_tuple = coll_ret;
    return true;
}


} // namespace tri_mesh
} // namespace wmtk::operations