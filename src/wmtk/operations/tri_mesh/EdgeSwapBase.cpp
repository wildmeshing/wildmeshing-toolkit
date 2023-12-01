#include "EdgeSwapBase.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>

#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::EdgeSwapBase>::initialize_invariants(const TriMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
    invariants.add(std::make_shared<InteriorEdgeInvariant>(m));

    collapse_settings.initialize_invariants(m);
    split_settings.initialize_invariants(m);
}


namespace tri_mesh {
EdgeSwapBase::EdgeSwapBase(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwapBase>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{}

std::string EdgeSwapBase::name() const
{
    return "tri_mesh_edge_swap_base";
}

Tuple EdgeSwapBase::return_tuple() const
{
    return m_output_tuple;
}

bool EdgeSwapBase::execute()
{
    // input
    //    / \ .
    //   /   \ .
    //  /  f  \ .
    // X--->---
    //  \     /
    //   \   /
    //    \ /

    Tuple split_ret;
    {
        tri_mesh::EdgeSplit split_op(mesh(), input_tuple(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        split_ret = split_op.return_tuple();
    }
    // after split
    //    /|\ .
    //   / | \ .
    //  /  | f\ .
    //  ---X-->
    //  \  |  /
    //   \ | /
    //    \|/

    // switch also face to keep edge orientation
    const Tuple coll_input_tuple = mesh().switch_face(mesh().switch_edge(split_ret));
    // switch edge - switch face
    //    /|\ .
    //   / ^ \ .
    //  /f |  \ .
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    Tuple coll_ret;
    {
        tri_mesh::EdgeCollapse coll_op(mesh(), coll_input_tuple, m_settings.collapse_settings);
        if (!coll_op()) {
            return false;
        }
        coll_ret = coll_op.return_tuple();
    }
    // collapse output
    //     X
    //    /|\ .
    //   < | \ .
    //  /  |  \ .
    // | f |   |
    //  \  |  /
    //   \ | /
    //    \|/
    // adjust return tuple to be the swapped edge in the same orientation as the input
    m_output_tuple = mesh().switch_vertex(mesh().switch_edge(coll_ret));

    return true;
}

} // namespace tri_mesh
} // namespace wmtk::operations
