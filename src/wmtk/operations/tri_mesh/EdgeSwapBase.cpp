#include "EdgeSwapBase.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>

#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::EdgeSwapBase>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
    invariants->add(std::make_shared<InteriorEdgeInvariant>(m_mesh));

    collapse_settings.create_invariants();
    split_settings.create_invariants();
}


namespace tri_mesh {
EdgeSwapBase::EdgeSwapBase(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<EdgeSwapBase>& settings)
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
        tri_mesh::EdgeSplit split_op(mesh(), input_simplex(), m_settings.split_settings);
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
        tri_mesh::EdgeCollapse coll_op(
            mesh(),
            Simplex::edge(coll_input_tuple),
            m_settings.collapse_settings);
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

std::vector<Simplex> EdgeSwapBase::modified_primitives() const
{
    std::vector<Simplex> s;
    s.emplace_back(simplex::Simplex::edge(m_output_tuple));
    return s;
}

} // namespace tri_mesh
} // namespace wmtk::operations
