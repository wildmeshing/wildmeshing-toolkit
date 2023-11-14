#include "TetSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tet_mesh::TetSplit>::initialize_invariants(const TetMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
}

bool OperationSettings<tet_mesh::TetSplit>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}

namespace tet_mesh {

TetSplit::TetSplit(TetMesh& m, const Tuple& t, const OperationSettings<TetSplit>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool TetSplit::execute()
{
    Tuple first_split_ret;
    {
        OperationSettings<EdgeSplit> settings;
        settings.initialize_invariants(mesh());
        EdgeSplit op(mesh(), input_tuple(), settings);
        if (!op()) {
            return false;
        }
        first_split_ret = op.return_tuple();
    }


    Tuple second_split_ret;
    {
        OperationSettings<EdgeSplit> settings;
        settings.initialize_invariants(mesh());
        EdgeSplit op(mesh(), mesh().switch_edge(mesh().switch_vertex(first_split_ret)), settings);
        if (!op()) {
            return false;
        }
        second_split_ret = op.return_tuple();
    }

    Tuple third_split_ret;
    {
        OperationSettings<EdgeSplit> settings;
        settings.initialize_invariants(mesh());
        EdgeSplit op(
            mesh(),
            mesh().switch_edge(mesh().switch_vertex(mesh().switch_face(second_split_ret))),
            settings);
        if (!op()) {
            return false;
        }
        third_split_ret = op.return_tuple();
    }

    Tuple first_collapse_ret;
    {
        OperationSettings<EdgeCollapse> settings;
        settings.initialize_invariants(mesh());
        EdgeCollapse op(mesh(), mesh().switch_face(mesh().switch_edge(third_split_ret)), settings);
        if (!op()) {
            return false;
        }
        first_collapse_ret = op.return_tuple();
    }

    Tuple second_collapse_ret;
    {
        OperationSettings<EdgeCollapse> settings;
        settings.initialize_invariants(mesh());
        EdgeCollapse op(mesh(), first_collapse_ret, settings);
        if (!op()) {
            return false;
        }
        second_collapse_ret = op.return_tuple();
    }

    m_output_tuple = mesh().switch_face(mesh().switch_tetrahedron(second_collapse_ret));

    return true;
}

std::string TetSplit::name() const
{
    return "tet_mesh_split_tet";
}

Tuple TetSplit::new_vertex() const
{
    return m_output_tuple;
}

Tuple TetSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> TetSplit::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        // TODO
        // return modified_triangles();
    } else if (type == PrimitiveType::Vertex) {
        return {new_vertex()};
    }
    return {};
}
} // namespace tet_mesh
} // namespace wmtk::operations
