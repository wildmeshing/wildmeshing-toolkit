#include "TetEdgeSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::TetEdgeSplit>::initialize_invariants(const TetMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
}

bool OperationSettings<tet_mesh::TetEdgeSplit>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}

namespace tet_mesh {

// TetEdgeSplit::TetEdgeSplit(Mesh& m, const Tuple& t, const OperationSettings<TetEdgeSplit>&
// settings)
//     : TetMeshOperation(m)
//     , TupleOperation(settings.invariants, t)
//     , m_settings{settings}
// {
//     assert(m_settings.are_invariants_initialized());
// }

TetEdgeSplit::TetEdgeSplit(Mesh& m, const Tuple& t, const OperationSettings<TetEdgeSplit>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

// TetEdgeSplit::~TetEdgeSplit() = default;

bool TetEdgeSplit::execute()
{
    auto return_data = mesh().split_edge(input_tuple(), hash_accessor());
    m_output_tuple = return_data.m_output_tuple;

    return true;
}

std::string TetEdgeSplit::name() const
{
    return "tet_mesh_split_edge";
}

Tuple TetEdgeSplit::new_vertex() const
{
    return m_output_tuple;
}

Tuple TetEdgeSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> TetEdgeSplit::modified_primitives(PrimitiveType type) const
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
