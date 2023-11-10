#pragma once
#include <optional>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class TetEdgeSplit;
}

template <>
struct OperationSettings<tet_mesh::TetEdgeSplit>
{
    InvariantCollection invariants;
    void initialize_invariants(const TetMesh& m);
    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tet_mesh {
/**
 * TetEdgeSplit
 *
 * This class just uses the TetMeshOperationExecutor' API, so please see
 * the class TetMeshOperationExecutor to see more details.
 *
 * return tuple: the return tuple is the original tuple, same face and edge,
 * arrowing to the new vertex
 */
class TetEdgeSplit : public TetMeshOperation, private TupleOperation
{
public:
    // TetEdgeSplit(Mesh& m, const Tuple& t, const OperationSettings<TetEdgeSplit>& settings);
    TetEdgeSplit(Mesh& m, const Tuple& t, const OperationSettings<TetEdgeSplit>& settings);

    std::string name() const override;

    Tuple new_vertex() const;


    Tuple return_tuple() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TetMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    const OperationSettings<TetEdgeSplit>& m_settings;
};

} // namespace tet_mesh
} // namespace wmtk::operations