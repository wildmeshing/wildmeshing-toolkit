#pragma once
#include <optional>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class TetSplit;
}

template <>
struct OperationSettings<tet_mesh::TetSplit>
{
    InvariantCollection invariants;
    void initialize_invariants(const TetMesh& m);
    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tet_mesh {
/**
 * TetSplit
 *
 * This class is used to split a tetrahedra into 4 parts, we did this
 * by only using two atomic operations, EdgeSplit and EdgeCollapse
 * in order to make our lib more reliable.
 *
 * return tuple: the return tuple is the original tuple, arrowing to the new vertex,
 * the face belongs to the original (original_vertex,new vertex,switch_vertex(original_vertex))
 * the tetrahedra is the most front one.
 */
class TetSplit : public TetMeshOperation, private TupleOperation
{
public:
    TetSplit(TetMesh& m, const Tuple& t, const OperationSettings<TetSplit>& settings);

    std::string name() const override;

    Tuple new_vertex() const;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TetMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    const OperationSettings<TetSplit>& m_settings;
};

} // namespace tet_mesh
} // namespace wmtk::operations
