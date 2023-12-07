
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class EdgeCollapse;
}

template <>
struct OperationSettings<tet_mesh::EdgeCollapse>
{
    OperationSettings();
    InvariantCollection invariants;
    void initialize_invariants(const TetMesh& m);
    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized(std::shared_ptr<InvariantCollection> inv_col_ptr) const;
};

namespace tet_mesh {
/**
 * EdgeCollapse
 *
 * This class just uses the TetMeshOperationExecutor' API, so please see
 * the class TetMeshOperationExecutor to see more details.
 *
 * return tuple: return next-->opposite tuple if it exists, otherwise return previous-->opposite
 */
class EdgeCollapse : public TetMeshOperation, private TupleOperation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(TetMesh& m, const Tuple& t, const OperationSettings<EdgeCollapse>& settings);

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TetMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
};

} // namespace tet_mesh
} // namespace wmtk::operations
