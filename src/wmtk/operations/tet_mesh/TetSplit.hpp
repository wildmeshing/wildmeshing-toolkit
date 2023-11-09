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
class TetSplit : public TetMeshOperation, private TupleOperation
{
public:
    // TetSplit(Mesh& m, const Tuple& t, const OperationSettings<TetSplit>& settings);
    TetSplit(TetMesh& m, const Tuple& t, const OperationSettings<TetSplit>& settings);

    std::string name() const override;

    Tuple new_vertex() const;

    /**
     * the return tuple is the original tuple, arrowing to the new vertex
     */
    Tuple return_tuple() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

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
