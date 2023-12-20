#pragma once
#include <optional>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class EdgeSplit;
}

template <>
struct OperationSettings<tet_mesh::EdgeSplit> : public OperationSettingsBase
{
    OperationSettings<tet_mesh::EdgeSplit>(TetMesh& m)
        : m_mesh(m)
    {}

    TetMesh& m_mesh;

    void create_invariants();
};

namespace tet_mesh {
/**
 * EdgeSplit
 *
 * This class just uses the TetMeshOperationExecutor' API, so please see
 * the class TetMeshOperationExecutor to see more details.
 *
 * return tuple: the return tuple is the original tuple, same face and edge,
 * arrowing to the new vertex
 */
class EdgeSplit : public TetMeshOperation, private TupleOperation
{
public:
    EdgeSplit(TetMesh& m, const Simplex& t, const OperationSettings<EdgeSplit>& settings);

    std::string name() const override;

    Tuple new_vertex() const;

    std::array<Tuple, 2> new_spine_edges() const;

    Tuple return_tuple() const;
    std::vector<Simplex> modified_primitives() const override;

    std::vector<Simplex> unmodified_primitives() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TetMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    const OperationSettings<EdgeSplit>& m_settings;
};

} // namespace tet_mesh
} // namespace wmtk::operations
