#pragma once
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
#include "TetMeshOperation.hpp"

namespace wmtk::operations {

namespace tet_mesh {
class EdgeSwap;
}

template <>
struct OperationSettings<tet_mesh::EdgeSwap> : public OperationSettingsBase
{
    OperationSettings<tet_mesh::EdgeSwap>(TetMesh& m)
        : m_mesh(m)
        , collapse_settings(m)
        , split_settings(m)
    {}

    TetMesh& m_mesh;

    OperationSettings<tet_mesh::EdgeCollapse> collapse_settings;
    OperationSettings<tet_mesh::EdgeSplit> split_settings;

    void create_invariants();
};

namespace tet_mesh {
/**
 * Performs an edge swap, implemented as a combination of swap and collapse.
 *
 * When swapping an edge, one first split the edge (that splits all the tets incident to that edge),
 * then collapse the edge with index collapse_index.)
 *
 * Edge swap cannot be performed on boundary edges.
 *
 */

class EdgeSwap : public TetMeshOperation, protected TupleOperation
{
public:
    EdgeSwap(
        Mesh& m,
        const Simplex& t,
        int collapse_index,
        const OperationSettings<EdgeSwap>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    std::vector<Simplex> modified_primitives() const;

    std::vector<Simplex> unmodified_primitives() const override;

    std::vector<Tuple> new_tets_after_swap() const;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwap>& m_settings;
    int m_collapse_index;
    std::vector<Tuple> m_new_tets;
};

} // namespace tet_mesh
} // namespace wmtk::operations