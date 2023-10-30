
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwapBase;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwapBase>
{
    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;
    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    InvariantCollection invariants;
    void initialize_invariants(const TriMesh& m);
};

namespace tri_mesh {
/**
 * Performs an edge swap, implemented as a combination of swap and collapse.
 *
 * There are no explicit checks for valence. However, the collapse checks implicitly for validity of
 * the swap. The swap will be not performed if the collapse does not fulfill the link condition.
 *
 * The edge swap cannot be performed on boundary edges.
 *
 * input:
 *     .
 *    / \
 *   /   \
 *  /  f  \
 * X--->---.
 *  \     /
 *   \   /
 *    \ /
 *     .
 *
 * output:
 *     .
 *    /|\
 *   / | \
 *  /  |  \
 * . f ^   .
 *  \  |  /
 *   \ | /
 *    \|/
 *     X
 */
class EdgeSwapBase : public TriMeshOperation, protected TupleOperation
{
public:
    EdgeSwapBase(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwapBase>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwapBase>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
