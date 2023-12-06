
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeCollapseToMidpoint.hpp"
#include "EdgeSplitAtMidpoint.hpp"
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwapSafe;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwapSafe>
{
    OperationSettings<tri_mesh::EdgeCollapseToMidpoint> collapse_settings;
    OperationSettings<tri_mesh::EdgeSplitAtMidpoint> split_settings;
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
class EdgeSwapSafe : public TriMeshOperation, protected TupleOperation
{
public:
    EdgeSwapSafe(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwapSafe>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwapSafe>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
