
#pragma once

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::operations::composite {
/**
 * Performs an edge swap, implemented as a combination of swap and collapse.
 *
 * There are no explicit checks for valence. However, the collapse checks implicitly for
 * validity of the swap. The swap will be not performed if the collapse does not fulfill the
 * link condition.
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

class TriEdgeSwap : public Operation
{
public:
    TriEdgeSwap(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline EdgeSplit& split() { return m_split; }
    inline EdgeCollapse& collapse() { return m_collapse; }

protected:
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
    std::vector<Simplex> execute(const Simplex& simplex) override;

private:
    EdgeSplit m_split;
    EdgeCollapse m_collapse;
};

} // namespace wmtk::operations::composite
