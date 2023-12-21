
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

class TriEdgeSwap : public EdgeSplit, public EdgeCollapse
{
public:
    TriEdgeSwap(Mesh& m);


protected:
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
    std::vector<Simplex> execute(const Simplex& simplex) override;
};

} // namespace wmtk::operations::composite
