
#pragma once

#include "EdgeSwap.hpp"

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

class TriEdgeSwap : public EdgeSwap
{
public:
    TriEdgeSwap(TriMesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }


protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

};

} // namespace wmtk::operations::composite
