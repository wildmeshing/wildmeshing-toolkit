#pragma once

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::operations::composite {

/**
 * Performs an edge swap, implemented as a combination of swap and collapse.
 *
 * When swapping an edge, one first split the edge (that splits all the tets incident to that edge),
 * then collapse the edge with index collapse_index.)
 *
 * Edge swap cannot be performed on boundary edges.
 *
 *
 *        0 ---------- 4
 *       / \\        /
 *      /   \ \     /
 *     /     \  \  /
 *    /       \  \\ 3
 *  1 --------- 2/         input: (0, 0-2, 0-2-3, 0-2-3-1)
 *
 *
 *        0 ---------- 4
 *       / \\     _- /
 *      /   \ \ _-  /
 *     /   __\- \  /
 *    /_--    \  \\ 3
 *  1 --------- 2/         output: (4, 4-1, 4-1-3, 4-1-3-2)
 *
 *
 */

class TetEdgeSwap : public Operation
{
public:
    TetEdgeSwap(Mesh& m, long collapse_index = 0);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline EdgeSplit& split() { return m_split; }
    inline EdgeCollapse& collapse() { return m_collapse; }


protected:
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
    std::vector<Simplex> execute(const Simplex& simplex) override;

private:
    EdgeSplit m_split;
    EdgeCollapse m_collapse;
    long m_collapse_index;
};


} // namespace wmtk::operations::composite