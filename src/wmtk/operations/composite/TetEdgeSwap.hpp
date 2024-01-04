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
 */

class TetEdgeSwap : public Operation
{
public:
    TetEdgeSwap(Mesh& m, int64_t collapse_index = 0);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline EdgeSplit& split() { return m_split; }
    inline EdgeCollapse& collapse() { return m_collapse; }


protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    EdgeSplit m_split;
    EdgeCollapse m_collapse;
    int64_t m_collapse_index;
};


} // namespace wmtk::operations::composite