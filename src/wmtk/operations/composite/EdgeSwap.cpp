#include "EdgeSwap.hpp"

namespace wmtk::operations::composite {
EdgeSwap::EdgeSwap(Mesh& m)
    : EdgeSwap(m, std::make_shared<EdgeSplit>(m), std::make_shared<EdgeCollapse>(m))
{}
EdgeSwap::EdgeSwap(
    Mesh& m,
    std::shared_ptr<EdgeSplit> split,
    std::shared_ptr<EdgeCollapse> collapse)
    : Operation(m)
    , m_split(std::move(split))
    , m_collapse(std::move(collapse))
{}
} // namespace wmtk::operations::composite
