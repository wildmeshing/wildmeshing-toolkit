#pragma once
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/Operation.hpp>

namespace wmtk::operations::composite {
class EdgeSwap : public Operation
{
public:
    EdgeSwap(Mesh& m);
    EdgeSwap(Mesh& m, std::shared_ptr<EdgeSplit> split, std::shared_ptr<EdgeCollapse> collapse);

    EdgeSplit& split() { return *m_split; }
    EdgeCollapse& collapse() { return *m_collapse; }
    const EdgeSplit& split() const { return *m_split; }
    const EdgeCollapse& collapse() const { return *m_collapse; }

protected:
    std::shared_ptr<EdgeSplit> m_split;
    std::shared_ptr<EdgeCollapse> m_collapse;
};
} // namespace wmtk::operations::composite

