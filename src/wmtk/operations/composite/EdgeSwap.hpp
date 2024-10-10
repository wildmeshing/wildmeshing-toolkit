#pragma once
#include <wmtk/operations/Operation.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::operations::composite {
    class EdgeSwap: public Operation {
        public:
            EdgeSwap(Mesh& m);

    inline EdgeSplit& split() { return m_split; }
    inline EdgeCollapse& collapse() { return m_collapse; }
protected:
    EdgeSplit m_split;
    EdgeCollapse m_collapse;
    };
}

