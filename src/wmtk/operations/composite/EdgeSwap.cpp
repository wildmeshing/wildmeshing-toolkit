#include "EdgeSwap.hpp"

namespace wmtk::operations::composite {
    EdgeSwap::EdgeSwap(Mesh& m): Operation(m), m_split(m), m_collapse(m) {}
}
