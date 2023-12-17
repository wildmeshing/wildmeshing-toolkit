#include "EdgeOperationData.hpp"
#include <wmtk/EdgeMesh.hpp>
namespace wmtk::operations::edge_mesh {
std::array<Tuple, 2> EdgeOperationData::input_endpoints(const EdgeMesh& m) const
{
    std::array<Tuple, 2> r;
    r[0] = m_operating_tuple;
    r[1] = m.switch_tuple(m_operating_tuple, PrimitiveType::Vertex);
    return r;
}
}
