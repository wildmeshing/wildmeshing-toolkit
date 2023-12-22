#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations {
class EdgeOperationData
{
public:
    Tuple m_operating_tuple;

    Tuple m_output_tuple;
    std::array<long, 2> m_spine_vids; // two endpoints of the edge
protected:
    static Tuple tuple_from_id(const Mesh& m, const PrimitiveType type, const long gid);
};
} // namespace wmtk::operations
