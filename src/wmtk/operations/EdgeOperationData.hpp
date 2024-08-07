#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations {
class EdgeOperationData
{
public:
    Tuple m_operating_tuple;

    Tuple m_output_tuple;
    std::array<int64_t, 2> m_spine_vids; // two endpoints of the edge
protected:
    static Tuple tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid);
    static simplex::Simplex simplex_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid);
};
} // namespace wmtk::operations
